/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/World.h>

#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

struct WallProperties
{
	double height = 2.0;
	double thickness = 0.10;
};

static Block::Ptr create_wall_segment(
	World* parent, const mrpt::math::TPoint3D& rawEnd1, const mrpt::math::TPoint3D& rawEnd2,
	const WallProperties& wp, mrpt::maps::CSimplePointsMap& allPts)
{
	Block::Ptr b = std::make_shared<Block>(parent);

	{
		static int cnt = 0;
		b->setName(mrpt::format("wall_%04i", ++cnt));
	}

	float pt1Dist = std::numeric_limits<float>::max(), pt2Dist = std::numeric_limits<float>::max();

	if (!allPts.empty())
	{
		allPts.kdTreeClosestPoint2D(rawEnd1.x, rawEnd1.y, pt1Dist);
		allPts.kdTreeClosestPoint2D(rawEnd2.x, rawEnd2.y, pt2Dist);
	}

	// Do we need to move these points due to former points already there, to
	// avoid "collisions"?
	const bool end1move = pt1Dist < mrpt::square(1.01 * wp.thickness);
	const bool end2move = pt2Dist < mrpt::square(1.01 * wp.thickness);

	const auto vec12 = rawEnd2 - rawEnd1;
	ASSERT_(vec12.norm() > 0);
	const auto u12 = vec12.unitarize();

	const double t_hf = 0.5 * wp.thickness;
	const double t = 0.5 * wp.thickness;

	const auto end1 = end1move ? (rawEnd1 + u12 * (-t)) : rawEnd1;
	const auto end2 = end2move ? (rawEnd2 + u12 * t) : rawEnd2;

	const auto ptCenter = (end1 + end2) * 0.5;
	const double wallLineAngle = std::atan2(vec12.y, vec12.x);

	allPts.insertPoint(rawEnd1);
	allPts.insertPoint(rawEnd2);

	// initial pose:
	b->setPose({ptCenter.x, ptCenter.y, 0, wallLineAngle, 0, 0});

	// Shape:
	{
		const double l_hf = 0.5 * (end2 - end1).norm();

		ASSERT_(l_hf > 0);
		ASSERT_(t_hf > 0);

		auto bbmin = mrpt::math::TPoint3D(-l_hf, -t_hf, 0);
		auto bbmax = mrpt::math::TPoint3D(+l_hf, +t_hf, 0);

		mrpt::math::TPolygon2D p;
		p.emplace_back(bbmin.x, bbmin.y);
		p.emplace_back(bbmin.x, bbmax.y);
		p.emplace_back(bbmax.x, bbmax.y);
		p.emplace_back(bbmax.x, bbmin.y);

		b->blockShape(p);

		b->block_z_min(.0);
		b->block_z_max(wp.height);
	}

	// make the walls non-movable:
	b->ground_friction(1e5);
	b->mass(1e5);

	// Register bodies, fixtures, etc. in Box2D simulator:
	// ----------------------------------------------------
	b->create_multibody_system(*parent->getBox2DWorld());
	// This makes the walls non-mobile:
	b->setIsStatic(true);

	if (auto bb = b->b2d_body(); bb != nullptr)
	{
		// Init pos:
		const auto q = b->getPose();

		bb->SetTransform(b2Vec2(q.x, q.y), q.yaw);
		// Init vel:
		bb->SetLinearVelocity({0, 0});
		bb->SetAngularVelocity(0);
	}

	return b;
}

void World::process_load_walls(const rapidxml::xml_node<char>& node)
{
	MRPT_START

	// Sanity checks:
	ASSERT_(0 == strcmp(node.name(), "walls"));

	std::string wallModelFileName;
	std::string sTransformation;
	double scale = 1.0;
	mrpt::img::TColor wallColor{0x32, 0x32, 0x32, 0xff};
	WallProperties wp;

	TParameterDefinitions params;
	params["model_uri"] = TParamEntry("%s", &wallModelFileName);
	params["transformation"] = TParamEntry("%s", &sTransformation);
	params["wallThickness"] = TParamEntry("%lf", &wp.thickness);
	params["wallHeight"] = TParamEntry("%lf", &wp.height);
	params["scale"] = TParamEntry("%lf", &scale);
	params["color"] = TParamEntry("%color", &wallColor);

	// Parse XML params:
	parse_xmlnode_children_as_param(node, params, user_defined_variables());

	// Optional transformation:
	auto HM = mrpt::math::CMatrixDouble44::Identity();
	if (!sTransformation.empty())
	{
		std::stringstream ssError;
		bool ok = HM.fromMatlabStringFormat(sTransformation, ssError);
		if (!ok)
			THROW_EXCEPTION_FMT(
				"Error parsing 'transformation=\"%s\"' parameter of walls:\n%s",
				sTransformation.c_str(), ssError.str().c_str());
		MRPT_LOG_DEBUG_STREAM("Walls: using transformation: " << HM.asString());
	}
	const auto tf = mrpt::poses::CPose3D(HM);

	// Walls shape can come from external model file, or from a "shape" entry:
	const auto* xml_shape = node.first_node("shape");

	// Final coordinates for wall perimeter are defined here:
	std::vector<mrpt::math::TPoint3Df> tfPts;

	if (xml_shape)
	{
		// Load wall segments from "<shape>" tag.
		mrpt::math::TPolygon2D segments;

		mvsim::parse_xmlnode_shape(*xml_shape, segments, "[World::process_load_walls]");

		MRPT_LOG_DEBUG_STREAM("Walls loaded from <shape> tag, " << segments.size() << " segments.");

		// Transform them:
		tfPts.reserve(segments.size());
		for (const auto& pt : segments)
			tfPts.emplace_back(tf.composePoint(mrpt::math::TPoint3D(pt)));
	}
	else
	{
		// Load wall segments from external file.
		ASSERT_(!wallModelFileName.empty());
		const std::string localFileName = xmlPathToActualPath(wallModelFileName);
		ASSERT_FILE_EXISTS_(localFileName);

		MRPT_LOG_DEBUG_STREAM("Loading walls definition model from: " << localFileName);

		auto glModel = mrpt::opengl::CAssimpModel::Create();
		glModel->loadScene(localFileName);

		const auto& points = glModel->shaderWireframeVertexPointBuffer();
		MRPT_LOG_DEBUG_STREAM("Walls loaded from model file, " << points.size() << " segments.");

		// Transform them:
		tfPts.reserve(points.size());
		for (const auto& pt : points) tfPts.emplace_back(tf.composePoint(pt));
	}

	// Insert all points for KD-tree lookup:
	mrpt::maps::CSimplePointsMap ptsMap;

	// Create walls themselves:
	ASSERT_(tfPts.size() % 2 == 0);
	for (size_t i = 0; i < tfPts.size() / 2; i++)
	{
		const auto& pt1 = tfPts[i * 2 + 0];
		const auto& pt2 = tfPts[i * 2 + 1];
		auto block = create_wall_segment(this, pt1, pt2, wp, ptsMap);
		block->block_color(wallColor);
		insertBlock(block);
	}

	MRPT_END
}
