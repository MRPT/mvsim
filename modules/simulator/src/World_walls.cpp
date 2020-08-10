/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/World.h>

#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

void World::processLoadWalls(const rapidxml::xml_node<char>& node)
{
	MRPT_START

	// Sanity checks:
	ASSERT_(0 == strcmp(node.name(), "walls"));

	std::string wallModelFileName;
	std::string sTransformation;
	double wallThickness = 0.10;
	double scale = 1.0;

	TParameterDefinitions params;
	params["model_uri"] = TParamEntry("%s", &wallModelFileName);
	params["transformation"] = TParamEntry("%s", &sTransformation);
	params["wallThickness"] = TParamEntry("%lf", &wallThickness);
	params["scale"] = TParamEntry("%lf", &scale);

	// Parse XML params:
	parse_xmlnode_children_as_param(node, params);

	ASSERT_(!wallModelFileName.empty());
	const std::string localFileName = xmlPathToActualPath(wallModelFileName);
	ASSERT_FILE_EXISTS_(localFileName);

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

	MRPT_LOG_DEBUG_STREAM(
		"Loading walls definition model from: " << localFileName);

	auto glModel = mrpt::opengl::CAssimpModel::Create();
	glModel->loadScene(localFileName);

	const auto& points = glModel->shaderWireframeVertexPointBuffer();
	MRPT_LOG_DEBUG_STREAM("Walls loaded, " << points.size() << " segments.");

	// Transform them:
	std::vector<mrpt::math::TPoint3Df> tfPts;
	tfPts.reserve(points.size());
	for (const auto& pt : points) tfPts.emplace_back(tf.composePoint(pt));

	// Create walls themselves:
	MRPT_TODO("Continue");

	MRPT_END
}
