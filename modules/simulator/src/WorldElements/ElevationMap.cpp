/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/tfest.h>	 // least-squares methods
#include <mrpt/version.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/ElevationMap.h>

#include <limits>
#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

ElevationMap::ElevationMap(World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent)
{
	ElevationMap::loadConfigFrom(root);
}

ElevationMap::~ElevationMap() {}
void ElevationMap::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	// Other general params:
	TParameterDefinitions params;

	std::string sElevationImgFile;
	params["elevation_image"] = TParamEntry("%s", &sElevationImgFile);
	std::string sTextureImgFile;
	params["texture_image"] = TParamEntry("%s", &sTextureImgFile);

	double img_min_z = 0.0, img_max_z = 5.0;
	params["elevation_image_min_z"] = TParamEntry("%lf", &img_min_z);
	params["elevation_image_max_z"] = TParamEntry("%lf", &img_max_z);

	double corner_min_x = std::numeric_limits<double>::max();
	double corner_min_y = std::numeric_limits<double>::max();

	params["corner_min_x"] = TParamEntry("%lf", &corner_min_x);
	params["corner_min_y"] = TParamEntry("%lf", &corner_min_y);

	mrpt::img::TColor mesh_color(0xa0, 0xe0, 0xa0);
	params["mesh_color"] = TParamEntry("%color", &mesh_color);

	params["resolution"] = TParamEntry("%f", &resolution_);
	params["texture_extension_x"] = TParamEntry("%f", &textureExtensionX_);
	params["texture_extension_y"] = TParamEntry("%f", &textureExtensionY_);

	params["debug_show_contact_points"] = TParamEntry("%bool", &debugShowContactPoints_);

	parse_xmlnode_children_as_param(*root, params, world_->user_defined_variables());

	// Load elevation data:
	mrpt::math::CMatrixFloat elevation_data;
	if (!sElevationImgFile.empty())
	{
		sElevationImgFile = world_->local_to_abs_path(sElevationImgFile);

		mrpt::img::CImage imgElev;
		if (!imgElev.loadFromFile(sElevationImgFile, 0 /*force load grayscale*/))
			throw std::runtime_error(mrpt::format(
				"[ElevationMap] ERROR: Cannot read elevation image '%s'",
				sElevationImgFile.c_str()));

		// Scale: [0,1] => [min_z,max_z]
		// Get image normalized in range [0,1]
		imgElev.getAsMatrix(elevation_data);
		ASSERT_(img_min_z != img_max_z);

		const double vmin = elevation_data.minCoeff();
		const double vmax = elevation_data.maxCoeff();
		mrpt::math::CMatrixFloat f = elevation_data;
		f -= vmin;
		f *= (img_max_z - img_min_z) / (vmax - vmin);
		mrpt::math::CMatrixFloat m(elevation_data.rows(), elevation_data.cols());
		m.setConstant(img_min_z);
		f += m;
		elevation_data = std::move(f);
	}
	else
	{
		// TODO: Support reading from txt file?
	}

	// Load texture (optional):
	mrpt::img::CImage mesh_image;
	bool has_mesh_image = false;
	if (!sTextureImgFile.empty())
	{
		sTextureImgFile = world_->xmlPathToActualPath(sTextureImgFile);

		if (!mesh_image.loadFromFile(sTextureImgFile))
			throw std::runtime_error(mrpt::format(
				"[ElevationMap] ERROR: Cannot read texture image '%s'", sTextureImgFile.c_str()));
		has_mesh_image = true;
	}

	// Build mesh:
	MRPT_TODO("Support model_split_size option here too");
	gl_mesh_ = mrpt::opengl::CMesh::Create();

	gl_mesh_->enableTransparency(false);

	if (has_mesh_image)
	{
		gl_mesh_->assignImageAndZ(mesh_image, elevation_data);
		gl_mesh_->setMeshTextureExtension(textureExtensionX_, textureExtensionY_);
	}
	else
	{
		gl_mesh_->setZ(elevation_data);
		gl_mesh_->setColor_u8(mesh_color);
	}

	// Extension: X,Y
	const double LX = (elevation_data.rows() - 1) * resolution_;
	const double LY = (elevation_data.cols() - 1) * resolution_;

	if (corner_min_x == std::numeric_limits<double>::max()) corner_min_x = -0.5 * LX;

	if (corner_min_y == std::numeric_limits<double>::max()) corner_min_y = -0.5 * LY;

	// Important: the yMin/yMax in the next line are swapped to handle
	// the "+y" different direction in image and map coordinates, it is not
	// a bug:
	gl_mesh_->setGridLimits(corner_min_x, corner_min_x + LX, corner_min_y, corner_min_y + LY);

	// Save copy for calcs:
	meshCacheZ_ = elevation_data;
	meshMinX_ = corner_min_x;
	meshMinY_ = corner_min_y;
	meshMaxX_ = corner_min_x + LX;
	meshMaxY_ = corner_min_y + LY;

	// hint for rendering z-order:
	gl_mesh_->setLocalRepresentativePoint(
		mrpt::math::TPoint3Df(corner_min_x + 0.5 * LX, corner_min_y + 0.5 * LY, .0f));

	gl_debugWheelsContactPoints_ = mrpt::opengl::CPointCloud::Create();
	gl_debugWheelsContactPoints_->enableVariablePointSize(false);
	gl_debugWheelsContactPoints_->setPointSize(7.0f);
}

void ElevationMap::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	using namespace mrpt::math;

	ASSERTMSG_(
		gl_mesh_,
		"ERROR: Can't render Mesh before loading it! Have you called "
		"loadConfigFrom() first?");

	// 1st time call?? -> Create objects
	if (firstSceneRendering_ && viz && physical)
	{
		firstSceneRendering_ = false;
		viz->get().insert(gl_mesh_);
		physical->get().insert(gl_mesh_);

		viz->get().insert(gl_debugWheelsContactPoints_);
	}
}

void ElevationMap::simul_pre_timestep([[maybe_unused]] const TSimulContext& context)
{
	// For each vehicle:
	// 1) Compute its 3D pose according to the mesh tilt angle.
	// 2) Apply gravity force
	const double gravity = parent()->get_gravity();

	const World::VehicleList& lstVehs = this->world_->getListOfVehicles();
	for (auto& nameVeh : lstVehs)
	{
		world_->getTimeLogger().enter("elevationmap.handle_vehicle");

		auto& veh = nameVeh.second;

		const size_t nWheels = veh->getNumWheels();

		// 1) Compute its 3D pose according to the mesh tilt angle.
		// Idea: run a least-squares method to find the best
		// SE(3) transformation that map the wheels contact point,
		// as seen in local & global coordinates.
		// (For large tilt angles, may have to run it iteratively...)
		// -------------------------------------------------------------
		// the final downwards direction (unit vector (0,0,-1)) as seen in
		// vehicle local frame.
		mrpt::math::TPoint3D dir_down;
		for (int iter = 0; iter < 2; iter++)
		{
			const mrpt::math::TPose3D& cur_pose = veh->getPose();
			// This object is faster for repeated point projections
			const mrpt::poses::CPose3D cur_cpose(cur_pose);

			mrpt::math::TPose3D new_pose = cur_pose;
			corrs_.clear();

			bool out_of_area = false;
			for (size_t iW = 0; !out_of_area && iW < nWheels; iW++)
			{
				const Wheel& wheel = veh->getWheelInfo(iW);

				// Local frame
				mrpt::tfest::TMatchingPair corr;

				corr.localIdx = iW;
				corr.local = mrpt::math::TPoint3D(wheel.x, wheel.y, 0);

				// Global frame
				const mrpt::math::TPoint3D gPt = cur_cpose.composePoint({wheel.x, wheel.y, 0.0});
				auto z = this->getElevationAt(mrpt::math::TPoint2Df(gPt.x, gPt.y));
				if (!z.has_value())
				{
					out_of_area = true;
					continue;  // vehicle is out of bounds!
				}

				corr.globalIdx = iW;
				corr.global = mrpt::math::TPoint3D(gPt.x, gPt.y, *z);

				corrs_.push_back(corr);
			}
			if (out_of_area) continue;

			// Register:
			double transf_scale;
			mrpt::poses::CPose3DQuat tmpl;

			mrpt::tfest::se3_l2(corrs_, tmpl, transf_scale, true /*force scale unity*/);

			optimalTf_ = mrpt::poses::CPose3D(tmpl);

			new_pose.z = optimalTf_.z();
			new_pose.yaw = optimalTf_.yaw();
			new_pose.pitch = optimalTf_.pitch();
			new_pose.roll = optimalTf_.roll();

			veh->setPose(new_pose);

		}  // end iters

		// debug contact points:
		if (debugShowContactPoints_)
		{
			gl_debugWheelsContactPoints_->clear();
			for (const auto& c : corrs_) gl_debugWheelsContactPoints_->insertPoint(c.global);
		}

		// compute "down" direction:
		{
			mrpt::poses::CPose3D rot_only;
			rot_only.setRotationMatrix(optimalTf_.getRotationMatrix());
			rot_only.inverseComposePoint(.0, .0, -1.0, dir_down.x, dir_down.y, dir_down.z);
		}

		// 2) Apply gravity force
		// -------------------------------------------------------------
		{
			// To chassis:
			const double chassis_weight = veh->getChassisMass() * gravity;
			const mrpt::math::TPoint2D chassis_com = veh->getChassisCenterOfMass();
			veh->apply_force(
				{dir_down.x * chassis_weight, dir_down.y * chassis_weight}, chassis_com);

			// To wheels:
			for (size_t iW = 0; iW < nWheels; iW++)
			{
				const Wheel& wheel = veh->getWheelInfo(iW);
				const double wheel_weight = wheel.mass * gravity;
				veh->apply_force(
					{dir_down.x * wheel_weight, dir_down.y * wheel_weight}, {wheel.x, wheel.y});
			}
		}

		world_->getTimeLogger().leave("elevationmap.handle_vehicle");
	}
}

void ElevationMap::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);

	// TODO: Think if this is still applicable:
	// Save all elements positions in prestep, then here scale their
	// movements * cos(angle)
}

namespace
{
float calcz(
	const mrpt::math::TPoint3Df& p1, const mrpt::math::TPoint3Df& p2,
	const mrpt::math::TPoint3Df& p3, float x, float y)
{
	const float det = (p2.x - p3.x) * (p1.y - p3.y) +  //
					  (p3.y - p2.y) * (p1.x - p3.x);
	ASSERT_(det != 0.0f);

	const float l1 = ((p2.x - p3.x) * (y - p3.y) + (p3.y - p2.y) * (x - p3.x)) / det;
	const float l2 = ((p3.x - p1.x) * (y - p3.y) + (p1.y - p3.y) * (x - p3.x)) / det;
	const float l3 = 1.0f - l1 - l2;

	return l1 * p1.z + l2 * p2.z + l3 * p3.z;
}
}  // namespace

std::optional<float> ElevationMap::getElevationAt(const mrpt::math::TPoint2Df& pt) const
{
	// mesh->getxMin();
	const float x0 = meshMinX_;
	const float y0 = meshMinY_;
	const float x1 = meshMaxX_;
	const float y1 = meshMaxY_;

	const size_t nCellsX = meshCacheZ_.rows();
	const size_t nCellsY = meshCacheZ_.cols();

	const float sCellX = (x1 - x0) / (nCellsX - 1);
	const float sCellY = (y1 - y0) / (nCellsY - 1);

	// Discretize:
	const int cx00 = ::floor((pt.x - x0) / sCellX);
	const int cy00 = ::floor((pt.y - y0) / sCellY);

	if (cx00 < 1 || cx00 >= int(nCellsX - 1) || cy00 < 1 || cy00 >= int(nCellsY - 1))  //
		return {};	// out of bounds!

	// Linear interpolation:
	const float z00 = meshCacheZ_(cx00, cy00);
	const float z01 = meshCacheZ_(cx00, cy00 + 1);
	const float z10 = meshCacheZ_(cx00 + 1, cy00);
	const float z11 = meshCacheZ_(cx00 + 1, cy00 + 1);

	//
	//   p01 ---- p11
	//    |        |
	//   p00 ---- p10
	//
	const mrpt::math::TPoint3Df p00(.0f, .0f, z00);
	const mrpt::math::TPoint3Df p01(.0f, sCellY, z01);
	const mrpt::math::TPoint3Df p10(sCellX, .0f, z10);
	const mrpt::math::TPoint3Df p11(sCellX, sCellY, z11);

	const float lx = pt.x - (x0 + cx00 * sCellX);
	const float ly = pt.y - (y0 + cy00 * sCellY);

	if (ly >= lx)
		return calcz(p00, p01, p11, lx, ly);
	else
		return calcz(p00, p10, p11, lx, ly);
}
