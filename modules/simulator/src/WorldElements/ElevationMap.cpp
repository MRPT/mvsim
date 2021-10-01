/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/tfest.h>	 // least-squares methods
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/ElevationMap.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

ElevationMap::ElevationMap(World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent), m_first_scene_rendering(true), m_resolution(1.0)
{
	loadConfigFrom(root);
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

	mrpt::img::TColor mesh_color(0xa0, 0xe0, 0xa0);
	params["mesh_color"] = TParamEntry("%color", &mesh_color);

	params["resolution"] = TParamEntry("%lf", &m_resolution);

	parse_xmlnode_children_as_param(*root, params);

	// Load elevation data:
	mrpt::math::CMatrixFloat elevation_data;
	if (!sElevationImgFile.empty())
	{
		sElevationImgFile = m_world->resolvePath(sElevationImgFile);

		mrpt::img::CImage imgElev;
		if (!imgElev.loadFromFile(
				sElevationImgFile, 0 /*force load grayscale*/))
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
		mrpt::math::CMatrixFloat m(
			elevation_data.rows(), elevation_data.cols());
		m.setConstant(img_min_z);
		f += m;
		elevation_data = std::move(f);
	}
	else
	{
		MRPT_TODO("Imgs or txt matrix")
	}

	// Load texture (optional):
	mrpt::img::CImage mesh_image;
	bool has_mesh_image = false;
	if (!sTextureImgFile.empty())
	{
		sTextureImgFile = m_world->resolvePath(sTextureImgFile);

		if (!mesh_image.loadFromFile(sTextureImgFile))
			throw std::runtime_error(mrpt::format(
				"[ElevationMap] ERROR: Cannot read texture image '%s'",
				sTextureImgFile.c_str()));
		has_mesh_image = true;
	}

	// Build mesh:
	m_gl_mesh = mrpt::opengl::CMesh::Create();

	m_gl_mesh->enableTransparency(false);

	if (has_mesh_image)
	{
		ASSERT_EQUAL_(mesh_image.getWidth(), (size_t)elevation_data.cols());
		ASSERT_EQUAL_(mesh_image.getHeight(), (size_t)elevation_data.rows());

		m_gl_mesh->assignImageAndZ(mesh_image, elevation_data);
	}
	else
	{
		m_gl_mesh->setZ(elevation_data);
		m_gl_mesh->setColor_u8(mesh_color);
	}

	// Save copy for calcs:
	m_mesh_z_cache = elevation_data;

	// Extension: X,Y
	const double LX = (elevation_data.cols() - 1) * m_resolution;
	const double LY = (elevation_data.rows() - 1) * m_resolution;
	m_gl_mesh->setGridLimits(-0.5 * LX, 0.5 * LX, -0.5 * LY, 0.5 * LY);
}

void ElevationMap::internalGuiUpdate(
	mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
	bool childrenOnly)
{
	using namespace mrpt::math;

	ASSERTMSG_(
		m_gl_mesh,
		"ERROR: Can't render Mesh before loading it! Have you called "
		"loadConfigFrom() first?");

	// 1st time call?? -> Create objects
	if (m_first_scene_rendering)
	{
		m_first_scene_rendering = false;
		viz.insert(m_gl_mesh);
	}
}

void ElevationMap::simul_pre_timestep(const TSimulContext& context)
{
	// For each vehicle:
	// 1) Compute its 3D pose according to the mesh tilt angle.
	// 2) Apply gravity force
	const double gravity = getWorldObject()->get_gravity();

	ASSERT_(m_gl_mesh);

	const World::VehicleList& lstVehs = this->m_world->getListOfVehicles();
	for (World::VehicleList::const_iterator itVeh = lstVehs.begin();
		 itVeh != lstVehs.end(); ++itVeh)
	{
		m_world->getTimeLogger().enter("elevationmap.handle_vehicle");

		const size_t nWheels = itVeh->second->getNumWheels();

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
			const mrpt::math::TPose3D& cur_pose = itVeh->second->getPose();
			// This object is faster for repeated point projections
			const mrpt::poses::CPose3D cur_cpose(cur_pose);

			mrpt::math::TPose3D new_pose = cur_pose;
			corrs.clear();

			bool out_of_area = false;
			for (size_t iW = 0; !out_of_area && iW < nWheels; iW++)
			{
				const Wheel& wheel = itVeh->second->getWheelInfo(iW);

				// Local frame
				mrpt::tfest::TMatchingPair corr;

				corr.other_idx = iW;
				corr.other_x = wheel.x;
				corr.other_y = wheel.y;
				corr.other_z = 0;

				// Global frame
				const mrpt::math::TPoint3D gPt =
					cur_cpose.composePoint({wheel.x, wheel.y, 0.0});
				float z;
				if (!getElevationAt(gPt.x /*in*/, gPt.y /*in*/, z /*out*/))
				{
					out_of_area = true;
					continue;  // vehicle is out of bounds!
				}

				corr.this_idx = iW;
				corr.this_x = gPt.x;
				corr.this_y = gPt.y;
				corr.this_z = z;

				corrs.push_back(corr);
			}
			if (out_of_area) continue;

			// Register:
			double transf_scale;
			mrpt::poses::CPose3DQuat tmpl;

			mrpt::tfest::se3_l2(
				corrs, tmpl, transf_scale, true /*force scale unity*/);

			m_optimal_transf = mrpt::poses::CPose3D(tmpl);
			new_pose.z = m_optimal_transf.z();
			new_pose.yaw = m_optimal_transf.yaw();
			new_pose.pitch = m_optimal_transf.pitch();
			new_pose.roll = m_optimal_transf.roll();

			// cout << new_pose << endl;

			itVeh->second->setPose(new_pose);

		}  // end iters

		{
			mrpt::poses::CPose3D rot_only;
			rot_only.setRotationMatrix(m_optimal_transf.getRotationMatrix());
			rot_only.inverseComposePoint(
				.0, .0, -1.0, dir_down.x, dir_down.y, dir_down.z);
		}

		// 2) Apply gravity force
		// -------------------------------------------------------------
		{
			// To chassis:
			const double chassis_weight =
				itVeh->second->getChassisMass() * gravity;
			const mrpt::math::TPoint2D chassis_com =
				itVeh->second->getChassisCenterOfMass();
			itVeh->second->apply_force(
				{dir_down.x * chassis_weight, dir_down.y * chassis_weight},
				chassis_com);

			// To wheels:
			for (size_t iW = 0; iW < nWheels; iW++)
			{
				const Wheel& wheel = itVeh->second->getWheelInfo(iW);
				const double wheel_weight = wheel.mass * gravity;
				itVeh->second->apply_force(
					{dir_down.x * wheel_weight, dir_down.y * wheel_weight},
					{wheel.x, wheel.y});
			}
		}

		m_world->getTimeLogger().leave("elevationmap.handle_vehicle");
	}
}

void ElevationMap::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);

	MRPT_TODO(
		"Save all elements positions in prestep, then here scale their "
		"movements * cos(angle)");
}

static float calcz(
	const mrpt::math::TPoint3Df& p1, const mrpt::math::TPoint3Df& p2,
	const mrpt::math::TPoint3Df& p3, float x, float y)
{
	const float det =
		(p2.x - p3.x) * (p1.y - p3.y) + (p3.y - p2.y) * (p1.x - p3.x);
	ASSERT_(det != 0.0f);

	const float l1 =
		((p2.x - p3.x) * (y - p3.y) + (p3.y - p2.y) * (x - p3.x)) / det;
	const float l2 =
		((p3.x - p1.x) * (y - p3.y) + (p1.y - p3.y) * (x - p3.x)) / det;
	const float l3 = 1.0f - l1 - l2;

	return l1 * p1.z + l2 * p2.z + l3 * p3.z;
}

bool ElevationMap::getElevationAt(double x, double y, float& z) const
{
	const mrpt::opengl::CMesh* mesh = m_gl_mesh.get();
	const float x0 = mesh->getxMin();
	const float y0 = mesh->getyMin();
	const size_t nCellsX = m_mesh_z_cache.cols();
	const size_t nCellsY = m_mesh_z_cache.rows();

	// Discretize:
	const int cx00 = ::floor((x - x0) / m_resolution);
	const int cy00 = ::floor((y - y0) / m_resolution);

	if (cx00 < 1 || cx00 >= int(nCellsX - 1) || cy00 < 1 ||
		cy00 >= int(nCellsY - 1))
		return false;

	// Linear interpolation:
	const float z00 = m_mesh_z_cache(cy00, cx00);
	const float z01 = m_mesh_z_cache(cy00 + 1, cx00);
	const float z10 = m_mesh_z_cache(cy00, cx00 + 1);
	const float z11 = m_mesh_z_cache(cy00 + 1, cx00 + 1);

	//
	//   p01 ---- p11
	//    |        |
	//   p00 ---- p10
	//
	const mrpt::math::TPoint3Df p00(.0f, .0f, z00);
	const mrpt::math::TPoint3Df p01(.0f, m_resolution, z01);
	const mrpt::math::TPoint3Df p10(m_resolution, .0f, z10);
	const mrpt::math::TPoint3Df p11(m_resolution, m_resolution, z11);

	const float lx = x - (x0 + cx00 * m_resolution);
	const float ly = y - (y0 + cy00 * m_resolution);

	if (ly >= lx)
		z = calcz(p00, p01, p11, lx, ly);
	else
		z = calcz(p00, p10, p11, lx, ly);

	return true;
}
