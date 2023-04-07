/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

OccupancyGridMap::OccupancyGridMap(
	World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent),
	  gui_uptodate_(false),
	  show_grid_collision_points_(true),
	  restitution_(0.01),
	  lateral_friction_(0.5)
{
	doLoadConfigFrom(root);
}

OccupancyGridMap::~OccupancyGridMap() {}

void OccupancyGridMap::doLoadConfigFrom(const rapidxml::xml_node<char>* root)
{
	gui_uptodate_ = false;

	// <file>FILENAME.{png,gridmap}</file>
	xml_node<>* xml_file = root->first_node("file");
	if (!xml_file || !xml_file->value())
		throw std::runtime_error(
			"Error: <file></file> XML entry not found inside gridmap node!");

	const string sFile = world_->local_to_abs_path(xml_file->value());
	const string sFileExt =
		mrpt::system::extractFileExtension(sFile, true /*ignore gz*/);

	// ROS YAML map files:
	if (sFileExt == "yaml")
	{
#if MRPT_VERSION >= 0x250
		bool ok = grid_.loadFromROSMapServerYAML(sFile);
		ASSERTMSG_(
			ok,
			mrpt::format("Error loading ROS map file: '%s'", sFile.c_str()));
#else
		THROW_EXCEPTION("Loading ROS YAML map files requires MRPT>=2.5.0");
#endif
	}
	else if (sFileExt == "gridmap")
	{
		mrpt::io::CFileGZInputStream fi(sFile);
		auto f = mrpt::serialization::archiveFrom(fi);
		f >> grid_;
	}
	else
	// Assume it's an image:
	{
		TParameterDefinitions other_params;
		double xcenterpixel = -1, ycenterpixel = -1;
		double resolution = 0.10;

		other_params["resolution"] = TParamEntry("%lf", &resolution);
		other_params["centerpixel_x"] = TParamEntry("%lf", &xcenterpixel);
		other_params["centerpixel_y"] = TParamEntry("%lf", &ycenterpixel);

		parse_xmlnode_children_as_param(
			*root, other_params, world_->user_defined_variables());

		if (!grid_.loadFromBitmapFile(
				sFile, resolution, {xcenterpixel, ycenterpixel}))
			throw std::runtime_error(mrpt::format(
				"[OccupancyGridMap] ERROR: File not found '%s'",
				sFile.c_str()));
	}

	{
		// Other general params:
		TParameterDefinitions ps;
		ps["show_collisions"] =
			TParamEntry("%bool", &show_grid_collision_points_);
		ps["restitution"] = TParamEntry("%lf", &restitution_);
		ps["lateral_friction"] = TParamEntry("%lf", &lateral_friction_);

		parse_xmlnode_children_as_param(
			*root, ps, world_->user_defined_variables());
	}
}

void OccupancyGridMap::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	using namespace mrpt::math;

	// 1st time call?? -> Create objects
	if (!gl_grid_ && viz && physical)
	{
		gl_grid_ = mrpt::opengl::CSetOfObjects::Create();
		gl_grid_->setName("OccupancyGridMap");
		viz->get().insert(gl_grid_);
		physical->get().insert(gl_grid_);
	}
	if (gl_obs_clouds_.size() != obstacles_for_each_obj_.size())
	{
		gl_obs_clouds_.resize(obstacles_for_each_obj_.size());
	}

	// 1st call OR gridmap changed?
	if (!gui_uptodate_)
	{
#if MRPT_VERSION >= 0x240
		grid_.getVisualizationInto(*gl_grid_);
#else
		grid_.getAs3DObject(gl_grid_);
#endif
		gui_uptodate_ = true;
	}

	// Update obstacles:
	if (viz)
	{
		std::lock_guard<std::mutex> csl(gl_obs_clouds_buffer_cs_);
		for (size_t i = 0; i < gl_obs_clouds_.size(); i++)
		{
			mrpt::opengl::CSetOfObjects::Ptr& gl_objs = gl_obs_clouds_[i];
			if (!gl_objs)
			{
				gl_objs = mrpt::opengl::CSetOfObjects::Create();
				gl_objs->setName(
					"OccupancyGridMap"s + this->getName() + ".obstacles["s +
					std::to_string(i) + "]"s);
				viz->get().insert(gl_objs);
			}

			// Now that we are in a safe thread (with the OpenGL scene lock
			// adquired from the caller)
			// proceed to replace the old with the new point cloud:
			gl_objs->clear();
			if (gl_obs_clouds_buffer_.size() > i)
				gl_objs->insert(gl_obs_clouds_buffer_[i]);
		}

		gl_obs_clouds_buffer_.clear();
		// end lock
	}
}

void OccupancyGridMap::simul_pre_timestep(
	[[maybe_unused]] const TSimulContext& context)
{
	// Make a list of objects subject to collide with the occupancy grid:
	// - Vehicles
	// - Blocks
	{
		const World::VehicleList& lstVehs = this->world_->getListOfVehicles();
		const World::BlockList& lstBlocks = this->world_->getListOfBlocks();
		const size_t nObjs = lstVehs.size() + lstBlocks.size();

		obstacles_for_each_obj_.resize(nObjs);
		size_t obj_idx = 0;
		for (World::VehicleList::const_iterator itVeh = lstVehs.begin();
			 itVeh != lstVehs.end(); ++itVeh, ++obj_idx)
		{
			TInfoPerCollidableobj& ipv = obstacles_for_each_obj_[obj_idx];
			ipv.max_obstacles_ranges =
				itVeh->second->getMaxVehicleRadius() * 1.50f;
			const mrpt::math::TPose3D& pose = itVeh->second->getPose();
			ipv.pose = mrpt::poses::CPose2D(
				pose.x, pose.y,
				0 /* angle=0, no need to rotate everything to later rotate back again! */);
		}
		for (const auto& block : lstBlocks)
		{
			TInfoPerCollidableobj& ipv = obstacles_for_each_obj_[obj_idx];
			ipv.max_obstacles_ranges =
				block.second->getMaxBlockRadius() * 1.50f;
			const mrpt::math::TPose3D& pose = block.second->getPose();
			/* angle=0, no need to rotate everything to later rotate back again!
			 */
			ipv.pose = mrpt::poses::CPose2D(pose.x, pose.y, 0);

			obj_idx++;
		}
	}

	// For each object, create a ground body with fixtures at each scanned point
	// around the vehicle, so it can collide with the environment:
	// Upon first usage, reserve mem:
	{  // lock
		std::lock_guard<std::mutex> csl(gl_obs_clouds_buffer_cs_);
		const size_t nObjs = obstacles_for_each_obj_.size();
		gl_obs_clouds_buffer_.resize(nObjs);

		for (size_t obj_idx = 0; obj_idx < nObjs; obj_idx++)
		{
			// 1) Simulate scan to get obstacles around the vehicle:
			TInfoPerCollidableobj& ipv = obstacles_for_each_obj_[obj_idx];
			mrpt::obs::CObservation2DRangeScan::Ptr& scan = ipv.scan;
			// Upon first time, reserve mem:
			if (!scan) scan = mrpt::obs::CObservation2DRangeScan::Create();

			const float veh_max_obstacles_ranges = ipv.max_obstacles_ranges;
			const float occup_threshold = 0.5f;
			const size_t nRays = 50;

			scan->aperture = 2.0 * M_PI;  // 360 field of view
			scan->maxRange = veh_max_obstacles_ranges;

			grid_.laserScanSimulator(
				*scan, ipv.pose, occup_threshold, nRays, 0.0f /*noise*/);

			// Since we'll dilate obstacle points, let's give a bit more space
			// as compensation:
			const float range_enlarge = 0.25f * grid_.getResolution();
			for (size_t k = 0; k < scan->getScanSize(); k++)
			{
				scan->setScanRange(k, scan->getScanRange(k) + range_enlarge);
			}
			// 2) Create a Box2D "ground body" with square "fixtures" so the
			// vehicle can collide with the occ. grid:

			// Create Box2D objects upon first usage:
			if (!ipv.collide_body)
			{
				b2BodyDef bdef;
				ipv.collide_body = world_->getBox2DWorld()->CreateBody(&bdef);
				ASSERT_(ipv.collide_body);
			}
			// Force move the body to the vehicle origins (to use obstacles in
			// local coords):
			ipv.collide_body->SetTransform(
				b2Vec2(ipv.pose.x(), ipv.pose.y()), .0f);

			// GL:
			// 1st usage?
			mrpt::opengl::CPointCloud::Ptr& gl_pts =
				gl_obs_clouds_buffer_[obj_idx];
			if (show_grid_collision_points_)
			{
				gl_pts = mrpt::opengl::CPointCloud::Create();
				gl_pts->setPointSize(4.0f);
				gl_pts->setColor(0, 0, 1);

				gl_pts->setVisibility(show_grid_collision_points_);
				gl_pts->setPose(mrpt::poses::CPose2D(ipv.pose));
				gl_pts->clear();
			}

			// Physical properties of each "occupied cell":
			const float occCellSemiWidth = grid_.getResolution() * 0.4f;
			b2PolygonShape sqrPoly;
			sqrPoly.SetAsBox(occCellSemiWidth, occCellSemiWidth);
			sqrPoly.m_radius = 1e-3;  // The "skin" depth of the body
			b2FixtureDef fixtureDef;
			fixtureDef.shape = &sqrPoly;
			fixtureDef.restitution = restitution_;
			fixtureDef.density = 0;	 // Fixed (inf. mass)
			fixtureDef.friction = lateral_friction_;  // 0.5f;

			// Create fixtures at their place (or disable it if no obstacle has
			// been sensed):
			const mrpt::obs::CSinCosLookUpTableFor2DScans::TSinCosValues&
				sincos_tab = sincos_lut_.getSinCosForScan(*scan);
			ipv.collide_fixtures.resize(nRays);
			for (size_t k = 0; k < nRays; k++)
			{
				if (!ipv.collide_fixtures[k].fixture)
					ipv.collide_fixtures[k].fixture =
						ipv.collide_body->CreateFixture(&fixtureDef);

				if (!scan->getScanRangeValidity(k))
				{
					ipv.collide_fixtures[k].fixture->SetSensor(
						true);	// Box2D's way of saying: don't collide with
								// this!
					ipv.collide_fixtures[k].fixture->GetUserData().pointer =
						INVISIBLE_FIXTURE_USER_DATA;
				}
				else
				{
					// Box2D's way of saying: don't collide with this!
					ipv.collide_fixtures[k].fixture->SetSensor(false);
					ipv.collide_fixtures[k].fixture->GetUserData().pointer = 0;

					b2PolygonShape* poly = dynamic_cast<b2PolygonShape*>(
						ipv.collide_fixtures[k].fixture->GetShape());
					ASSERT_(poly != nullptr);

					const float llx =
						sincos_tab.ccos[k] * scan->getScanRange(k);
					const float lly =
						sincos_tab.csin[k] * scan->getScanRange(k);

					const float ggx = ipv.pose.x() + llx;
					const float ggy = ipv.pose.y() + lly;

					const float gx = grid_.idx2x(grid_.x2idx(ggx));
					const float gy = grid_.idx2y(grid_.y2idx(ggy));

					const float lx = gx - ipv.pose.x();
					const float ly = gy - ipv.pose.y();

					poly->SetAsBox(
						occCellSemiWidth, occCellSemiWidth, b2Vec2(lx, ly),
						.0f /*angle*/);

					if (gl_pts && show_grid_collision_points_)
					{
						gl_pts->mrpt::opengl::CPointCloud::insertPoint(
							lx, ly, .0);
					}
				}
			}
		}  // end for obj_idx

	}  // end lock
}
