/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSinCosLookUpTableFor2DScans.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/poses/CPose2D.h>
#include <mvsim/WorldElements/WorldElementBase.h>

#include <mutex>

namespace mvsim
{
class OccupancyGridMap : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(OccupancyGridMap)
   public:
	OccupancyGridMap(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~OccupancyGridMap();

	void doLoadConfigFrom(const rapidxml::xml_node<char>* root);

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override
	{
		doLoadConfigFrom(root);
	}

	virtual void simul_pre_timestep(const TSimulContext& context) override;

	const mrpt::maps::COccupancyGridMap2D& getOccGrid() const { return m_grid; }
	mrpt::maps::COccupancyGridMap2D& getOccGrid() { return m_grid; }

	void poses_mutex_lock() override {}
	void poses_mutex_unlock() override {}

   protected:
	virtual void internalGuiUpdate(
		mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
		bool childrenOnly) override;

	mrpt::maps::COccupancyGridMap2D m_grid;

	bool m_gui_uptodate;  //!< Whether m_gl_grid has to be updated upon next
						  //! call of internalGuiUpdate()
	mrpt::opengl::CSetOfObjects::Ptr m_gl_grid;

	struct TFixturePtr
	{
		b2Fixture* fixture;
		TFixturePtr() : fixture(nullptr) {}
	};

	struct TInfoPerCollidableobj
	{
		float max_obstacles_ranges;
		mrpt::poses::CPose2D pose;
		mrpt::obs::CObservation2DRangeScan::Ptr scan;
		b2Body* collide_body;
		std::vector<TFixturePtr> collide_fixtures;

		TInfoPerCollidableobj() : max_obstacles_ranges(0), collide_body(nullptr)
		{
		}
	};

	std::vector<TInfoPerCollidableobj> m_obstacles_for_each_obj;
	std::vector<mrpt::opengl::CSetOfObjects::Ptr> m_gl_obs_clouds;

	std::mutex m_gl_obs_clouds_buffer_cs;
	std::vector<mrpt::opengl::CPointCloud::Ptr> m_gl_obs_clouds_buffer;

	mrpt::obs::CSinCosLookUpTableFor2DScans m_sincos_lut;

	bool m_show_grid_collision_points;
	double m_restitution;  //!< Elastic restitution coef (default: 0.01)
	double m_lateral_friction;	//!< (Default: 0.5)
};
}  // namespace mvsim
