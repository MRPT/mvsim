/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
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
/** Simulates a 2D gridmap, including collisions with occupied voxels.
 *
 * \ingroup world_elements_module
 */
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

	const mrpt::maps::COccupancyGridMap2D& getOccGrid() const { return grid_; }
	mrpt::maps::COccupancyGridMap2D& getOccGrid() { return grid_; }

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	mrpt::maps::COccupancyGridMap2D grid_;

	bool gui_uptodate_;	 //!< Whether gl_grid_ has to be updated upon next
						 //! call of internalGuiUpdate()
	mrpt::opengl::CSetOfObjects::Ptr gl_grid_;

	struct TFixturePtr
	{
		TFixturePtr() = default;
		b2Fixture* fixture = nullptr;
	};

	struct TInfoPerCollidableobj
	{
		TInfoPerCollidableobj() = default;

		mrpt::poses::CPose2D pose;
		b2Body* collide_body = nullptr;
		mrpt::obs::CObservation2DRangeScan::Ptr scan;
		std::vector<TFixturePtr> collide_fixtures;
		float max_obstacles_ranges = 0;
	};

	std::vector<TInfoPerCollidableobj> obstacles_for_each_obj_;
	std::vector<mrpt::opengl::CSetOfObjects::Ptr> gl_obs_clouds_;

	std::mutex gl_obs_clouds_buffer_cs_;
	std::vector<mrpt::opengl::CPointCloud::Ptr> gl_obs_clouds_buffer_;

	mrpt::obs::CSinCosLookUpTableFor2DScans sincos_lut_;

	bool show_grid_collision_points_;
	double restitution_;  //!< Elastic restitution coef (default: 0.01)
	double lateral_friction_;  //!< (Default: 0.5)
};
}  // namespace mvsim
