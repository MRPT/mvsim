/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mvsim/WorldElements/WorldElementBase.h>

#include <mutex>

namespace mvsim
{
class PointCloud : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(PointCloud)
   public:
	PointCloud(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~PointCloud();

	void doLoadConfigFrom(const rapidxml::xml_node<char>* root);

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override
	{
		doLoadConfigFrom(root);
	}

	virtual void simul_pre_timestep(const TSimulContext& context) override;

	const mrpt::maps::CPointsMap::Ptr& getPoints() const { return m_points; }
	mrpt::maps::CPointsMap::Ptr& getPoints() { return m_points; }

	void poses_mutex_lock() override {}
	void poses_mutex_unlock() override {}

   protected:
	virtual void internalGuiUpdate(
		mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
		bool childrenOnly) override;

	mrpt::maps::CPointsMap::Ptr m_points;

	/// Whether m_gl_grid has to be updated upon next call of
	/// internalGuiUpdate()
	bool m_gui_uptodate = false;
	mrpt::opengl::CSetOfObjects::Ptr m_gl_points;

	double m_render_points_size = 3.0;
	mrpt::poses::CPose3D m_pointcloud_pose = mrpt::poses::CPose3D::Identity();
};

}  // namespace mvsim
