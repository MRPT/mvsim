/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
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

	const mrpt::maps::CPointsMap::Ptr& getPoints() const { return points_; }
	mrpt::maps::CPointsMap::Ptr& getPoints() { return points_; }

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	mrpt::maps::CPointsMap::Ptr points_;

	/// Whether gl_grid_ has to be updated upon next call of
	/// internalGuiUpdate()
	bool gui_uptodate_ = false;
	mrpt::opengl::CSetOfObjects::Ptr gl_points_;

	double render_points_size_ = 3.0;
	mrpt::poses::CPose3D pointcloud_pose_ = mrpt::poses::CPose3D::Identity();
};

}  // namespace mvsim
