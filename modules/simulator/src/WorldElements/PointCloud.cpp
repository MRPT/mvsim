/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/PointCloud.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

PointCloud::PointCloud(World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent)
{
	doLoadConfigFrom(root);
}

PointCloud::~PointCloud() {}

void PointCloud::doLoadConfigFrom(const rapidxml::xml_node<char>* root)
{
	gui_uptodate_ = false;

	if (auto x2d = root->first_node("file_txt_2d"); x2d && x2d->value())
	{
		const string sFile = world_->local_to_abs_path(x2d->value());

		points_ = mrpt::maps::CSimplePointsMap::Create();
		points_->load2D_from_text_file(sFile);
	}
	else if (auto x3d = root->first_node("file_txt_3d"); x3d && x3d->value())
	{
		const string sFile = world_->local_to_abs_path(x3d->value());

		points_ = mrpt::maps::CSimplePointsMap::Create();
		points_->load3D_from_text_file(sFile);
	}
	else
	{
		THROW_EXCEPTION("Error: No valid <file_*></file_*> XML tag was found");
	}

	{
		// Other general params:
		TParameterDefinitions ps;
		ps["points_size"] = TParamEntry("%lf", &render_points_size_);
		ps["pose_3d"] = TParamEntry("%pose3d", &pointcloud_pose_);
		parse_xmlnode_children_as_param(*root, ps, world_->user_defined_variables());
	}

	points_->renderOptions.point_size = render_points_size_;

	gui_uptodate_ = false;
}

void PointCloud::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	using namespace mrpt::math;

	// 1st time call?? -> Create objects
	if (!gl_points_ && viz && physical)
	{
		gl_points_ = mrpt::opengl::CSetOfObjects::Create();
		gl_points_->setName("PointCloud");

		gl_points_->setPose(parent()->applyWorldRenderOffset(pointcloud_pose_));
		viz->get().insert(gl_points_);
		physical->get().insert(gl_points_);
	}

	// 1st call OR gridmap changed?
	if (!gui_uptodate_ && points_)
	{
		points_->getVisualizationInto(*gl_points_);
		gui_uptodate_ = true;
	}
}

void PointCloud::simul_pre_timestep([[maybe_unused]] const TSimulContext& context) {}
