/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
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
	m_gui_uptodate = false;

	if (auto x2d = root->first_node("file_txt_2d"); x2d && x2d->value())
	{
		const string sFile = m_world->resolvePath(x2d->value());

		m_points = mrpt::maps::CSimplePointsMap::Create();
		m_points->load2D_from_text_file(sFile);
	}
	else if (auto x3d = root->first_node("file_txt_3d"); x3d && x3d->value())
	{
		const string sFile = m_world->resolvePath(x3d->value());

		m_points = mrpt::maps::CSimplePointsMap::Create();
		m_points->load3D_from_text_file(sFile);
	}
	else
	{
		THROW_EXCEPTION("Error: No valid <file_*></file_*> XML tag was found");
	}

	{
		// Other general params:
		TParameterDefinitions ps;
		ps["points_size"] = TParamEntry("%lf", &m_render_points_size);
		ps["pose_3d"] = TParamEntry("%pose3d", &m_pointcloud_pose);
		parse_xmlnode_children_as_param(*root, ps);
	}

	m_points->renderOptions.point_size = m_render_points_size;

	m_gui_uptodate = false;
}

void PointCloud::internalGuiUpdate(
	mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
	bool childrenOnly)
{
	using namespace mrpt::math;

	// 1st time call?? -> Create objects
	if (!m_gl_points)
	{
		m_gl_points = mrpt::opengl::CSetOfObjects::Create();
		m_gl_points->setName("PointCloud");
		m_gl_points->setPose(m_pointcloud_pose);
		viz.insert(m_gl_points);
		physical.insert(m_gl_points);
	}

	// 1st call OR gridmap changed?
	if (!m_gui_uptodate && m_points)
	{
		m_points->getVisualizationInto(*m_gl_points);
		m_gui_uptodate = true;
	}
}

void PointCloud::simul_pre_timestep(const TSimulContext& context) {}
