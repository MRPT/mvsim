/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/GroundGrid.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

GroundGrid::GroundGrid(World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent),
	  m_is_floating(true),
	  m_x_min(-25.0),
	  m_x_max(25.0),
	  m_y_min(-25.0),
	  m_y_max(25.0),
	  m_interval(5.0),
	  m_color(0xe0, 0xe0, 0xe0, 0xff),
	  m_line_width(1.0)
{
	// Create opengl object: in this class, we'll store most state data directly
	// in the mrpt::opengl object.
	loadConfigFrom(root);
}

GroundGrid::~GroundGrid() {}
void GroundGrid::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	if (!root) return;	// Assume defaults

	TParameterDefinitions params;
	params["floating"] = TParamEntry("%bool", &m_is_floating);
	params["floating_focus"] =
		TParamEntry("%s", &m_float_center_at_vehicle_name);
	params["color"] = TParamEntry("%color", &m_color);
	params["line_width"] = TParamEntry("%lf", &m_line_width);

	params["x_min"] = TParamEntry("%lf", &m_x_min);
	params["x_max"] = TParamEntry("%lf", &m_x_max);
	params["y_min"] = TParamEntry("%lf", &m_y_min);
	params["y_max"] = TParamEntry("%lf", &m_y_max);
	params["interval"] = TParamEntry("%lf", &m_interval);

	parse_xmlnode_children_as_param(*root, params);

	// If a vehicle name is given, setting "is_floating=true" by the user is
	// optional:
	if (!m_float_center_at_vehicle_name.empty()) m_is_floating = true;
}

void GroundGrid::internalGuiUpdate(
	mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
	bool childrenOnly)
{
	using namespace mrpt::math;

	// 1st call OR gridmap changed?
	if (!m_gl_groundgrid)
	{
		m_gl_groundgrid = mrpt::opengl::CGridPlaneXY::Create();
		m_gl_groundgrid->setPlaneLimits(m_x_min, m_x_max, m_y_min, m_y_max);
		m_gl_groundgrid->setGridFrequency(m_interval);
		m_gl_groundgrid->setColor_u8(m_color);
		m_gl_groundgrid->setLineWidth(m_line_width);

		viz.insert(m_gl_groundgrid);
	}

	// Update:
	mrpt::math::TPoint3D center_offset(.0, .0, .0);
	if (m_is_floating)
	{
		// Centered at a vehicle:
		const World::VehicleList& vehs = m_world->getListOfVehicles();
		// Look for the vehicle by its name:
		World::VehicleList::const_iterator it_veh =
			vehs.find(m_float_center_at_vehicle_name);
		// not found -> error:
		if (!m_float_center_at_vehicle_name.empty() && it_veh == vehs.end())
			throw std::runtime_error(mrpt::format(
				"[GroundGrid] *ERROR* Cannot find vehicle named '%s' to "
				"focus on.",
				m_float_center_at_vehicle_name.c_str()));

		// If the user didn't specify any name, assume it's the first vehicle,
		// or ignore it if none exists:
		if (it_veh == vehs.end()) it_veh = vehs.begin();

		if (it_veh != vehs.end())
		{
			const mrpt::math::TPose3D& pose = it_veh->second->getPose();
			center_offset.x = pose.x;
			center_offset.y = pose.y;
		}
	}

	// "Discretize" offset for a better visual impact:
	ASSERT_(m_interval > .0);
	center_offset.x = m_interval *
					  ::floor(std::abs(center_offset.x) / m_interval) *
					  (center_offset.x < 0 ? -1. : 1.);
	center_offset.y = m_interval *
					  ::floor(std::abs(center_offset.y) / m_interval) *
					  (center_offset.y < 0 ? -1. : 1.);
	m_gl_groundgrid->setLocation(center_offset);
}
