/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
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
	  is_floating_(true),
	  x_min_(-25.0),
	  x_max_(25.0),
	  y_min_(-25.0),
	  y_max_(25.0),
	  interval_(5.0),
	  color_(0xe0, 0xe0, 0xe0, 0xff),
	  line_width_(1.0)
{
	// Create opengl object: in this class, we'll store most state data directly
	// in the mrpt::opengl object.
	loadConfigFrom(root);
}

GroundGrid::~GroundGrid() = default;

void GroundGrid::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	if (!root) return;	// Assume defaults

	TParameterDefinitions params;
	params["floating"] = TParamEntry("%bool", &is_floating_);
	params["floating_focus"] = TParamEntry("%s", &float_center_at_vehicle_name_);
	params["color"] = TParamEntry("%color", &color_);
	params["line_width"] = TParamEntry("%lf", &line_width_);

	params["x_min"] = TParamEntry("%lf", &x_min_);
	params["x_max"] = TParamEntry("%lf", &x_max_);
	params["y_min"] = TParamEntry("%lf", &y_min_);
	params["y_max"] = TParamEntry("%lf", &y_max_);
	params["interval"] = TParamEntry("%lf", &interval_);

	parse_xmlnode_children_as_param(*root, params, world_->user_defined_variables());

	// If a vehicle name is given, setting "is_floating=true" by the user is
	// optional:
	if (!float_center_at_vehicle_name_.empty()) is_floating_ = true;
}

void GroundGrid::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	using namespace mrpt::math;

	// 1st call OR gridmap changed?
	if (!gl_groundgrid_ && viz)
	{
		gl_groundgrid_ = mrpt::opengl::CGridPlaneXY::Create();
		gl_groundgrid_->setPlaneLimits(x_min_, x_max_, y_min_, y_max_);
		gl_groundgrid_->setGridFrequency(interval_);
		gl_groundgrid_->setColor_u8(color_);
		gl_groundgrid_->setLineWidth(line_width_);

		viz->get().insert(gl_groundgrid_);
	}

	// Update:
	mrpt::math::TPoint3D center_offset(.0, .0, .0);
	if (is_floating_)
	{
		// Centered at a vehicle:
		const World::VehicleList& vehs = world_->getListOfVehicles();
		// Look for the vehicle by its name:
		World::VehicleList::const_iterator it_veh = vehs.find(float_center_at_vehicle_name_);
		// not found -> error:
		if (!float_center_at_vehicle_name_.empty() && it_veh == vehs.end())
			throw std::runtime_error(mrpt::format(
				"[GroundGrid] *ERROR* Cannot find vehicle named '%s' to "
				"focus on.",
				float_center_at_vehicle_name_.c_str()));

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
	ASSERT_(interval_ > .0);
	center_offset.x = interval_ * ::floor(std::abs(center_offset.x) / interval_) *
					  (center_offset.x < 0 ? -1. : 1.);
	center_offset.y = interval_ * ::floor(std::abs(center_offset.y) / interval_) *
					  (center_offset.y < 0 ? -1. : 1.);
	gl_groundgrid_->setLocation(center_offset);
}
