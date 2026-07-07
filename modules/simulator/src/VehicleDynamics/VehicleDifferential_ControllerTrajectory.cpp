/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/World.h>

#include "xml_utils.h"

using namespace mvsim;

DynamicsDifferential::ControllerTrajectory::ControllerTrajectory(DynamicsDifferential& veh)
	: ControllerBase(veh)
{
	// Signal that friction reaction forces must not be applied to the
	// chassis body, the twist is imposed directly by this controller.
	veh_.idealControllerActive_ = true;
}

void DynamicsDifferential::ControllerTrajectory::control_step(
	[[maybe_unused]] const DynamicsDifferential::TControllerInput& ci,
	DynamicsDifferential::TControllerOutput& co)
{
	co.wheel_torque_l = 0;
	co.wheel_torque_r = 0;
}

void DynamicsDifferential::ControllerTrajectory::on_post_step(const TSimulContext& context)
{
	const auto twist =
		follower_.computeTwist(context.simul_time, mrpt::math::TPose2D(veh_.getPose()));
	veh_.setRefVelocityLocal(twist);
}

void DynamicsDifferential::ControllerTrajectory::load_config(const rapidxml::xml_node<char>& node)
{
	const auto& vars = veh_.getSimulableWorldObject()->user_defined_variables();
	parse_trajectory_controller_config(
		node, vars, "[DynamicsDifferential::ControllerTrajectory]", follower_, vizHeight_);
}

bool DynamicsDifferential::ControllerTrajectory::getTrajectoryPlotPoints(
	std::vector<mrpt::math::TPoint2D>& pts, double& height) const
{
	if (follower_.empty())
	{
		return false;
	}
	pts.clear();
	for (const auto& wp : follower_.waypoints())
	{
		pts.push_back(wp.xy);
	}
	height = vizHeight_;
	return true;
}
