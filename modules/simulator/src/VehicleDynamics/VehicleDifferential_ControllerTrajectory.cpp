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

	bool loop = true;
	double lookAheadDistance = 0.5;
	double maxAngularSpeed = 2.0;

	TParameterDefinitions params;
	params["loop"] = TParamEntry("%bool", &loop);
	params["lookahead_distance"] = TParamEntry("%lf", &lookAheadDistance);
	params["max_angular_speed"] = TParamEntry("%lf", &maxAngularSpeed);

	parse_xmlnode_attribs(node, params, vars, "[DynamicsDifferential::ControllerTrajectory]");
	parse_xmlnode_children_as_param(
		node, params, vars, "[DynamicsDifferential::ControllerTrajectory]");

	std::vector<PoseTrajectoryFollower::Waypoint> waypoints;
	for (auto n = node.first_node("waypoint"); n; n = n->next_sibling("waypoint"))
	{
		double t = 0, x = 0, y = 0;
		TParameterDefinitions wpParams;
		wpParams["t"] = TParamEntry("%lf", &t);
		wpParams["x"] = TParamEntry("%lf", &x);
		wpParams["y"] = TParamEntry("%lf", &y);

		parse_xmlnode_attribs(*n, wpParams, vars, "[DynamicsDifferential::ControllerTrajectory]");

		waypoints.emplace_back(t, x, y);
	}

	if (waypoints.size() < 2)
	{
		THROW_EXCEPTION(
			"[DynamicsDifferential::ControllerTrajectory] At least 2 "
			"<waypoint t=\"..\" x=\"..\" y=\"..\"/> entries are required inside "
			"<controller class=\"trajectory\">");
	}

	follower_.setWaypoints(std::move(waypoints));
	follower_.setLoop(loop);
	follower_.setLookAheadDistance(lookAheadDistance);
	follower_.setMaxAngularSpeed(maxAngularSpeed);
}
