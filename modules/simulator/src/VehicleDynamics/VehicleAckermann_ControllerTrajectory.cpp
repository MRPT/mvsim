/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/World.h>

#include "xml_utils.h"

using namespace mvsim;

DynamicsAckermann::ControllerTrajectory::ControllerTrajectory(DynamicsAckermann& veh)
	: ControllerBase(veh)
{
	// Pre-compute the rear-to-front axle distance (wheelbase), needed to
	// derive the equivalent Ackermann steering angle from the (vx, omega)
	// twist computed by the trajectory follower: steer_ang = atan(omega*L/vx)
	r2f_L_ = veh_.wheels_info_[WHEEL_FL].x - veh_.wheels_info_[WHEEL_RL].x;
	ASSERT_(r2f_L_ > 0.0);

	// Signal that friction reaction forces must not be applied to the
	// chassis body, the twist is imposed directly by this controller.
	veh_.idealControllerActive_ = true;
}

void DynamicsAckermann::ControllerTrajectory::control_step(
	const DynamicsAckermann::TControllerInput& ci, DynamicsAckermann::TControllerOutput& co)
{
	// Ideal controller: no wheel torques are needed, the twist is imposed
	// directly in on_post_step(). We still need to fill steer_ang so that
	// computeFrontWheelAngles() receives a sensible value.
	co.fl_torque = 0;
	co.fr_torque = 0;
	co.rl_torque = 0;
	co.rr_torque = 0;

	lastTwist_ = follower_.computeTwist(ci.context.simul_time, mrpt::math::TPose2D(veh_.getPose()));

	// Kinematic relation:  omega = vx * tan(delta) / L  =>  delta = atan(omega * L / vx)
	// When vx == 0 we fall back to a direct omega-based clamped angle.
	if (std::abs(lastTwist_.vx) > 1e-3)
	{
		co.steer_ang = std::atan(lastTwist_.omega * r2f_L_ / lastTwist_.vx);
	}
	else
	{
		co.steer_ang = (lastTwist_.omega >= 0 ? 1.0 : -1.0) *
					   std::min(std::abs(lastTwist_.omega * r2f_L_), veh_.getMaxSteeringAngle());
	}

	co.steer_ang =
		std::clamp(co.steer_ang, -veh_.getMaxSteeringAngle(), veh_.getMaxSteeringAngle());
}

void DynamicsAckermann::ControllerTrajectory::on_post_step(
	[[maybe_unused]] const TSimulContext& context)
{
	// Fake / ideal controller: directly override the vehicle twist with the
	// twist computed in control_step(). Box2D integration will propagate
	// this to the pose.
	veh_.setRefVelocityLocal(lastTwist_);
}

void DynamicsAckermann::ControllerTrajectory::load_config(const rapidxml::xml_node<char>& node)
{
	const auto& vars = veh_.getSimulableWorldObject()->user_defined_variables();

	bool loop = true;
	double lookAheadDistance = 0.5;
	double maxAngularSpeed = 2.0;

	TParameterDefinitions params;
	params["loop"] = TParamEntry("%bool", &loop);
	params["lookahead_distance"] = TParamEntry("%lf", &lookAheadDistance);
	params["max_angular_speed"] = TParamEntry("%lf", &maxAngularSpeed);

	parse_xmlnode_attribs(node, params, vars, "[DynamicsAckermann::ControllerTrajectory]");
	parse_xmlnode_children_as_param(
		node, params, vars, "[DynamicsAckermann::ControllerTrajectory]");

	std::vector<PoseTrajectoryFollower::Waypoint> waypoints;
	for (auto n = node.first_node("waypoint"); n; n = n->next_sibling("waypoint"))
	{
		double t = 0, x = 0, y = 0;
		TParameterDefinitions wpParams;
		wpParams["t"] = TParamEntry("%lf", &t);
		wpParams["x"] = TParamEntry("%lf", &x);
		wpParams["y"] = TParamEntry("%lf", &y);

		parse_xmlnode_attribs(*n, wpParams, vars, "[DynamicsAckermann::ControllerTrajectory]");

		waypoints.emplace_back(t, x, y);
	}

	if (waypoints.size() < 2)
	{
		THROW_EXCEPTION(
			"[DynamicsAckermann::ControllerTrajectory] At least 2 "
			"<waypoint t=\"..\" x=\"..\" y=\"..\"/> entries are required inside "
			"<controller class=\"trajectory\">");
	}

	follower_.setWaypoints(std::move(waypoints));
	follower_.setLoop(loop);
	follower_.setLookAheadDistance(lookAheadDistance);
	follower_.setMaxAngularSpeed(maxAngularSpeed);
}
