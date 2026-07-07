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
	// Floor |vx| to a small positive minimum (keeping its sign, or assuming
	// forward motion when exactly zero) instead of switching to a different
	// approximation near vx==0, so steer_ang stays continuous through it.
	constexpr double kMinAbsVx = 1e-3;
	const double vxSign = (lastTwist_.vx >= 0) ? 1.0 : -1.0;
	const double vxForSteer = vxSign * std::max(std::abs(lastTwist_.vx), kMinAbsVx);
	co.steer_ang = std::atan(lastTwist_.omega * r2f_L_ / vxForSteer);

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
	parse_trajectory_controller_config(
		node, vars, "[DynamicsAckermann::ControllerTrajectory]", follower_, vizHeight_);
}

bool DynamicsAckermann::ControllerTrajectory::getTrajectoryPlotPoints(
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
