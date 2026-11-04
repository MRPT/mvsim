/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>

using namespace mvsim;

void DynamicsAckermannDrivetrain::ControllerJointCommands::control_step(
	const DynamicsAckermannDrivetrain::TControllerInput& ci,
	DynamicsAckermannDrivetrain::TControllerOutput& co)
{
	co.drive_torque = joints_.computeCentralTractionTorque(
		veh_.driveshaftSpeed(), veh_.driveshaftRadius(), ci.context.dt, co.drive_motor);

	joints_.ackermannSteering(
		veh_.steer_ang_, {veh_.wheels_info_[WHEEL_FL].yaw, veh_.wheels_info_[WHEEL_FR].yaw},
		co.steer_ang, co.front_wheels_steer_ang);
}

void DynamicsAckermannDrivetrain::ControllerJointCommands::on_post_step(
	const TSimulContext& context)
{
	joints_.updateStates(
		veh_, context.dt, {veh_.steer_ang_, veh_.driveshaftSpeed() / veh_.driveshaftRadius()});
}

void DynamicsAckermannDrivetrain::ControllerJointCommands::load_config(
	const rapidxml::xml_node<char>& node)
{
	WheelJointsInterface::AckermannLayout layout;
	layout.fl = WHEEL_FL;
	layout.fr = WHEEL_FR;
	layout.centralTraction = true;
	layout.maxSteer = veh_.getMaxSteeringAngle();
	layout.flSteerRange = veh_.wheelSteeringRange(WHEEL_FL);
	layout.frSteerRange = veh_.wheelSteeringRange(WHEEL_FR);
	layout.maxSteerRate = veh_.getMaxSteeringRate();
	joints_.loadConfigAckermann(node, veh_, layout);
}
