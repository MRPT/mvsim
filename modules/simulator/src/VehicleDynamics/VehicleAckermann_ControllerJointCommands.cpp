/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleAckermann.h>

using namespace mvsim;

void DynamicsAckermann::ControllerJointCommands::control_step(
	const DynamicsAckermann::TControllerInput& ci, DynamicsAckermann::TControllerOutput& co)
{
	std::vector<MotorTorqueModel> motors;
	const auto torques = joints_.computeWheelTorques(veh_, ci.context.dt, motors);
	co.rl_torque = torques[WHEEL_RL];
	co.rr_torque = torques[WHEEL_RR];
	co.fl_torque = torques[WHEEL_FL];
	co.fr_torque = torques[WHEEL_FR];
	co.rl_motor = motors[WHEEL_RL];
	co.rr_motor = motors[WHEEL_RR];
	co.fl_motor = motors[WHEEL_FL];
	co.fr_motor = motors[WHEEL_FR];

	joints_.ackermannSteering(
		veh_.steer_ang_, {veh_.wheels_info_[WHEEL_FL].yaw, veh_.wheels_info_[WHEEL_FR].yaw},
		co.steer_ang, co.front_wheels_steer_ang);
}

void DynamicsAckermann::ControllerJointCommands::on_post_step(const TSimulContext& context)
{
	joints_.updateStates(veh_, context.dt, {veh_.steer_ang_, 0.0});
}

void DynamicsAckermann::ControllerJointCommands::load_config(const rapidxml::xml_node<char>& node)
{
	WheelJointsInterface::AckermannLayout layout;
	layout.fl = WHEEL_FL;
	layout.fr = WHEEL_FR;
	layout.centralTraction = false;
	layout.maxSteer = veh_.getMaxSteeringAngle();
	layout.flSteerRange = veh_.wheelSteeringRange(WHEEL_FL);
	layout.frSteerRange = veh_.wheelSteeringRange(WHEEL_FR);
	layout.maxSteerRate = veh_.getMaxSteeringRate();
	joints_.loadConfigAckermann(node, veh_, layout);
}
