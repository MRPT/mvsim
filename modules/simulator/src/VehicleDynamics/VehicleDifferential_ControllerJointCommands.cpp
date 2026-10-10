/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleDifferential.h>

using namespace mvsim;

void DynamicsDifferential::ControllerJointCommands::control_step(
	const DynamicsDifferential::TControllerInput& ci, DynamicsDifferential::TControllerOutput& co)
{
	co.wheel_torques = joints_.computeWheelTorques(veh_, ci.context.dt);
}

void DynamicsDifferential::ControllerJointCommands::on_post_step(
	[[maybe_unused]] const TSimulContext& context)
{
	joints_.updateStates(veh_);
}

void DynamicsDifferential::ControllerJointCommands::load_config(
	const rapidxml::xml_node<char>& node)
{
	joints_.loadConfig(node, veh_);
}
