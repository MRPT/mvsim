/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

DynamicsAckermannDrivetrain::ControllerRawForces::ControllerRawForces(
	DynamicsAckermannDrivetrain& veh)
	: ControllerBase(veh), setpoint_wheel_torque(0), setpoint_steer_ang(0)
{
}

// See base class docs
void DynamicsAckermannDrivetrain::ControllerRawForces::control_step(
	[[maybe_unused]] const DynamicsAckermannDrivetrain::TControllerInput& ci,
	DynamicsAckermannDrivetrain::TControllerOutput& co)
{
	co.drive_torque = this->setpoint_wheel_torque;
	co.steer_ang = this->setpoint_steer_ang;
}

void DynamicsAckermannDrivetrain::ControllerRawForces::load_config(
	const rapidxml::xml_node<char>& node)
{
	TParameterDefinitions params;
	params["torque"] = TParamEntry("%lf", &setpoint_wheel_torque);

	// Initial speed.
	params["steer_ang_deg"] = TParamEntry("%lf_deg", &this->setpoint_steer_ang);

	parse_xmlnode_children_as_param(node, params);
}

void DynamicsAckermannDrivetrain::ControllerRawForces::teleop_interface(
	const TeleopInput& in, TeleopOutput& out)
{
	ControllerBase::teleop_interface(in, out);

	switch (in.keycode)
	{
		case 'W':
		case 'w':
			setpoint_wheel_torque -= 1.0;
			break;

		case 'S':
		case 's':
			setpoint_wheel_torque += 1.0;
			break;

		case 'A':
		case 'a':
			setpoint_steer_ang += 1.0 * M_PI / 180.0;
			mrpt::keep_min(setpoint_steer_ang, veh_.getMaxSteeringAngle());
			break;

		case 'D':
		case 'd':
			setpoint_steer_ang -= 1.0 * M_PI / 180.0;
			mrpt::keep_max(setpoint_steer_ang, -veh_.getMaxSteeringAngle());
			break;

		case ' ':
			setpoint_wheel_torque = .0;
			break;
	};

	out.append_gui_lines += "[Controller=" + string(class_name()) +
							"] Teleop keys: w/s=incr/decr torques. "
							"a/d=left/right steering. spacebar=stop.\n";
	out.append_gui_lines += mrpt::format(
		"setpoint: t=%.03f steer=%.03f deg\n", setpoint_wheel_torque,
		setpoint_steer_ang * 180.0 / M_PI);
}
