/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleDifferential.h>
// #include <mvsim/World.h>
// #include <rapidxml.hpp>

using namespace mvsim;
using namespace std;

// See base class docs
void DynamicsDifferential::ControllerRawForces::control_step(
	[[maybe_unused]] const DynamicsDifferential::TControllerInput& ci,
	DynamicsDifferential::TControllerOutput& co)
{
	co.wheel_torque_l = this->setpoint_wheel_torque_l;
	co.wheel_torque_r = this->setpoint_wheel_torque_r;
}

void DynamicsDifferential::ControllerRawForces::teleop_interface(
	const TeleopInput& in, TeleopOutput& out)
{
	ControllerBase::teleop_interface(in, out);

	const double dt = setpoint_teleop_steps;

	switch (in.keycode)
	{
		case 'W':
		case 'w':
			setpoint_wheel_torque_l -= dt;
			setpoint_wheel_torque_r -= dt;
			break;

		case 'S':
		case 's':
			setpoint_wheel_torque_l += dt;
			setpoint_wheel_torque_r += dt;
			break;

		case 'A':
		case 'a':
			setpoint_wheel_torque_l += dt;
			setpoint_wheel_torque_r -= dt;
			break;

		case 'D':
		case 'd':
			setpoint_wheel_torque_l -= dt;
			setpoint_wheel_torque_r += dt;
			break;

		case ' ':
			setpoint_wheel_torque_l = setpoint_wheel_torque_r = 0.0;
			break;
	};

	out.append_gui_lines += "[Controller=" + string(class_name()) +
							"] Teleop keys:\n"
							"w/s=incr/decr both torques.\n"
							"a/d=left/right. spacebar=stop.\n";
	out.append_gui_lines += mrpt::format(
		"setpoint: tl=%.03f tr=%.03f deg\n", setpoint_wheel_torque_l, setpoint_wheel_torque_r);
}
