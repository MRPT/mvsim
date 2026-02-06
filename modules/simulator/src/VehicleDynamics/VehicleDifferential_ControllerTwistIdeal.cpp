/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleDifferential.h>

using namespace mvsim;

DynamicsDifferential::ControllerTwistIdeal::ControllerTwistIdeal(DynamicsDifferential& veh)
	: ControllerBase(veh)
{
	// Get distance between wheels:
	// Warning: the controller *assumes* that both wheels are parallel (as it's
	// a rule in differential robots!!)
	distWheels_ = veh_.wheels_info_[0].y - veh_.wheels_info_[1].y;
	ASSERT_(distWheels_ > 0);
}

void DynamicsDifferential::ControllerTwistIdeal::control_step(
	[[maybe_unused]] const DynamicsDifferential::TControllerInput& ci,
	DynamicsDifferential::TControllerOutput& co)
{
	co.wheel_torque_l = 0;
	co.wheel_torque_r = 0;
}

void DynamicsDifferential::ControllerTwistIdeal::on_post_step(
	[[maybe_unused]] const TSimulContext& context)
{
	// Fake controller: just set the setpoint as state and we are done.
	const auto sp = setpoint();
	this->veh_.setTwist(sp);
}

void DynamicsDifferential::ControllerTwistIdeal::teleop_interface(
	const TeleopInput& in, TeleopOutput& out)
{
	ControllerBase::teleop_interface(in, out);

	auto lck = mrpt::lockHelper(setpointMtx_);

	switch (in.keycode)
	{
		case 'W':
		case 'w':
			setpoint_.vx += 0.1;
			break;

		case 'S':
		case 's':
			setpoint_.vx -= 0.1;
			break;

		case 'A':
		case 'a':
			setpoint_.omega += 2.0 * M_PI / 180;
			break;

		case 'D':
		case 'd':
			setpoint_.omega -= 2.0 * M_PI / 180;
			break;

		case ' ':
		{
			setpoint_ = {0, 0, 0};
		}
		break;
	};

	out.append_gui_lines += "[Controller=" + std::string(class_name()) + "]";

	if (in.js && in.js->axes.size() >= 2)
	{
		const auto& js = in.js.value();
		const float js_x = js.axes[0];
		const float js_y = js.axes[1];

		setpoint_.vx = -js_y * joyMaxLinSpeed;
		setpoint_.omega = -js_x * joyMaxAngSpeed;

		if (js.buttons.size() >= 7)
		{
			if (js.buttons[5]) joyMaxLinSpeed *= 1.01;
			if (js.buttons[7]) joyMaxLinSpeed /= 1.01;

			if (js.buttons[4]) joyMaxAngSpeed *= 1.01;
			if (js.buttons[6]) joyMaxAngSpeed /= 1.01;

			// brake
			if (js.buttons[3]) setpoint_ = {0, 0, 0};
		}

		out.append_gui_lines += mrpt::format(
			"Teleop joystick:\n"
			"maxLinSpeed=%.03f m/s\n"
			"maxAngSpeed=%.03f deg/s\n",
			joyMaxLinSpeed, mrpt::RAD2DEG(joyMaxAngSpeed));
	}
	else
	{
		out.append_gui_lines +=
			"Teleop keys:\n"
			"w/s=forward/backward.\n"
			"a/d=left/right.\n"
			"spacebar=stop.\n";
	}

	out.append_gui_lines += mrpt::format(
		"setpoint: lin=%.03f ang=%.03f deg/s\n", setpoint_.vx, 180.0 / M_PI * setpoint_.omega);
}
