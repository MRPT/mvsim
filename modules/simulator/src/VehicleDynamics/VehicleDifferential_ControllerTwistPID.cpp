/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleDifferential.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

DynamicsDifferential::ControllerTwistPID::ControllerTwistPID(DynamicsDifferential& veh)
	: ControllerBase(veh)
{
	// Get distance between wheels:
	// Warning: the controller *assumes* that both wheels are parallel (as it's
	// a rule in differential robots!!)
	distWheels_ = veh_.wheels_info_[0].y - veh_.wheels_info_[1].y;
	ASSERT_(distWheels_ > 0);
}

// See base class docs
void DynamicsDifferential::ControllerTwistPID::control_step(
	const DynamicsDifferential::TControllerInput& ci, DynamicsDifferential::TControllerOutput& co)
{
	const auto sp = setpoint();

	// For each wheel:
	// 1) Compute desired velocity set-point (in m/s)
	// 2) Run the PI/PID for that wheel independently (in newtons)
	const double spVelL = sp.vx - 0.5 * sp.omega * distWheels_;
	const double spVelR = sp.vx + 0.5 * sp.omega * distWheels_;

	// Compute each wheel actual velocity (from an "odometry" estimation of
	// velocity, not ground-truth!):
	const mrpt::math::TTwist2D vehVelOdo = veh_.getVelocityLocalOdoEstimate();
	const double actVelL = vehVelOdo.vx - 0.5 * vehVelOdo.omega * distWheels_;
	const double actVelR = vehVelOdo.vx + 0.5 * vehVelOdo.omega * distWheels_;

	// Apply controller:
	for (auto& pid : PIDs_)
	{
		pid.KP = KP;
		pid.KI = KI;
		pid.KD = KD;
		pid.max_out = max_torque;
	}

	// "-" because \tau<0 makes robot moves forwards.
	const double followErrorL = spVelL - actVelL;
	const double followErrorR = spVelR - actVelR;

	const double zeroThres = 0.001;	 // m/s

	if (std::abs(spVelL) < zeroThres &&	 //
		std::abs(spVelR) < zeroThres &&	 //
		std::abs(spVelR) < zeroThres &&	 //
		std::abs(spVelR) < zeroThres)
	{
		co.wheel_torque_l = 0;
		co.wheel_torque_r = 0;
		for (auto& pid : PIDs_) pid.reset();
	}
	else
	{
		co.wheel_torque_l = -PIDs_[0].compute(followErrorL, ci.context.dt);
		co.wheel_torque_r = -PIDs_[1].compute(followErrorR, ci.context.dt);
	}
}

void DynamicsDifferential::ControllerTwistPID::load_config(const rapidxml::xml_node<char>& node)
{
	TParameterDefinitions params;
	params["KP"] = TParamEntry("%lf", &KP);
	params["KI"] = TParamEntry("%lf", &KI);
	params["KD"] = TParamEntry("%lf", &KD);
	params["max_torque"] = TParamEntry("%lf", &max_torque);

	// Initial speed.
	params["V"] = TParamEntry("%lf", &setpoint_.vx);
	params["W"] = TParamEntry("%lf_deg", &setpoint_.omega);

	parse_xmlnode_children_as_param(node, params);
}

void DynamicsDifferential::ControllerTwistPID::teleop_interface(
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
			for (auto& pid : PIDs_) pid.reset();
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

			if (js.buttons[3])	// brake
			{
				setpoint_ = {0, 0, 0};
				for (auto& pid : PIDs_) pid.reset();
			}
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
