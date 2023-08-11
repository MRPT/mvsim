/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleAckermann.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

DynamicsAckermann::ControllerFrontSteerPID::ControllerFrontSteerPID(
	DynamicsAckermann& veh)
	: ControllerBase(veh),
	  setpoint_lin_speed(0),
	  setpoint_steer_ang(0),
	  KP(100),
	  KI(0),
	  KD(0),
	  max_torque(100.0),
	  twist_control_(veh)
{
	// Get distance between wheels:
	r2f_L_ = veh_.wheels_info_[WHEEL_FL].x - veh_.wheels_info_[WHEEL_RL].x;
	ASSERT_(r2f_L_ > 0.0);
}

// See base class docs
void DynamicsAckermann::ControllerFrontSteerPID::control_step(
	const DynamicsAckermann::TControllerInput& ci,
	DynamicsAckermann::TControllerOutput& co)
{
	// Equivalent v/w velocities:
	const double v = setpoint_lin_speed;
	double w;
	if (setpoint_steer_ang == 0.0)
	{
		w = 0.0;
	}
	else
	{
		// ang = atan(r2f_L/R)  ->  R= r2f_L / tan(ang)
		// R = v/w              ->   w=v/R
		const double R = r2f_L_ / tan(setpoint_steer_ang);
		w = v / R;
	}

	// Let the twist controller do the calculations:
	twist_control_.setpoint_lin_speed = v;
	twist_control_.setpoint_ang_speed = w;

	twist_control_.KP = KP;
	twist_control_.KI = KI;
	twist_control_.KD = KD;
	twist_control_.max_torque = max_torque;

	twist_control_.control_step(ci, co);
	co.steer_ang = setpoint_steer_ang;	// Mainly for the case of v=0
}

void DynamicsAckermann::ControllerFrontSteerPID::load_config(
	const rapidxml::xml_node<char>& node)
{
	TParameterDefinitions params;
	params["KP"] = TParamEntry("%lf", &KP);
	params["KI"] = TParamEntry("%lf", &KI);
	params["KD"] = TParamEntry("%lf", &KD);
	params["max_torque"] = TParamEntry("%lf", &max_torque);

	// Initial speed.
	params["V"] = TParamEntry("%lf", &this->setpoint_lin_speed);
	params["STEER_ANG"] = TParamEntry("%lf_deg", &this->setpoint_steer_ang);

	parse_xmlnode_children_as_param(node, params);
}

void DynamicsAckermann::ControllerFrontSteerPID::teleop_interface(
	const TeleopInput& in, TeleopOutput& out)
{
	ControllerBase::teleop_interface(in, out);

	switch (in.keycode)
	{
		case 'W':
		case 'w':
			setpoint_lin_speed += 0.1;
			break;

		case 'S':
		case 's':
			setpoint_lin_speed -= 0.1;
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
			setpoint_lin_speed = .0;
			break;
	};

	out.append_gui_lines += "[Controller=" + std::string(class_name()) + "]";

	if (in.js)
	{
		const auto& js = in.js.value();
		setpoint_lin_speed = -js.y * joyMaxLinSpeed;
		setpoint_steer_ang = -js.x * joyMaxSteerAng;

		if (js.buttons.size() >= 7)
		{
			if (js.buttons[5]) joyMaxLinSpeed *= 1.01;
			if (js.buttons[7]) joyMaxLinSpeed /= 1.01;

			if (js.buttons[4]) joyMaxSteerAng *= 1.01;
			if (js.buttons[6]) joyMaxSteerAng /= 1.01;

			if (js.buttons[3])	// brake
			{
				setpoint_lin_speed = 0;
			}
		}

		out.append_gui_lines += mrpt::format(
			"Teleop joystick:\n"
			"maxLinSpeed=%.03f m/s\n"
			"maxSteerAng=%.03f deg\n",
			joyMaxLinSpeed, mrpt::RAD2DEG(joyMaxSteerAng));
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
		"setpoint: v=%.03f steer=%.03f deg\n", setpoint_lin_speed,
		setpoint_steer_ang * 180.0 / M_PI);
}
