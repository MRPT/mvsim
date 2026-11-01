/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>

#include <algorithm>
#include <limits>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

DynamicsAckermannDrivetrain::ControllerTwistFrontSteerPID::ControllerTwistFrontSteerPID(
	DynamicsAckermannDrivetrain& veh)
	: ControllerBase(veh),
	  setpoint_lin_speed(0),
	  setpoint_ang_speed(0),
	  KP(100),
	  KI(0),
	  KD(0),
	  max_torque(400.0)
{
	// Get distance between wheels:
	dist_fWheels_ = veh_.wheels_info_[WHEEL_FL].y - veh_.wheels_info_[WHEEL_FR].y;
	r2f_L_ = veh_.wheels_info_[WHEEL_FL].x - veh_.wheels_info_[WHEEL_RL].x;

	ASSERT_(dist_fWheels_ > 0.0);
	ASSERT_(r2f_L_ > 0.0);
}

void DynamicsAckermannDrivetrain::ControllerTwistFrontSteerPID::control_step(
	const DynamicsAckermannDrivetrain::TControllerInput& ci,
	DynamicsAckermannDrivetrain::TControllerOutput& co)
{
	// 1st: desired steering angle:
	// --------------------------------
	if (setpoint_ang_speed == 0)
	{
		co.steer_ang = 0.0;
	}
	else
	{
		const double R = setpoint_lin_speed / setpoint_ang_speed;
		co.steer_ang = atan(r2f_L_ / R);
	}

	PID_.KP = KP;
	PID_.KI = KI;
	PID_.KD = KD;
	PID_.N = N;
	PID_.max_out = max_torque;
	PID_.enable_antiwindup = enable_antiwindup;
	PID_.enable_reference_filter = enable_reference_filter;
	PID_.reference_filter_tau = reference_filter_tau;
	PID_.reference_filter_order = reference_filter_order;

	// Driveshaft speed (from the driven wheels spin), so the loop also limits
	// the slip of the driven wheels:
	const double vel_act = veh_.driveshaftSpeed();
	double vel_des = setpoint_lin_speed;

	if (enable_reference_filter)
	{
		vel_des = PID_.filterReference(vel_des, ci.context.dt);
	}

	// Feedforward for slope compensation:
	double ff = 0;
	if (enable_feedforward)
	{
		// Drivetrain has a single drive torque shared across all wheels
		ff = feedforward_gain * veh_.estimateSlopeTorquePerWheel(4) * 4.0;
	}

	const double zero_threshold = 0.001;  // m/s

	if (std::abs(vel_des) < zero_threshold)
	{
		// (max_torque<=0 means no limit, as in PID_Controller)
		const double maxT = max_torque > 0 ? max_torque : std::numeric_limits<double>::infinity();
		// Braking toward a full stop: proportional action only, which always
		// opposes the current motion and vanishes at zero speed, so the vehicle
		// neither coasts nor rebounds due to the integral term.
		PID_.reset();
		co.drive_torque = std::clamp(KP * vel_act - ff, -maxT, maxT);
		co.drive_motor = {KP, ff - KP * vel_act, max_torque};
	}
	else
	{
		// "-" because \tau<0 makes robot moves forwards.
		co.drive_torque = -PID_.compute(vel_des - vel_act, ci.context.dt, ff);
		co.drive_motor = {PID_.lastOutputSensitivity(), PID_.lastUnsaturatedOutput(), max_torque};
	}
}

void DynamicsAckermannDrivetrain::ControllerTwistFrontSteerPID::load_config(
	const rapidxml::xml_node<char>& node)
{
	TParameterDefinitions params;
	params["KP"] = TParamEntry("%lf", &KP);
	params["KI"] = TParamEntry("%lf", &KI);
	params["KD"] = TParamEntry("%lf", &KD);
	params["N"] = TParamEntry("%lf", &N);
	params["max_torque"] = TParamEntry("%lf", &max_torque);
	params["enable_antiwindup"] = TParamEntry("%bool", &enable_antiwindup);
	params["enable_feedforward"] = TParamEntry("%bool", &enable_feedforward);
	params["feedforward_gain"] = TParamEntry("%lf", &feedforward_gain);
	params["enable_reference_filter"] = TParamEntry("%bool", &enable_reference_filter);
	params["reference_filter_tau"] = TParamEntry("%lf", &reference_filter_tau);
	params["reference_filter_order"] = TParamEntry("%d", &reference_filter_order);

	params["joyMaxLinSpeed"] = TParamEntry("%lf", &joyMaxLinSpeed);
	params["joyMaxAngSpeed"] = TParamEntry("%lf", &joyMaxAngSpeed);

	// Initial speed.
	params["V"] = TParamEntry("%lf", &this->setpoint_lin_speed);
	params["W"] = TParamEntry("%lf_deg", &this->setpoint_ang_speed);

	parse_xmlnode_children_as_param(node, params);
}

void DynamicsAckermannDrivetrain::ControllerTwistFrontSteerPID::teleop_interface(
	const TeleopInput& in, TeleopOutput& out)
{
	ControllerBase::teleop_interface(in, out);

	switch (in.keycode)
	{
		default:
			break;
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
			setpoint_ang_speed += 1.0 * M_PI / 180.0;
			break;

		case 'D':
		case 'd':
			setpoint_ang_speed -= 1.0 * M_PI / 180.0;
			break;

		case ' ':
			setpoint_lin_speed = .0;
			setpoint_ang_speed = .0;
			break;
	};

	out.append_gui_lines += "[Controller=" + std::string(class_name()) + "]";

	if (in.js && in.js->axes.size() >= 2)
	{
		const auto& js = in.js.value();
		const float js_x = js.axes[0];
		const float js_y = js.axes[1];

		setpoint_lin_speed = -js_y * joyMaxLinSpeed;
		setpoint_ang_speed = -js_x * joyMaxAngSpeed;

		if (js.buttons.size() > 7)
		{
			if (js.buttons[5])
			{
				joyMaxLinSpeed *= 1.01;
			}
			if (js.buttons[7])
			{
				joyMaxLinSpeed /= 1.01;
			}

			if (js.buttons[4])
			{
				joyMaxAngSpeed *= 1.01;
			}
			if (js.buttons[6])
			{
				joyMaxAngSpeed /= 1.01;
			}

			if (js.buttons[3])	// brake
			{
				setpoint_lin_speed = 0;
				setpoint_ang_speed = 0;
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
		"setpoint: lin=%.03f ang=%.03f deg/s\n", setpoint_lin_speed,
		180.0 / M_PI * setpoint_ang_speed);
}
