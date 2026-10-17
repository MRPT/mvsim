/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleAckermann.h>

#include <algorithm>
#include <limits>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

DynamicsAckermann::ControllerTwistFrontSteerPID::ControllerTwistFrontSteerPID(
	DynamicsAckermann& veh)
	: ControllerBase(veh),
	  setpoint_lin_speed(0),
	  setpoint_ang_speed(0),
	  KP(100),
	  KI(0),
	  KD(0),
	  max_torque(100.0)
{
	// Get distance between wheels:
	dist_fWheels_ = veh_.wheels_info_[WHEEL_FL].y - veh_.wheels_info_[WHEEL_FR].y;
	r2f_L_ = veh_.wheels_info_[WHEEL_FL].x - veh_.wheels_info_[WHEEL_RL].x;
	ASSERT_(dist_fWheels_ > 0.0);
	ASSERT_(r2f_L_ > 0.0);
}

// See base class docs
void DynamicsAckermann::ControllerTwistFrontSteerPID::control_step(
	const DynamicsAckermann::TControllerInput& ci, DynamicsAckermann::TControllerOutput& co)
{
	// For each wheel:
	// 1) Compute desired velocity set-point (in m/s)
	// 2) Run the PI/PID for that wheel independently (in newtons)

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

	// Desired velocities for each wheel
	// In local vehicle frame:
	const std::vector<mrpt::math::TVector2D> desired_wheel_vels = veh_.getWheelsVelocityLocal(
		mrpt::math::TTwist2D(setpoint_lin_speed, 0.0, setpoint_ang_speed));

	ASSERT_(desired_wheel_vels.size() == 4);

	// Rotate to obtain the actual desired longitudinal velocity for each wheel:
	// FL:
	double vel_fl, vel_fr;
	double desired_fl_steer_ang, desired_fr_steer_ang;
	veh_.computeFrontWheelAngles(co.steer_ang, desired_fl_steer_ang, desired_fr_steer_ang);
	{
		const mrpt::poses::CPose2D wRotInv(0, 0, -desired_fl_steer_ang);
		mrpt::math::TPoint2D vel_w;
		wRotInv.composePoint(desired_wheel_vels[DynamicsAckermann::WHEEL_FL], vel_w);
		vel_fl = vel_w.x;
	}
	{
		const mrpt::poses::CPose2D wRotInv(0, 0, -desired_fr_steer_ang);
		mrpt::math::TPoint2D vel_w;
		wRotInv.composePoint(desired_wheel_vels[DynamicsAckermann::WHEEL_FR], vel_w);
		vel_fr = vel_w.x;
	}

	// Each driven wheel actual velocity, from its own spin (as a motor encoder
	// would measure it), so the loop also limits the wheel slip:
	const auto& wFL = veh_.getWheelInfo(DynamicsAckermann::WHEEL_FL);
	const auto& wFR = veh_.getWheelInfo(DynamicsAckermann::WHEEL_FR);
	const double act_vel_fl = wFL.getW() * 0.5 * wFL.diameter;
	const double act_vel_fr = wFR.getW() * 0.5 * wFR.diameter;

	// Apply controller:
	for (int i = 0; i < 2; i++)
	{
		PID_[i].KP = KP;
		PID_[i].KI = KI;
		PID_[i].KD = KD;
		PID_[i].N = N;
		PID_[i].max_out = max_torque;
		PID_[i].enable_antiwindup = enable_antiwindup;
		PID_[i].enable_reference_filter = enable_reference_filter;
		PID_[i].reference_filter_tau = reference_filter_tau;
		PID_[i].reference_filter_order = reference_filter_order;
	}

	// Optional: filter reference setpoints
	if (enable_reference_filter)
	{
		vel_fl = PID_[0].filterReference(vel_fl, ci.context.dt);
		vel_fr = PID_[1].filterReference(vel_fr, ci.context.dt);
	}

	// Feedforward for slope compensation:
	double ff = 0;
	if (enable_feedforward)
	{
		ff = feedforward_gain * veh_.estimateSlopeTorquePerWheel(2);
	}

	co.rl_torque = .0;
	co.rr_torque = .0;

	const bool setpointIsZero =
		std::abs(vel_fl) < zero_threshold && std::abs(vel_fr) < zero_threshold;

	if (setpointIsZero)
	{
		// (max_torque<=0 means no limit, as in PID_Controller)
		const double maxT = max_torque > 0 ? max_torque : std::numeric_limits<double>::infinity();
		// Braking toward a full stop: proportional action only, which always
		// opposes the current motion and vanishes at zero speed, so the vehicle
		// neither coasts nor rebounds due to the integral term.
		for (auto& pid : PID_)
		{
			pid.reset();
		}
		co.fl_torque = std::clamp(KP * act_vel_fl - ff, -maxT, maxT);
		co.fr_torque = std::clamp(KP * act_vel_fr - ff, -maxT, maxT);
		co.fl_motor = {KP * 0.5 * wFL.diameter, ff - KP * act_vel_fl, max_torque};
		co.fr_motor = {KP * 0.5 * wFR.diameter, ff - KP * act_vel_fr, max_torque};
	}
	else
	{
		// "-" because \tau<0 makes robot moves forwards.
		co.fl_torque = -PID_[0].compute(vel_fl - act_vel_fl, ci.context.dt, ff);
		co.fr_torque = -PID_[1].compute(vel_fr - act_vel_fr, ci.context.dt, ff);
		co.fl_motor = {
			PID_[0].lastOutputSensitivity() * 0.5 * wFL.diameter, PID_[0].lastUnsaturatedOutput(),
			max_torque};
		co.fr_motor = {
			PID_[1].lastOutputSensitivity() * 0.5 * wFR.diameter, PID_[1].lastUnsaturatedOutput(),
			max_torque};
	}
}

void DynamicsAckermann::ControllerTwistFrontSteerPID::load_config(
	const rapidxml::xml_node<char>& node)
{
	TParameterDefinitions params;
	params["KP"] = TParamEntry("%lf", &KP);
	params["KI"] = TParamEntry("%lf", &KI);
	params["KD"] = TParamEntry("%lf", &KD);
	params["N"] = TParamEntry("%lf", &N);
	params["max_torque"] = TParamEntry("%lf", &max_torque);
	params["zero_threshold"] = TParamEntry("%lf", &zero_threshold);
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

void DynamicsAckermann::ControllerTwistFrontSteerPID::teleop_interface(
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
			setpoint_ang_speed += 1.0 * M_PI / 180.0;
			break;

		case 'D':
		case 'd':
			setpoint_ang_speed -= 1.0 * M_PI / 180.0;
			break;

		case ' ':
			setpoint_lin_speed = .0;
			setpoint_ang_speed = .0;
			for (auto& pid : PID_)
			{
				pid.reset();
			}
			break;
		default:
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
				for (auto& pid : PID_)
				{
					pid.reset();
				}
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
