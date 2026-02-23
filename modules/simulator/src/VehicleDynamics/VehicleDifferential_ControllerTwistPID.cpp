/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2025-2026 Fernando Cañadas Aránega                        |
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
	double spVelL = sp.vx - 0.5 * sp.omega * distWheels_;
	double spVelR = sp.vx + 0.5 * sp.omega * distWheels_;

	// Compute each wheel actual velocity (from an "odometry" estimation of
	// velocity, not ground-truth!):
	const mrpt::math::TTwist2D vehVelOdo = veh_.getVelocityLocalOdoEstimate();
	const double actVelL = vehVelOdo.vx - 0.5 * vehVelOdo.omega * distWheels_;
	const double actVelR = vehVelOdo.vx + 0.5 * vehVelOdo.omega * distWheels_;

	// Configure PID controllers:
	for (auto& pid : PIDs_)
	{
		pid.KP = KP;
		pid.KI = KI;
		pid.KD = KD;
		pid.N = N;
		pid.max_out = max_torque;
		pid.enable_antiwindup = enable_antiwindup;
		pid.enable_reference_filter = enable_reference_filter;
		pid.reference_filter_tau = reference_filter_tau;
		pid.reference_filter_order = reference_filter_order;
	}

	// Optional: filter reference setpoints
	if (enable_reference_filter)
	{
		spVelL = PIDs_[0].filterReference(spVelL, ci.context.dt);
		spVelR = PIDs_[1].filterReference(spVelR, ci.context.dt);
	}

	// Compute feedforward term for slope compensation:
	double ff = 0;
	if (enable_feedforward)
	{
		// nDrivenWheels=2 for differential drive
		ff = feedforward_gain * veh_.estimateSlopeTorquePerWheel(2);
	}

	// "-" because \tau<0 makes robot moves forwards.
	const double followErrorL = spVelL - actVelL;
	const double followErrorR = spVelR - actVelR;

	const double zero_threshold = 0.001;  // m/s
	const double stop_threshold = 0.05;	 // m/s , wider threshold for stop detection

	const bool setpointIsZero =
		std::abs(spVelL) < zero_threshold && std::abs(spVelR) < zero_threshold;

	if (setpointIsZero && std::abs(actVelL) < stop_threshold && std::abs(actVelR) < stop_threshold)
	{
		// Near-zero velocity with zero setpoint: full stop, reset PIDs
		co.wheel_torque_l = 0;
		co.wheel_torque_r = 0;
		for (auto& pid : PIDs_)
		{
			pid.reset();
		}
	}
	else
	{
		co.wheel_torque_l = -PIDs_[0].compute(followErrorL, ci.context.dt, ff);
		co.wheel_torque_r = -PIDs_[1].compute(followErrorR, ci.context.dt, ff);

		if (setpointIsZero)
		{
			// Braking toward zero: clamp torque so it can only oppose current
			// motion direction, preventing the PID integral from causing rebound.
			// Sign convention: positive co.wheel_torque = backward motor force.
			if (actVelL > 0)
			{
				co.wheel_torque_l = std::max(0.0, co.wheel_torque_l);
			}
			else if (actVelL < 0)
			{
				co.wheel_torque_l = std::min(0.0, co.wheel_torque_l);
			}

			if (actVelR > 0)
			{
				co.wheel_torque_r = std::max(0.0, co.wheel_torque_r);
			}
			else if (actVelR < 0)
			{
				co.wheel_torque_r = std::min(0.0, co.wheel_torque_r);
			}
		}
	}

	// Log controller stats via CsvLogger (LOGGER_IDX_WHEELS = 1):
	if (auto logger = veh_.getLoggerPtr(VehicleBase::LOGGER_IDX_WHEELS))
	{
		if (logger->isActive())
		{
			logger->updateColumn("pid_sp_vel_l", spVelL);
			logger->updateColumn("pid_sp_vel_r", spVelR);
			logger->updateColumn("pid_act_vel_l", actVelL);
			logger->updateColumn("pid_act_vel_r", actVelR);
			logger->updateColumn("pid_error_l", followErrorL);
			logger->updateColumn("pid_error_r", followErrorR);
			logger->updateColumn("pid_torque_l", co.wheel_torque_l);
			logger->updateColumn("pid_torque_r", co.wheel_torque_r);
			logger->updateColumn("pid_feedforward", ff);
		}
	}
}

void DynamicsDifferential::ControllerTwistPID::load_config(const rapidxml::xml_node<char>& node)
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
			for (auto& pid : PIDs_)
			{
				pid.reset();
			}
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

		if (js.buttons.size() > 7)
		{
			if (js.buttons[5]) joyMaxLinSpeed *= 1.01;
			if (js.buttons[7]) joyMaxLinSpeed /= 1.01;

			if (js.buttons[4]) joyMaxAngSpeed *= 1.01;
			if (js.buttons[6]) joyMaxAngSpeed /= 1.01;

			if (js.buttons[3])	// brake
			{
				setpoint_ = {0, 0, 0};
				for (auto& pid : PIDs_)
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
		"setpoint: lin=%.03f ang=%.03f deg/s\n", setpoint_.vx, 180.0 / M_PI * setpoint_.omega);
}
