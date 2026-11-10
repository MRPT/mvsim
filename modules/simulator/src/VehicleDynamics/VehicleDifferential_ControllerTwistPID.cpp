/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2025-2026 Fernando Cañadas Aránega                        |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>

#include <algorithm>
#include <limits>

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
	const double spTrack = distWheels_ * wheel_separation_multiplier;
	const double zero_threshold = 0.001;  // m/s
	const bool commandIsZero =
		std::abs(sp.vx) < zero_threshold && std::abs(0.5 * sp.omega * spTrack) < zero_threshold;

	// Compute each wheel actual velocity (from the wheel speeds, not
	// ground-truth!):
	const mrpt::math::TTwist2D vehVelOdo = veh_.getVelocityLocalFromWheels();

	// Turn slip compensation: the wheels must turn faster by the ratio between
	// the yaw rate implied by their speeds (in setpoint units, i.e. before the
	// separation multiplier) and the actual one. The ratio is only measured
	// while turning steadily (the chassis lags behind the wheels while its yaw
	// rate changes), and kept otherwise:
	double spOmega = sp.omega;
	const double yawRate = veh_.getRefVelocityLocal().omega;
	if (slip_compensation_tau > 0)
	{
		const double dt = ci.context.dt;
		const double minYawRate = 0.005;  // [rad/s]
		const double maxRelativeYawAccel = 2.0;	 // [1/s]
		const double minYawAccelLimit = 0.1;  // [rad/s^2]
		const double wheelsYawRate = vehVelOdo.omega / wheel_separation_multiplier;
		const double yawAccel = dt > 0 ? (yawRate - lastYawRate_) / dt : 0;
		if (!commandIsZero && std::abs(yawRate) > minYawRate &&
			std::abs(wheelsYawRate) > minYawRate && wheelsYawRate * yawRate > 0 &&
			std::abs(yawAccel) < maxRelativeYawAccel * std::abs(yawRate) + minYawAccelLimit)
		{
			const double ratio = std::clamp(
				wheelsYawRate / yawRate, 1.0 / slip_compensation_max, slip_compensation_max);
			turnSlipRatio_ += (ratio - turnSlipRatio_) * dt / (slip_compensation_tau + dt);
		}
		spOmega *= turnSlipRatio_;
	}
	lastYawRate_ = yawRate;

	double spVelL = sp.vx - 0.5 * spOmega * spTrack;
	double spVelR = sp.vx + 0.5 * spOmega * spTrack;

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
		// Each side torque drives all the wheels on that side (a caster is not
		// driven), so the slope load is shared among them:
		const size_t nDrivenWheels = veh_.getNumWheels() == 4 ? 4 : 2;
		ff = feedforward_gain * veh_.estimateSlopeTorquePerWheel(nDrivenWheels);
	}

	// "-" because \tau<0 makes robot moves forwards.
	const double followErrorL = spVelL - actVelL;
	const double followErrorR = spVelR - actVelR;

	const bool setpointIsZero =
		std::abs(spVelL) < zero_threshold && std::abs(spVelR) < zero_threshold;

	double sensL = 0;  // d(torque)/d(velocity error)
	double sensR = 0;
	if (setpointIsZero)
	{
		// (max_torque<=0 means no limit, as in PID_Controller)
		const double maxT = max_torque > 0 ? max_torque : std::numeric_limits<double>::infinity();
		// Braking toward a full stop: proportional action only, which always
		// opposes the current motion and vanishes at zero speed, so the vehicle
		// neither coasts nor rebounds due to the integral term.
		for (auto& pid : PIDs_)
		{
			pid.reset();
		}
		co.wheel_torque_l = std::clamp(KP * actVelL - ff, -maxT, maxT);
		co.wheel_torque_r = std::clamp(KP * actVelR - ff, -maxT, maxT);
		co.motor_l.unsaturated = ff - KP * actVelL;
		co.motor_r.unsaturated = ff - KP * actVelR;
		sensL = KP;
		sensR = KP;
	}
	else
	{
		co.wheel_torque_l = -PIDs_[0].compute(followErrorL, ci.context.dt, ff);
		co.wheel_torque_r = -PIDs_[1].compute(followErrorR, ci.context.dt, ff);
		co.motor_l.unsaturated = PIDs_[0].lastUnsaturatedOutput();
		co.motor_r.unsaturated = PIDs_[1].lastUnsaturatedOutput();
		sensL = PIDs_[0].lastOutputSensitivity();
		sensR = PIDs_[1].lastOutputSensitivity();
	}

	// Motor models, for the implicit integration of the wheel spin. Their
	// reaction to each wheel own spin speed is d(actVel)/d(w) (from the
	// odometry equations) times the controller sensitivity:
	{
		const auto& wL = veh_.wheels_info_[WHEEL_L];
		const auto& wR = veh_.wheels_info_[WHEEL_R];
		const double yMeanRel = 0.5 * (wL.y + wR.y) / distWheels_;
		co.motor_l.damping = sensL * 0.5 * wL.diameter * (1.0 - yMeanRel);
		co.motor_r.damping = sensR * 0.5 * wR.diameter * (1.0 + yMeanRel);
		co.motor_l.limit = max_torque;
		co.motor_r.limit = max_torque;
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
	params["wheel_separation_multiplier"] = TParamEntry("%lf", &wheel_separation_multiplier);
	params["slip_compensation_tau"] = TParamEntry("%lf", &slip_compensation_tau);
	params["slip_compensation_max"] = TParamEntry("%lf", &slip_compensation_max);
	params["enable_antiwindup"] = TParamEntry("%bool", &enable_antiwindup);
	params["enable_feedforward"] = TParamEntry("%bool", &enable_feedforward);
	params["feedforward_gain"] = TParamEntry("%lf", &feedforward_gain);
	params["enable_reference_filter"] = TParamEntry("%bool", &enable_reference_filter);
	params["reference_filter_tau"] = TParamEntry("%lf", &reference_filter_tau);
	params["reference_filter_order"] = TParamEntry("%d", &reference_filter_order);

	params["joyMaxLinSpeed"] = TParamEntry("%lf", &joyMaxLinSpeed);
	params["joyMaxAngSpeed"] = TParamEntry("%lf", &joyMaxAngSpeed);

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
