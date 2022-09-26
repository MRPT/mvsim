/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleDifferential.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

DynamicsDifferential::ControllerTwistPID::ControllerTwistPID(
	DynamicsDifferential& veh)
	: ControllerBase(veh)
{
	// Get distance between wheels:
	// Warning: the controller *assumes* that both wheels are parallel (as it's
	// a rule in differential robots!!)
	m_distWheels = m_veh.m_wheels_info[0].y - m_veh.m_wheels_info[1].y;
	ASSERT_(m_distWheels > 0);
}

// See base class docs
void DynamicsDifferential::ControllerTwistPID::control_step(
	const DynamicsDifferential::TControllerInput& ci,
	DynamicsDifferential::TControllerOutput& co)
{
	// For each wheel:
	// 1) Compute desired velocity set-point (in m/s)
	// 2) Run the PI/PID for that wheel independently (in newtons)
	const double spVelL =
		setpoint_lin_speed - 0.5 * setpoint_ang_speed * m_distWheels;
	const double spVelR =
		setpoint_lin_speed + 0.5 * setpoint_ang_speed * m_distWheels;

	// Compute each wheel actual velocity (from an "odometry" estimation of
	// velocity, not ground-truth!):
	const mrpt::math::TTwist2D vehVelOdo = m_veh.getVelocityLocalOdoEstimate();
	const double actVelL = vehVelOdo.vx - 0.5 * vehVelOdo.omega * m_distWheels;
	const double actVelR = vehVelOdo.vx + 0.5 * vehVelOdo.omega * m_distWheels;

	// Apply controller:
	for (auto& pid : m_PIDs)
	{
		pid.KP = KP;
		pid.KI = KI;
		pid.KD = KD;
		pid.max_out = max_torque;
	}

	// "-" because \tau<0 makes robot moves forwards.
	const double followErrorL = spVelL - actVelL;
	const double followErrorR = spVelR - actVelR;

	const double zeroThres = 0.05;	// m/s

	if (std::abs(spVelL) < zeroThres &&	 //
		std::abs(spVelR) < zeroThres &&	 //
		std::abs(spVelR) < zeroThres &&	 //
		std::abs(spVelR) < zeroThres)
	{
		co.wheel_torque_l = 0;
		co.wheel_torque_r = 0;
		for (auto& pid : m_PIDs) pid.reset();
	}
	else
	{
		co.wheel_torque_l = -m_PIDs[0].compute(followErrorL, ci.context.dt);
		co.wheel_torque_r = -m_PIDs[1].compute(followErrorR, ci.context.dt);
	}
}

void DynamicsDifferential::ControllerTwistPID::load_config(
	const rapidxml::xml_node<char>& node)
{
	TParameterDefinitions params;
	params["KP"] = TParamEntry("%lf", &KP);
	params["KI"] = TParamEntry("%lf", &KI);
	params["KD"] = TParamEntry("%lf", &KD);
	params["max_torque"] = TParamEntry("%lf", &max_torque);

	// Initial speed.
	params["V"] = TParamEntry("%lf", &this->setpoint_lin_speed);
	params["W"] = TParamEntry("%lf_deg", &this->setpoint_ang_speed);

	parse_xmlnode_children_as_param(node, params);
}

void DynamicsDifferential::ControllerTwistPID::teleop_interface(
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
			setpoint_ang_speed += 2.0 * M_PI / 180;
			break;

		case 'D':
		case 'd':
			setpoint_ang_speed -= 2.0 * M_PI / 180;
			break;

		case ' ':
		{
			setpoint_lin_speed = 0.0;
			setpoint_ang_speed = 0.0;
			for (auto& pid : m_PIDs) pid.reset();
		}
		break;
	};
	out.append_gui_lines += "[Controller=" + string(class_name()) +
							"] Teleop keys:\n"
							"w/s=forward/backward.\n"
							"a/d=left/right.\n"
							"spacebar=stop.\n";
	out.append_gui_lines += mrpt::format(
		"setpoint: lin=%.03f ang=%.03f deg/s\n", setpoint_lin_speed,
		180.0 / M_PI * setpoint_ang_speed);
}
