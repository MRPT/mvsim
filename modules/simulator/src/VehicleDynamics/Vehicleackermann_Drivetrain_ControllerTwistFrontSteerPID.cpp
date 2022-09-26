/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

DynamicsAckermannDrivetrain::ControllerTwistFrontSteerPID::
	ControllerTwistFrontSteerPID(DynamicsAckermannDrivetrain& veh)
	: ControllerBase(veh),
	  setpoint_lin_speed(0),
	  setpoint_ang_speed(0),
	  KP(100),
	  KI(0),
	  KD(0),
	  max_torque(400.0)
{
	// Get distance between wheels:
	m_dist_fWheels =
		m_veh.m_wheels_info[WHEEL_FL].y - m_veh.m_wheels_info[WHEEL_FR].y;
	m_r2f_L = m_veh.m_wheels_info[WHEEL_FL].x - m_veh.m_wheels_info[WHEEL_RL].x;

	ASSERT_(m_dist_fWheels > 0.0);
	ASSERT_(m_r2f_L > 0.0);
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
		co.steer_ang = atan(m_r2f_L / R);
	}

	m_PID.KP = KP;
	m_PID.KI = KI;
	m_PID.KD = KD;
	m_PID.max_out = max_torque;

	const double vel_act = m_veh.getVelocityLocalOdoEstimate().vx;
	const double vel_des = setpoint_lin_speed;

	// "-" because \tau<0 makes robot moves forwards.
	co.drive_torque = -m_PID.compute(vel_des - vel_act, ci.context.dt);
}

void DynamicsAckermannDrivetrain::ControllerTwistFrontSteerPID::load_config(
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

void DynamicsAckermannDrivetrain::ControllerTwistFrontSteerPID::
	teleop_interface(const TeleopInput& in, TeleopOutput& out)
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
			break;
	};
	out.append_gui_lines += "[Controller=" + string(class_name()) +
							"] Teleop keys:\n"
							"w/s=incr/decr lin speed.\n"
							"a/d=left/right steering.\n"
							"spacebar=stop.\n";
	out.append_gui_lines += mrpt::format(
		"setpoint: v=%.03f w=%.03f deg/s\n", setpoint_lin_speed,
		setpoint_ang_speed * 180.0 / M_PI);
}
