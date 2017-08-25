/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include "xml_utils.h"

using namespace mvsim;
using namespace std;

DynamicsDifferential::ControllerTwistPID::ControllerTwistPID(
	DynamicsDifferential& veh)
	: ControllerBase(veh),
	  setpoint_lin_speed(0),
	  setpoint_ang_speed(0),
	  KP(100),
	  KI(0),
	  KD(0),
	  max_torque(100.0)
{
	// Get distance between wheels:
	// Warning: the controller *assumes* that both wheels are parallel (as it's
	// a rule in differential robots!!)
	m_distWheels = m_veh.m_wheels_info[0].y - m_veh.m_wheels_info[1].y;
}

// See base class docs
void DynamicsDifferential::ControllerTwistPID::control_step(
	const DynamicsDifferential::TControllerInput& ci,
	DynamicsDifferential::TControllerOutput& co)
{
	// For each wheel:
	// 1) Compute desired velocity set-point (in m/s)
	// 2) Run the PI/PID for that wheel independently (in newtons)
	const double vel_l =
		setpoint_lin_speed - 0.5 * setpoint_ang_speed * m_distWheels;
	const double vel_r =
		setpoint_lin_speed + 0.5 * setpoint_ang_speed * m_distWheels;

	// Compute each wheel actual velocity (from an "odometry" estimation of
	// velocity, not ground-truth!):
	const vec3 vehVelOdo = m_veh.getVelocityLocalOdoEstimate();
	const double act_vel_l =
		vehVelOdo.vals[0] - 0.5 * vehVelOdo.vals[2] * m_distWheels;
	const double act_vel_r =
		vehVelOdo.vals[0] + 0.5 * vehVelOdo.vals[2] * m_distWheels;

	// Apply controller:
	for (int i = 0; i < 2; i++)
	{
		m_PID[i].KP = KP;
		m_PID[i].KI = KI;
		m_PID[i].KD = KD;
		m_PID[i].max_out = max_torque;
	}

	co.wheel_torque_l = -m_PID[0].compute(
		vel_l - act_vel_l,
		ci.context.dt);  // "-" because \tau<0 makes robot moves forwards.
	co.wheel_torque_r = -m_PID[1].compute(vel_r - act_vel_r, ci.context.dt);
}

void DynamicsDifferential::ControllerTwistPID::load_config(
	const rapidxml::xml_node<char>& node)
{
	std::map<std::string, TParamEntry> params;
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
			setpoint_lin_speed = 0.0;
			setpoint_ang_speed = 0.0;
			break;
	};
	out.append_gui_lines +=
		"[Controller=" + string(class_name()) +
		"] Teleop keys: w/s=forward/backward. a/d=left/right. spacebar=stop.\n";
	out.append_gui_lines += mrpt::format(
		"setpoint: lin=%.03f ang=%.03f deg/s\n", setpoint_lin_speed,
		180.0 / M_PI * setpoint_ang_speed);
}
