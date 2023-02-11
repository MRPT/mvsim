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
	dist_fWheels_ =
		veh_.wheels_info_[WHEEL_FL].y - veh_.wheels_info_[WHEEL_FR].y;
	r2f_L_ = veh_.wheels_info_[WHEEL_FL].x - veh_.wheels_info_[WHEEL_RL].x;
	ASSERT_(dist_fWheels_ > 0.0);
	ASSERT_(r2f_L_ > 0.0);
}

// See base class docs
void DynamicsAckermann::ControllerTwistFrontSteerPID::control_step(
	const DynamicsAckermann::TControllerInput& ci,
	DynamicsAckermann::TControllerOutput& co)
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
	const std::vector<mrpt::math::TVector2D> desired_wheel_vels =
		veh_.getWheelsVelocityLocal(
			mrpt::math::TTwist2D(setpoint_lin_speed, 0.0, setpoint_ang_speed));

	ASSERT_(desired_wheel_vels.size() == 4);

	// Rotate to obtain the actual desired longitudinal velocity for each wheel:
	// FL:
	double vel_fl, vel_fr;
	double desired_fl_steer_ang, desired_fr_steer_ang;
	veh_.computeFrontWheelAngles(
		co.steer_ang, desired_fl_steer_ang, desired_fr_steer_ang);
	{
		const mrpt::poses::CPose2D wRotInv(0, 0, -desired_fl_steer_ang);
		mrpt::math::TPoint2D vel_w;
		wRotInv.composePoint(
			desired_wheel_vels[DynamicsAckermann::WHEEL_FL], vel_w);
		vel_fl = vel_w.x;
	}
	{
		const mrpt::poses::CPose2D wRotInv(0, 0, -desired_fr_steer_ang);
		mrpt::math::TPoint2D vel_w;
		wRotInv.composePoint(
			desired_wheel_vels[DynamicsAckermann::WHEEL_FR], vel_w);
		vel_fr = vel_w.x;
	}

	// Compute each wheel actual velocity (from an "odometry" estimation of
	// velocity, not ground-truth!):
	double act_vel_fl, act_vel_fr;
	{
		// In local vehicle frame:
		const std::vector<mrpt::math::TVector2D> odo_wheel_vels =
			veh_.getWheelsVelocityLocal(veh_.getVelocityLocalOdoEstimate());
		ASSERT_(odo_wheel_vels.size() == 4);

		const double actual_fl_steer_ang =
			veh_.getWheelInfo(DynamicsAckermann::WHEEL_FL).yaw;
		const double actual_fr_steer_ang =
			veh_.getWheelInfo(DynamicsAckermann::WHEEL_FR).yaw;

		{
			const mrpt::poses::CPose2D wRotInv(0, 0, -actual_fl_steer_ang);
			mrpt::math::TPoint2D vel_w;
			wRotInv.composePoint(
				odo_wheel_vels[DynamicsAckermann::WHEEL_FL], vel_w);
			act_vel_fl = vel_w.x;
		}
		{
			const mrpt::poses::CPose2D wRotInv(0, 0, -actual_fr_steer_ang);
			mrpt::math::TPoint2D vel_w;
			wRotInv.composePoint(
				odo_wheel_vels[DynamicsAckermann::WHEEL_FR], vel_w);
			act_vel_fr = vel_w.x;
		}
	}

	// Apply controller:
	for (int i = 0; i < 2; i++)
	{
		PID_[i].KP = KP;
		PID_[i].KI = KI;
		PID_[i].KD = KD;
		PID_[i].max_out = max_torque;
	}

	co.rl_torque = .0;
	co.rr_torque = .0;
	co.fl_torque = -PID_[0].compute(
		vel_fl - act_vel_fl,
		ci.context.dt);	 // "-" because \tau<0 makes robot moves forwards.
	co.fr_torque = -PID_[1].compute(vel_fr - act_vel_fr, ci.context.dt);
}

void DynamicsAckermann::ControllerTwistFrontSteerPID::load_config(
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
			break;
	};
	out.append_gui_lines += "[Controller=" + string(class_name()) +
							"] Teleop keys: w/s=incr/decr lin speed. "
							"a/d=left/right steering. spacebar=stop.\n";
	out.append_gui_lines += mrpt::format(
		"setpoint: v=%.03f w=%.03f deg/s\n", setpoint_lin_speed,
		setpoint_ang_speed * 180.0 / M_PI);
}
