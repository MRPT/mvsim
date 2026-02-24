/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleDynamics/VehicleAckermann.h>

using namespace mvsim;

DynamicsAckermann::ControllerTwistIdeal::ControllerTwistIdeal(DynamicsAckermann& veh)
	: ControllerBase(veh)
{
	// Pre-compute the rear-to-front axle distance (wheelbase).
	// This is needed to derive the equivalent Ackermann steering angle from a
	// (vx, omega) twist setpoint:  steer_ang = atan(omega * L / vx)
	r2f_L_ = veh_.wheels_info_[WHEEL_FL].x - veh_.wheels_info_[WHEEL_RL].x;
	ASSERT_(r2f_L_ > 0.0);

	// Signal that friction reaction forces must not be applied to the
	// chassis body , the twist is imposed directly by this controller.
	veh_.idealControllerActive_ = true;
}

void DynamicsAckermann::ControllerTwistIdeal::control_step(
	[[maybe_unused]] const DynamicsAckermann::TControllerInput& ci,
	DynamicsAckermann::TControllerOutput& co)
{
	// Ideal controller: no wheel torques are needed, the twist is imposed
	// directly in on_post_step(). We still need to fill steer_ang so that
	// computeFrontWheelAngles() receives a sensible value.
	co.fl_torque = 0;
	co.fr_torque = 0;
	co.rl_torque = 0;
	co.rr_torque = 0;

	const auto sp = setpoint();

	// Compute the equivalent central Ackermann steering angle from (vx, omega).
	// Kinematic relation:  omega = vx * tan(delta) / L
	//   => delta = atan(omega * L / vx)
	// When vx == 0 we fall back to a direct omega-based clamped angle.
	if (std::abs(sp.vx) > 1e-3)
	{
		co.steer_ang = std::atan(sp.omega * r2f_L_ / sp.vx);
	}
	else
	{
		co.steer_ang = (sp.omega >= 0 ? 1.0 : -1.0) *
					   std::min(std::abs(sp.omega * r2f_L_), veh_.getMaxSteeringAngle());
	}

	// Clamp to vehicle steering limits
	co.steer_ang =
		std::clamp(co.steer_ang, -veh_.getMaxSteeringAngle(), veh_.getMaxSteeringAngle());
}

void DynamicsAckermann::ControllerTwistIdeal::on_post_step(
	[[maybe_unused]] const TSimulContext& context)
{
	// Fake / ideal controller: directly override the vehicle twist with the
	// setpoint. Box2D integration will propagate this to the pose.
	const auto sp = setpoint();
	veh_.setRefVelocityLocal(sp);
}

void DynamicsAckermann::ControllerTwistIdeal::teleop_interface(
	const TeleopInput& in, TeleopOutput& out)
{
	ControllerBase::teleop_interface(in, out);

	auto lck = mrpt::lockHelper(setpointMtx_);

	switch (in.keycode)
	{
		default:
			break;
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
			setpoint_.omega += 2.0 * M_PI / 180.0;
			break;

		case 'D':
		case 'd':
			setpoint_.omega -= 2.0 * M_PI / 180.0;
			break;

		case ' ':
		{
			setpoint_ = {0, 0, 0};
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

			// brake
			if (js.buttons[3])
			{
				setpoint_ = {0, 0, 0};
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
		"setpoint: lin=%.03f ang=%.03f deg/s\n", setpoint_.vx, mrpt::RAD2DEG(setpoint_.omega));
}
