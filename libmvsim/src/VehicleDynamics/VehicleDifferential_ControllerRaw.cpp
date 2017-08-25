/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mvsim/VehicleDynamics/VehicleDifferential.h>
//#include <mvsim/World.h>
//#include <rapidxml.hpp>

using namespace mvsim;
using namespace std;

// See base class docs
void DynamicsDifferential::ControllerRawForces::control_step(
	const DynamicsDifferential::TControllerInput &ci,
	DynamicsDifferential::TControllerOutput &co)
{
	co.wheel_torque_l = this->setpoint_wheel_torque_l;
	co.wheel_torque_r = this->setpoint_wheel_torque_r;
}

void DynamicsDifferential::ControllerRawForces::teleop_interface(const TeleopInput &in, TeleopOutput &out)
{
  ControllerBase::teleop_interface(in, out);

	switch (in.keycode)
	{
  case 'W':
	case 'w':  setpoint_wheel_torque_l -= 0.5; setpoint_wheel_torque_r -= 0.5; break;

  case 'S':
	case 's':  setpoint_wheel_torque_l += 0.5; setpoint_wheel_torque_r += 0.5; break;

  case 'A':
	case 'a':  setpoint_wheel_torque_l += 0.5; setpoint_wheel_torque_r -= 0.5; break;

  case 'D':
	case 'd':  setpoint_wheel_torque_l -= 0.5; setpoint_wheel_torque_r += 0.5; break;

  case ' ': setpoint_wheel_torque_l = setpoint_wheel_torque_r = 0.0; break;
	};
	out.append_gui_lines+="[Controller="+ string(class_name()) +"] Teleop keys: w/s=incr/decr both torques. a/d=left/right. spacebar=stop.\n";
	out.append_gui_lines+=mrpt::format("setpoint: tl=%.03f tr=%.03f deg\n", setpoint_wheel_torque_l, setpoint_wheel_torque_r);
}

