/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleDynamics/VehicleDifferential.h>
//#include <mv2dsim/World.h>
//#include <rapidxml.hpp>

using namespace mv2dsim;
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
	switch (in.keycode)
	{
	case 'w':  setpoint_wheel_torque_l -= 0.5; setpoint_wheel_torque_r -= 0.5; break;
	case 's':  setpoint_wheel_torque_l += 0.5; setpoint_wheel_torque_r += 0.5; break;
	case 'a':  setpoint_wheel_torque_l += 0.5; setpoint_wheel_torque_r -= 0.5; break;
	case 'd':  setpoint_wheel_torque_l -= 0.5; setpoint_wheel_torque_r += 0.5; break;
	case ' ': setpoint_wheel_torque_l = setpoint_wheel_torque_r = 0.0; break;
	};
	out.append_gui_lines+="[Controller="+ string(class_name()) +"] Teleop keys: w/s=incr/decr both torques. a/d=left/right. spacebar=stop.\n";
	out.append_gui_lines+=mrpt::format("setpoint: tl=%.03f tr=%.03f deg\n", setpoint_wheel_torque_l, setpoint_wheel_torque_r);
}

