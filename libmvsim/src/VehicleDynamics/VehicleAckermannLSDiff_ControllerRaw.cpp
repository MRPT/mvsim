/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mvsim/VehicleDynamics/VehicleAckermannLSDiff.h>
#include "xml_utils.h"

using namespace mvsim;
using namespace std;


DynamicsAckermannLSDiff::ControllerRawForces::ControllerRawForces(DynamicsAckermannLSDiff &veh) :
  ControllerBase(veh),
  setpoint_wheel_torque(0),
	setpoint_steer_ang(0)
{
  MRPT_TODO("I see this file!")
}

// See base class docs
void DynamicsAckermannLSDiff::ControllerRawForces::control_step(
  const DynamicsAckermannLSDiff::TControllerInput &ci,
  DynamicsAckermannLSDiff::TControllerOutput &co)
{
  co.drive_torque = this->setpoint_wheel_torque;
	co.steer_ang = this->setpoint_steer_ang;
}

void DynamicsAckermannLSDiff::ControllerRawForces::load_config(const rapidxml::xml_node<char>&node )
{
	std::map<std::string,TParamEntry> params;
  params["torque"] = TParamEntry("%lf", &setpoint_wheel_torque);

	// Initial speed.
  params["steer_ang_deg"] = TParamEntry("%lf_deg", &this->setpoint_steer_ang);

	parse_xmlnode_children_as_param(node,params);
}

void DynamicsAckermannLSDiff::ControllerRawForces::teleop_interface(const TeleopInput &in, TeleopOutput &out)
{
  ControllerBase::teleop_interface(in, out);

	switch (in.keycode)
	{
  case 'W':
  case 'w':  setpoint_wheel_torque-= 1.0; break;

  case 'S':
  case 's':  setpoint_wheel_torque+= 1.0; break;

  case 'A':
	case 'a':  setpoint_steer_ang += 1.0*M_PI/180.0; mrpt::utils::keep_min(setpoint_steer_ang, m_veh.getMaxSteeringAngle()); break;

  case 'D':
	case 'd':  setpoint_steer_ang -= 1.0*M_PI/180.0; mrpt::utils::keep_max(setpoint_steer_ang, -m_veh.getMaxSteeringAngle()); break;

  case ' ':  setpoint_wheel_torque= .0;break;
	};

	out.append_gui_lines+="[Controller="+ string(class_name()) +"] Teleop keys: w/s=incr/decr torques. a/d=left/right steering. spacebar=stop.\n";
  out.append_gui_lines+=mrpt::format("setpoint: t=%.03f steer=%.03f deg\n", setpoint_wheel_torque, setpoint_steer_ang*180.0/M_PI);
}
