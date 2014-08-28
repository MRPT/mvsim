/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleDynamics/VehicleAckermann.h>
#include "xml_utils.h"

using namespace mv2dsim;
using namespace std;


DynamicsAckermann::ControllerRawForces::ControllerRawForces(DynamicsAckermann &veh) : 
	ControllerBase(veh),
	setpoint_wheel_torque_l(0), 
	setpoint_wheel_torque_r(0),
	setpoint_steer_ang(0)
{
}

// See base class docs
void DynamicsAckermann::ControllerRawForces::control_step(
	const DynamicsAckermann::TControllerInput &ci, 
	DynamicsAckermann::TControllerOutput &co)
{
	co.fl_torque = this->setpoint_wheel_torque_l;
	co.fr_torque = this->setpoint_wheel_torque_r;
	co.steer_ang = this->setpoint_steer_ang;
}

void DynamicsAckermann::ControllerRawForces::load_config(const rapidxml::xml_node<char>&node )
{
	std::map<std::string,TParamEntry> params;
	params["fl_torque"] = TParamEntry("%lf", &setpoint_wheel_torque_l);
	params["fr_torque"] = TParamEntry("%lf", &setpoint_wheel_torque_r);
	
	// Initial speed.
	params["steer_ang_deg"] = TParamEntry("%lf_deg", &this->setpoint_steer_ang);

	parse_xmlnode_children_as_param(node,params);
}

