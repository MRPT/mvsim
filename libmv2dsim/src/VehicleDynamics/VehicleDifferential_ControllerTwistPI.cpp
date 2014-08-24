/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleDynamics/VehicleDifferential.h>
#include "xml_utils.h"

using namespace mv2dsim;
using namespace std;


DynamicsDifferential::ControllerTwistPI::ControllerTwistPI(DynamicsDifferential &veh) : 
	ControllerBase(veh),
	setpoint_lin_speed(0), 
	setpoint_ang_speed(0),
	KP(100),
	KI(0),
	I_MAX(10),
	max_force(20.0)
{
	// Get distance between wheels:
	// Warning: the controller *assumes* that both wheels are parallel (as it's a rule in differential robots!!)
	m_distWheels = m_veh.m_wheels_info[0].y - m_veh.m_wheels_info[1].y;
}

// See base class docs
void DynamicsDifferential::ControllerTwistPI::control_step(
	const DynamicsDifferential::TControllerInput &ci, 
	DynamicsDifferential::TControllerOutput &co)
{
	// For each wheel: 
	// 1) Compute desired velocity set-point (in m/s)
	// 2) Run the PI/PID for that wheel independently (in newtons)
	const double vel_l = setpoint_lin_speed - 0.5* setpoint_ang_speed * m_distWheels;
	const double vel_r = setpoint_lin_speed + 0.5* setpoint_ang_speed * m_distWheels;

	// Compute each wheel actual velocity:
	const vec3 vehVel = m_veh.getVelocityLocal();
	const double act_vel_l = vehVel.vals[0] - 0.5* vehVel.vals[2] * m_distWheels;
	const double act_vel_r = vehVel.vals[0] + 0.5* vehVel.vals[2] * m_distWheels;

	// Apply controller:
	for (int i=0;i<2;i++) {
		m_PID[i].I_MAX_ABS = I_MAX;
		m_PID[i].KP = KP;
		m_PID[i].KI = KI;
		m_PID[i].max_out = max_force;
	}

	co.wheel_force_l = m_PID[0].compute(vel_l-act_vel_l,ci.context.dt);
	co.wheel_force_r = m_PID[1].compute(vel_r-act_vel_r,ci.context.dt);
}

void DynamicsDifferential::ControllerTwistPI::load_config(const rapidxml::xml_node<char>&node )
{
	std::map<std::string,TParamEntry> params;
	params["KP"] = TParamEntry("%lf", &KP);
	params["KI"] = TParamEntry("%lf", &KI);
	params["I_MAX"] = TParamEntry("%lf", &I_MAX);
	params["max_force"] = TParamEntry("%lf", &max_force);



	// Initial speed.
	params["V"] = TParamEntry("%lf", &this->setpoint_lin_speed);
	params["W"] = TParamEntry("%lf_deg", &this->setpoint_ang_speed);

	parse_xmlnode_children_as_param(node,params);
}

