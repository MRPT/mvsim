/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleDynamics/VehicleAckermann.h>
#include <mv2dsim/World.h>

#include "xml_utils.h"

#include <mrpt/opengl/COpenGLScene.h>
#include <rapidxml.hpp>
#include <cmath>

using namespace mv2dsim;
using namespace std;

// Ctor:
DynamicsAckermann::DynamicsAckermann(World *parent) :
	VehicleBase(parent, 4 /*num wheels*/),
	m_max_steer_ang(mrpt::utils::DEG2RAD(30))
{
	using namespace mrpt::math;

	m_chassis_mass=500.0;
	m_chassis_z_min=0.20;
	m_chassis_z_max=1.40;
	m_chassis_color= mrpt::utils::TColor(0xe8,0x30,0x00);
	
	// Default shape:
	m_chassis_poly.clear();
	m_chassis_poly.push_back( TPoint2D(-0.8, -1.0) );
	m_chassis_poly.push_back( TPoint2D(-0.8,  1.0) );
	m_chassis_poly.push_back( TPoint2D( 1.5,  0.9) );
	m_chassis_poly.push_back( TPoint2D( 1.8,  0.8) );
	m_chassis_poly.push_back( TPoint2D( 1.8, -0.8) );
	m_chassis_poly.push_back( TPoint2D( 1.5, -0.9) );
	updateMaxRadiusFromPoly();

	m_fixture_chassis = NULL;
	for (int i=0;i<4;i++) m_fixture_wheels[i]=NULL;

	// Default values:
	// rear-left:
	m_wheels_info[WHEEL_RL].x = 0;
	m_wheels_info[WHEEL_RL].y = 0.9;
	// rear-right:
	m_wheels_info[WHEEL_RR].x = 0;
	m_wheels_info[WHEEL_RR].y = -0.9;
	// Front-left:
	m_wheels_info[WHEEL_FL].x = 1.3;
	m_wheels_info[WHEEL_FL].y = 0.9;
	// Front-right:
	m_wheels_info[WHEEL_FR].x = 1.3;
	m_wheels_info[WHEEL_FR].y = -0.9;
}

/** The derived-class part of load_params_from_xml() */
void DynamicsAckermann::dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node)
{
	// <chassis ...> </chassis>
	const rapidxml::xml_node<char> * xml_chassis = xml_node->first_node("chassis");
	if (xml_chassis)
	{
		// Attribs:	
		std::map<std::string,TParamEntry> attribs;
		attribs["mass"] = TParamEntry("%lf", &this->m_chassis_mass);
		attribs["zmin"] = TParamEntry("%lf", &this->m_chassis_z_min );
		attribs["zmax"] = TParamEntry("%lf", &this->m_chassis_z_max );
		attribs["color"] = TParamEntry("%color", &this->m_chassis_color );

		parse_xmlnode_attribs(*xml_chassis, attribs,"[DynamicsAckermann::dynamics_load_params_from_xml]" );

		// Shape node (optional, fallback to default shape if none found)
		const rapidxml::xml_node<char> * xml_shape = xml_chassis->first_node("shape");
		if (xml_shape)
			mv2dsim::parse_xmlnode_shape(*xml_shape, m_chassis_poly, "[DynamicsAckermann::dynamics_load_params_from_xml]");
	}

	//<rl_wheel pos="0  1" mass="6.0" width="0.30" diameter="0.62" />
	//<rr_wheel pos="0 -1" mass="6.0" width="0.30" diameter="0.62" />
	//<fl_wheel mass="6.0" width="0.30" diameter="0.62" />
	//<fr_wheel mass="6.0" width="0.30" diameter="0.62" />
	const char*w_names[4] = {
		"rl_wheel", // 0
		"rr_wheel", // 1
		"fl_wheel", // 2
		"fr_wheel"  // 3
	};
	// Load common params:
	for (size_t i=0;i<4;i++)
	{
		const rapidxml::xml_node<char> * xml_wheel = xml_node->first_node(w_names[i]);
		if (xml_wheel) m_wheels_info[i].loadFromXML(xml_wheel);		
	}

	//<f_wheels_x>1.3</f_wheels_x>
	//<f_wheels_d>2.0</f_wheels_d>
	// Load front ackermann wheels and other params:
	{
		double front_x = 1.3; 
		double front_d = 2.0;
		std::map<std::string,TParamEntry> ack_ps;
		// Front wheels:
		ack_ps["f_wheels_x"] = TParamEntry("%lf", &front_x);
		ack_ps["f_wheels_d"] = TParamEntry("%lf", &front_d);
		// other params:
		ack_ps["max_steer_ang_deg"] = TParamEntry("%lf_deg", &m_max_steer_ang);

		parse_xmlnode_children_as_param(*xml_node, ack_ps,"[DynamicsAckermann::dynamics_load_params_from_xml]" );

		// Front-left:
		m_wheels_info[WHEEL_FL].x = front_x;
		m_wheels_info[WHEEL_FL].y = 0.5*front_d;
		// Front-right:
		m_wheels_info[WHEEL_FR].x = front_x;
		m_wheels_info[WHEEL_FR].y = -0.5*front_d;
	}

	// Vehicle controller:
	// -------------------------------------------------
	{
		const rapidxml::xml_node<char> * xml_control = xml_node->first_node("controller");
		if (xml_control)
		{
			rapidxml::xml_attribute<char> *control_class = xml_control->first_attribute("class");
			if (!control_class || !control_class->value()) throw runtime_error("[DynamicsAckermann] Missing 'class' attribute in <controller> XML node");

			const std::string sCtrlClass = std::string(control_class->value());
			if (sCtrlClass==ControllerRawForces::class_name())    m_controller = ControllerBasePtr(new ControllerRawForces(*this) );
			else if (sCtrlClass==ControllerFrontSteerPID::class_name())    m_controller = ControllerBasePtr(new ControllerFrontSteerPID(*this) );			
			else throw runtime_error(mrpt::format("[DynamicsAckermann] Unknown 'class'='%s' in <controller> XML node",sCtrlClass.c_str()));

			m_controller->load_config(*xml_control);
		}
	}
	// Default controller: 
	if (!m_controller) m_controller = ControllerBasePtr(new ControllerRawForces(*this) );


}

// See docs in base class:
void DynamicsAckermann::invoke_motor_controllers(const TSimulContext &context, std::vector<double> &out_torque_per_wheel)
{
	// Longitudinal forces at each wheel:
	out_torque_per_wheel.assign(4, 0.0);

	if (m_controller)
	{
		// Invoke controller:
		TControllerInput ci;
		ci.context = context;
		TControllerOutput co;
		m_controller->control_step(ci,co);
		// Take its output:
		out_torque_per_wheel[WHEEL_RL] = co.rl_torque;
		out_torque_per_wheel[WHEEL_RR] = co.rr_torque;
		out_torque_per_wheel[WHEEL_FL] = co.fl_torque;
		out_torque_per_wheel[WHEEL_FR] = co.fr_torque;

		// Kinematically-driven steering wheels: 
		// Ackermann formulas for inner&outer weels turning angles wrt the equivalent (central) one:
		{
			// EQ1: cot(d)+0.5*w/l = cot(do)
			// EQ2: cot(di)=cot(do)-w/l
			const double w = m_wheels_info[WHEEL_FL].y - m_wheels_info[WHEEL_FR].y;
			const double l = m_wheels_info[WHEEL_FL].x - m_wheels_info[WHEEL_RL].x;
			ASSERT_(l>0)
			const double w_l=w/l;
			const double delta= b2Clamp( std::abs(co.steer_ang), 0.0, m_max_steer_ang);			

			const bool delta_neg = (co.steer_ang<0);
			ASSERT_BELOW_(delta,0.5*M_PI-0.01)
			const double cot_do = 1.0/tan(delta)+ 0.5*w_l;
			const double cot_di = cot_do - w_l;
			// delta>0: do->right, di->left wheel
			// delta<0: do->left , di->right wheel
			m_wheels_info[delta_neg ? WHEEL_FR:WHEEL_FL].yaw = atan(1.0/cot_di) * (delta_neg ? -1.0:1.0);
			m_wheels_info[delta_neg ? WHEEL_FL:WHEEL_FR].yaw = atan(1.0/cot_do) * (delta_neg ? -1.0:1.0);
		}

	}

}


// See docs in base class:
vec3 DynamicsAckermann::getVelocityLocalOdoEstimate() const
{
	vec3 odo_vel;
	// Equations:

	// Velocities in local +X at each wheel i={0,1}:
	// v_i = vx - w_veh * wheel_{i,y}  =  w_i * R_i
	// Re-arranging:
	const double w0 = m_wheels_info[WHEEL_RL].getW();
	const double w1 = m_wheels_info[WHEEL_RR].getW();
	const double R0 = m_wheels_info[WHEEL_RL].diameter*0.5;
	const double R1 = m_wheels_info[WHEEL_RR].diameter*0.5;

	const double Ay = m_wheels_info[WHEEL_RL].y-m_wheels_info[WHEEL_RR].y;
	ASSERTMSG_(Ay!=0.0, "The two wheels of a differential vehicle CAN'T by at the same Y coordinate!")

	const double w_veh  = (w1*R1-w0*R0)/Ay;
	const double vx_veh = w0*R0+w_veh*m_wheels_info[WHEEL_RL].y;
	
	odo_vel.vals[0] = vx_veh;
	odo_vel.vals[2] = w_veh;

	// v_y = 0 
	odo_vel.vals[1] = 0.0;
	
#if 0 // Debug
	{
		vec3 gt_vel = this->getVelocityLocal();
		printf("\n gt: vx=%7.03f, vy=%7.03f, w= %7.03fdeg\n", gt_vel.vals[0], gt_vel.vals[1], mrpt::utils::RAD2DEG(gt_vel.vals[2]));
		printf("odo: vx=%7.03f, vy=%7.03f, w= %7.03fdeg\n", odo_vel.vals[0], odo_vel.vals[1], mrpt::utils::RAD2DEG(odo_vel.vals[2]));
	}
#endif
	return odo_vel;
}
