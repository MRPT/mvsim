/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleDynamics/VehicleDifferential.h>
#include <mv2dsim/World.h>

#include "xml_utils.h"

#include <mrpt/opengl/COpenGLScene.h>
#include <rapidxml.hpp>

using namespace mv2dsim;
using namespace std;

// Ctor:
DynamicsDifferential::DynamicsDifferential(World *parent) :
	VehicleBase(parent, 2 /*num wheels*/)
{
	using namespace mrpt::math;
	
	m_chassis_mass=15.0;
	m_chassis_z_min=0.05;
	m_chassis_z_max=0.6;
	m_chassis_color=mrpt::utils::TColor(0xff,0x00,0x00);

	// Default shape:
	m_chassis_poly.clear();
	m_chassis_poly.push_back( TPoint2D(-0.4, -0.5) );
	m_chassis_poly.push_back( TPoint2D(-0.4,  0.5) );
	m_chassis_poly.push_back( TPoint2D( 0.4,  0.5) );
	m_chassis_poly.push_back( TPoint2D( 0.6,  0.3) );
	m_chassis_poly.push_back( TPoint2D( 0.6, -0.3) );
	m_chassis_poly.push_back( TPoint2D( 0.4, -0.5) );
	updateMaxRadiusFromPoly();

	m_fixture_chassis = NULL;
	for (int i=0;i<2;i++) m_fixture_wheels[i]=NULL;
}

/** The derived-class part of load_params_from_xml() */
void DynamicsDifferential::dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node)
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

		parse_xmlnode_attribs(*xml_chassis, attribs,"[DynamicsDifferential::dynamics_load_params_from_xml]" );

		// Shape node (optional, fallback to default shape if none found)
		const rapidxml::xml_node<char> * xml_shape = xml_chassis->first_node("shape");
		if (xml_shape)
			mv2dsim::parse_xmlnode_shape(*xml_shape, m_chassis_poly, "[DynamicsDifferential::dynamics_load_params_from_xml]");
	}

	// <l_wheel ...>, <r_wheel ...>
	{
		const rapidxml::xml_node<char> * xml_wheel_l = xml_node->first_node("l_wheel");
		if (xml_wheel_l)
			m_wheels_info[0].loadFromXML(xml_wheel_l);
		else
		{
			m_wheels_info[0] = Wheel();
			m_wheels_info[0].y = 0.5;
		}
	}
	{
		const rapidxml::xml_node<char> * xml_wheel_r = xml_node->first_node("r_wheel");
		if (xml_wheel_r)
			m_wheels_info[1].loadFromXML(xml_wheel_r);
		else
		{
			m_wheels_info[1] = Wheel();
			m_wheels_info[1].y = -0.5;
		}
	}

	// Vehicle controller:
	// -------------------------------------------------
	{
		const rapidxml::xml_node<char> * xml_control = xml_node->first_node("controller");
		if (xml_control)
		{
			rapidxml::xml_attribute<char> *control_class = xml_control->first_attribute("class");
			if (!control_class || !control_class->value()) throw runtime_error("[DynamicsDifferential] Missing 'class' attribute in <controller> XML node");

			const std::string sCtrlClass = std::string(control_class->value());
			if (sCtrlClass==ControllerRawForces::class_name())    m_controller = ControllerBasePtr(new ControllerRawForces(*this) );
			else if (sCtrlClass==ControllerTwistPID::class_name()) m_controller = ControllerBasePtr(new ControllerTwistPID(*this) );
			else throw runtime_error(mrpt::format("[DynamicsDifferential] Unknown 'class'='%s' in <controller> XML node",sCtrlClass.c_str()));

			m_controller->load_config(*xml_control);
		}
	}

	// Default controller: 
	if (!m_controller)
		m_controller = ControllerBasePtr(new ControllerRawForces(*this) );


}

// See docs in base class:
void DynamicsDifferential::invoke_motor_controllers(const TSimulContext &context, std::vector<double> &out_torque_per_wheel)
{
	// Longitudinal forces at each wheel:
	out_torque_per_wheel.assign(2, 0.0);

	if (m_controller)
	{
		// Invoke controller:
		TControllerInput ci;
		ci.context = context;
		TControllerOutput co;
		m_controller->control_step(ci,co);
		// Take its output:
		out_torque_per_wheel[0] = co.wheel_torque_l;
		out_torque_per_wheel[1] = co.wheel_torque_r;
	}

}
