/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/World.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

// Ctor:
DynamicsDifferential::DynamicsDifferential(World* parent)
	: VehicleBase(parent, 3 /*num wheels*/)
{
	using namespace mrpt::math;

	m_chassis_mass = 15.0;
	m_chassis_z_min = 0.05;
	m_chassis_z_max = 0.6;
	m_chassis_color = mrpt::img::TColor(0xff, 0x00, 0x00);

	// Default shape:
	m_chassis_poly.clear();
	m_chassis_poly.emplace_back(-0.4, -0.5);
	m_chassis_poly.emplace_back(-0.4, 0.5);
	m_chassis_poly.emplace_back(0.4, 0.5);
	m_chassis_poly.emplace_back(0.6, 0.3);
	m_chassis_poly.emplace_back(0.6, -0.3);
	m_chassis_poly.emplace_back(0.4, -0.5);
	updateMaxRadiusFromPoly();

	m_fixture_chassis = nullptr;
	for (int i = 0; i < 2; i++) m_fixture_wheels[i] = nullptr;
}

/** The derived-class part of load_params_from_xml() */
void DynamicsDifferential::dynamics_load_params_from_xml(
	const rapidxml::xml_node<char>* xml_node)
{
	const std::map<std::string, std::string> varValues = {{"NAME", m_name}};

	// <chassis ...> </chassis>
	const rapidxml::xml_node<char>* xml_chassis =
		xml_node->first_node("chassis");
	if (xml_chassis)
	{
		// Attribs:
		TParameterDefinitions attribs;
		attribs["mass"] = TParamEntry("%lf", &this->m_chassis_mass);
		attribs["zmin"] = TParamEntry("%lf", &this->m_chassis_z_min);
		attribs["zmax"] = TParamEntry("%lf", &this->m_chassis_z_max);
		attribs["color"] = TParamEntry("%color", &this->m_chassis_color);

		parse_xmlnode_attribs(
			*xml_chassis, attribs, varValues,
			"[DynamicsDifferential::dynamics_load_params_from_xml]");

		// Shape node (optional, fallback to default shape if none found)
		const rapidxml::xml_node<char>* xml_shape =
			xml_chassis->first_node("shape");
		if (xml_shape)
			mvsim::parse_xmlnode_shape(
				*xml_shape, m_chassis_poly,
				"[DynamicsDifferential::dynamics_load_params_from_xml]");
	}

	// <l_wheel ...>, <r_wheel ...>
	const char* w_names[3] = {"l_wheel", "r_wheel", "caster_wheel"};
	const double w_default_x[3] = {0, 0, 0.5};
	const double w_default_y[3] = {0.5, -0.5};
	m_wheels_info.clear();
	m_wheels_info.resize(3);  // reset default values

	// Load common params:
	for (size_t i = 0; i < 3; i++)
	{
		const rapidxml::xml_node<char>* xml_wheel =
			xml_node->first_node(w_names[i]);
		if (xml_wheel)
			m_wheels_info[i].loadFromXML(xml_wheel);
		else
		{
			m_wheels_info[i].x = w_default_x[i];
			m_wheels_info[i].y = w_default_y[i];
		}
	}

	// Vehicle controller:
	// -------------------------------------------------
	{
		const rapidxml::xml_node<char>* xml_control =
			xml_node->first_node("controller");
		if (xml_control)
		{
			rapidxml::xml_attribute<char>* control_class =
				xml_control->first_attribute("class");
			if (!control_class || !control_class->value())
				throw runtime_error(
					"[DynamicsDifferential] Missing 'class' attribute in "
					"<controller> XML node");

			const std::string sCtrlClass = std::string(control_class->value());
			if (sCtrlClass == ControllerRawForces::class_name())
				m_controller =
					ControllerBasePtr(new ControllerRawForces(*this));
			else if (sCtrlClass == ControllerTwistPID::class_name())
				m_controller = ControllerBasePtr(new ControllerTwistPID(*this));
			else
				throw runtime_error(mrpt::format(
					"[DynamicsDifferential] Unknown 'class'='%s' in "
					"<controller> XML node",
					sCtrlClass.c_str()));

			m_controller->load_config(*xml_control);
		}
	}

	// Default controller:
	if (!m_controller)
		m_controller = ControllerBasePtr(new ControllerRawForces(*this));
}

// See docs in base class:
void DynamicsDifferential::invoke_motor_controllers(
	const TSimulContext& context, std::vector<double>& out_torque_per_wheel)
{
	// Longitudinal forces at each wheel:
	out_torque_per_wheel.assign(2 /*active wheels*/ + 1 /* caster wheel*/, 0.0);

	if (m_controller)
	{
		// Invoke controller:
		TControllerInput ci;
		ci.context = context;
		TControllerOutput co;
		m_controller->control_step(ci, co);
		// Take its output:
		out_torque_per_wheel[WHEEL_L] = co.wheel_torque_l;
		out_torque_per_wheel[WHEEL_R] = co.wheel_torque_r;
	}
}

// See docs in base class:
mrpt::math::TTwist2D DynamicsDifferential::getVelocityLocalOdoEstimate() const
{
	mrpt::math::TTwist2D odo_vel;
	// Equations:

	// Velocities in local +X at each wheel i={0,1}:
	// v_i = vx - w_veh * wheel_{i,y}  =  w_i * R_i
	// Re-arranging:
	const double w0 = m_wheels_info[WHEEL_L].getW();
	const double w1 = m_wheels_info[WHEEL_R].getW();
	const double R0 = m_wheels_info[WHEEL_L].diameter * 0.5;
	const double R1 = m_wheels_info[WHEEL_R].diameter * 0.5;

	const double Ay = m_wheels_info[WHEEL_L].y - m_wheels_info[WHEEL_R].y;
	ASSERTMSG_(
		Ay != 0.0,
		"The two wheels of a differential vehicle CAN'T by at the same Y "
		"coordinate!");

	const double w_veh = (w1 * R1 - w0 * R0) / Ay;
	const double vx_veh = w0 * R0 + w_veh * m_wheels_info[WHEEL_L].y;

	odo_vel.vx = vx_veh;
	odo_vel.vy = 0.0;
	odo_vel.omega = w_veh;

	return odo_vel;
}
