/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
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
DynamicsDifferential::DynamicsDifferential(
	World* parent, const std::vector<ConfigPerWheel>& cfgPerWheel)
	: VehicleBase(parent, cfgPerWheel.size() /*num wheels*/),
	  configPerWheel_(cfgPerWheel)
{
	using namespace mrpt::math;

	chassis_mass_ = 15.0;
	chassis_z_min_ = 0.05;
	chassis_z_max_ = 0.6;
	chassis_color_ = mrpt::img::TColor(0xff, 0x00, 0x00);

	// Default shape:
	chassis_poly_.clear();
	chassis_poly_.emplace_back(-0.4, -0.5);
	chassis_poly_.emplace_back(-0.4, 0.5);
	chassis_poly_.emplace_back(0.4, 0.5);
	chassis_poly_.emplace_back(0.6, 0.3);
	chassis_poly_.emplace_back(0.6, -0.3);
	chassis_poly_.emplace_back(0.4, -0.5);
	updateMaxRadiusFromPoly();

	fixture_chassis_ = nullptr;
	for (auto& fw : fixture_wheels_) fw = nullptr;
}

/** The derived-class part of load_params_from_xml() */
void DynamicsDifferential::dynamics_load_params_from_xml(
	const rapidxml::xml_node<char>* xml_node)
{
	const std::map<std::string, std::string> varValues = {{"NAME", name_}};

	// <chassis ...> </chassis>
	const rapidxml::xml_node<char>* xml_chassis =
		xml_node->first_node("chassis");
	if (xml_chassis)
	{
		// Attribs:
		TParameterDefinitions attribs;
		attribs["mass"] = TParamEntry("%lf", &this->chassis_mass_);
		attribs["zmin"] = TParamEntry("%lf", &this->chassis_z_min_);
		attribs["zmax"] = TParamEntry("%lf", &this->chassis_z_max_);
		attribs["color"] = TParamEntry("%color", &this->chassis_color_);

		parse_xmlnode_attribs(
			*xml_chassis, attribs, varValues,
			"[DynamicsDifferential::dynamics_load_params_from_xml]");

		// Shape node (optional, fallback to default shape if none found)
		const rapidxml::xml_node<char>* xml_shape =
			xml_chassis->first_node("shape");
		if (xml_shape)
			mvsim::parse_xmlnode_shape(
				*xml_shape, chassis_poly_,
				"[DynamicsDifferential::dynamics_load_params_from_xml]");
	}

	// <l_wheel ...>, <r_wheel ...>

	// reset default values
	ASSERT_EQUAL_(getNumWheels(), configPerWheel_.size());

	// Load common params:
	for (size_t i = 0; i < getNumWheels(); i++)
	{
		const auto& cpw = configPerWheel_.at(i);

		const rapidxml::xml_node<char>* xml_wheel =
			xml_node->first_node(cpw.name.c_str());
		if (xml_wheel)
			wheels_info_[i].loadFromXML(xml_wheel);
		else
		{
			wheels_info_[i].x = cpw.pos.x;
			wheels_info_[i].y = cpw.pos.y;
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
				controller_ = std::make_shared<ControllerRawForces>(*this);
			else if (sCtrlClass == ControllerTwistPID::class_name())
				controller_ = std::make_shared<ControllerTwistPID>(*this);
			else
				THROW_EXCEPTION_FMT(
					"[DynamicsDifferential] Unknown 'class'='%s' in "
					"<controller> XML node",
					sCtrlClass.c_str());

			controller_->load_config(*xml_control);
		}
	}

	// Default controller:
	if (!controller_)
		controller_ = std::make_shared<ControllerRawForces>(*this);
}

// See docs in base class:
void DynamicsDifferential::invoke_motor_controllers(
	const TSimulContext& context, std::vector<double>& out_torque_per_wheel)
{
	// Longitudinal forces at each wheel:
	auto& otpw = out_torque_per_wheel;

	otpw.assign(getNumWheels(), 0.0);

	if (controller_)
	{
		// Invoke controller:
		TControllerInput ci;
		ci.context = context;
		TControllerOutput co;
		controller_->control_step(ci, co);

		// Take its output:
		switch (getNumWheels())
		{
			case 2:
				otpw[WHEEL_L] = co.wheel_torque_l;
				otpw[WHEEL_R] = co.wheel_torque_r;
				break;
			case 3:
				otpw[WHEEL_L] = co.wheel_torque_l;
				otpw[WHEEL_R] = co.wheel_torque_r;
				otpw[WHEEL_CASTER_FRONT] = 0;
				break;
			case 4:
				otpw[WHEEL_LR] = co.wheel_torque_l;
				otpw[WHEEL_RR] = co.wheel_torque_r;
				otpw[WHEEL_LF] = co.wheel_torque_l;
				otpw[WHEEL_RF] = co.wheel_torque_r;
				break;
			default:
				THROW_EXCEPTION("Unexpected number of wheels!");
		};
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
	const double w0 = wheels_info_[WHEEL_L].getW();
	const double w1 = wheels_info_[WHEEL_R].getW();
	const double R0 = wheels_info_[WHEEL_L].diameter * 0.5;
	const double R1 = wheels_info_[WHEEL_R].diameter * 0.5;

	const double Ay = wheels_info_[WHEEL_L].y - wheels_info_[WHEEL_R].y;
	ASSERTMSG_(
		Ay != 0.0,
		"The two wheels of a differential vehicle CAN'T by at the same Y "
		"coordinate!");

	const double w_veh = (w1 * R1 - w0 * R0) / Ay;
	const double vx_veh = w0 * R0 + w_veh * wheels_info_[WHEEL_L].y;

	odo_vel.vx = vx_veh;
	odo_vel.vy = 0.0;
	odo_vel.omega = w_veh;

	return odo_vel;
}
