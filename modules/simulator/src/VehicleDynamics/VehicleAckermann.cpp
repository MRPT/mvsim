/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/World.h>

#include <cmath>
#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

// Ctor:
DynamicsAckermann::DynamicsAckermann(World* parent)
	: VehicleBase(parent, 4 /*num wheels*/)
{
	m_chassis_mass = 500.0;
	m_chassis_z_min = 0.20;
	m_chassis_z_max = 1.40;
	m_chassis_color = mrpt::img::TColor(0xe8, 0x30, 0x00);

	// Default shape:
	m_chassis_poly.clear();
	m_chassis_poly.emplace_back(-0.8, -1.0);
	m_chassis_poly.emplace_back(-0.8, 1.0);
	m_chassis_poly.emplace_back(1.5, 0.9);
	m_chassis_poly.emplace_back(1.8, 0.8);
	m_chassis_poly.emplace_back(1.8, -0.8);
	m_chassis_poly.emplace_back(1.5, -0.9);
	updateMaxRadiusFromPoly();

	m_fixture_chassis = nullptr;
	for (int i = 0; i < 4; i++) m_fixture_wheels[i] = nullptr;

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
void DynamicsAckermann::dynamics_load_params_from_xml(
	const rapidxml::xml_node<char>* xml_node)
{
	const std::map<std::string, std::string> varValues = {{"NAME", m_name}};

	// <chassis ...> </chassis>
	if (const rapidxml::xml_node<char>* xml_chassis =
			xml_node->first_node("chassis");
		xml_chassis)
	{
		// Attribs:
		TParameterDefinitions attribs;
		attribs["mass"] = TParamEntry("%lf", &this->m_chassis_mass);
		attribs["zmin"] = TParamEntry("%lf", &this->m_chassis_z_min);
		attribs["zmax"] = TParamEntry("%lf", &this->m_chassis_z_max);
		attribs["color"] = TParamEntry("%color", &this->m_chassis_color);

		parse_xmlnode_attribs(
			*xml_chassis, attribs, {},
			"[DynamicsAckermann::dynamics_load_params_from_xml]");

		// Shape node (optional, fallback to default shape if none found)
		if (const rapidxml::xml_node<char>* xml_shape =
				xml_chassis->first_node("shape");
			xml_shape)
			mvsim::parse_xmlnode_shape(
				*xml_shape, m_chassis_poly,
				"[DynamicsAckermann::dynamics_load_params_from_xml]");
	}

	//<rl_wheel pos="0  1" mass="6.0" width="0.30" diameter="0.62" />
	//<rr_wheel pos="0 -1" mass="6.0" width="0.30" diameter="0.62" />
	//<fl_wheel mass="6.0" width="0.30" diameter="0.62" />
	//<fr_wheel mass="6.0" width="0.30" diameter="0.62" />
	const char* w_names[4] = {
		"rl_wheel",	 // 0
		"rr_wheel",	 // 1
		"fl_wheel",	 // 2
		"fr_wheel"	// 3
	};
	// Load common params:
	for (size_t i = 0; i < 4; i++)
	{
		if (auto xml_wheel = xml_node->first_node(w_names[i]); xml_wheel)
		{
			m_wheels_info[i].loadFromXML(xml_wheel);
		}
		else
		{
			m_world->logFmt(
				mrpt::system::LVL_WARN,
				"No XML entry '%s' found: using defaults for wheel #%u",
				w_names[i], static_cast<unsigned int>(i));
		}
	}

	//<f_wheels_x>1.3</f_wheels_x>
	//<f_wheels_d>2.0</f_wheels_d>
	// Load front ackermann wheels and other params:
	{
		double front_x = 1.3;
		double front_d = 2.0;
		TParameterDefinitions ack_ps;
		// Front wheels:
		ack_ps["f_wheels_x"] = TParamEntry("%lf", &front_x);
		ack_ps["f_wheels_d"] = TParamEntry("%lf", &front_d);
		// other params:
		ack_ps["max_steer_ang_deg"] = TParamEntry("%lf_deg", &m_max_steer_ang);

		parse_xmlnode_children_as_param(
			*xml_node, ack_ps, varValues,
			"[DynamicsAckermann::dynamics_load_params_from_xml]");

		// Front-left:
		m_wheels_info[WHEEL_FL].x = front_x;
		m_wheels_info[WHEEL_FL].y = 0.5 * front_d;
		// Front-right:
		m_wheels_info[WHEEL_FR].x = front_x;
		m_wheels_info[WHEEL_FR].y = -0.5 * front_d;
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
					"[DynamicsAckermann] Missing 'class' attribute in "
					"<controller> XML node");

			const std::string sCtrlClass = std::string(control_class->value());
			if (sCtrlClass == ControllerRawForces::class_name())
				m_controller = std::make_shared<ControllerRawForces>(*this);
			else if (sCtrlClass == ControllerTwistFrontSteerPID::class_name())
				m_controller =
					std::make_shared<ControllerTwistFrontSteerPID>(*this);
			else if (sCtrlClass == ControllerFrontSteerPID::class_name())
				m_controller = std::make_shared<ControllerFrontSteerPID>(*this);
			else
				THROW_EXCEPTION_FMT(
					"[DynamicsAckermann] Unknown 'class'='%s' in "
					"<controller> XML node",
					sCtrlClass.c_str());

			m_controller->load_config(*xml_control);
		}
	}
	// Default controller:
	if (!m_controller)
		m_controller = std::make_shared<ControllerRawForces>(*this);
}

// See docs in base class:
void DynamicsAckermann::invoke_motor_controllers(
	const TSimulContext& context, std::vector<double>& out_torque_per_wheel)
{
	// Longitudinal forces at each wheel:
	out_torque_per_wheel.assign(4, 0.0);

	if (m_controller)
	{
		// Invoke controller:
		TControllerInput ci;
		ci.context = context;
		TControllerOutput co;
		m_controller->control_step(ci, co);
		// Take its output:
		out_torque_per_wheel[WHEEL_RL] = co.rl_torque;
		out_torque_per_wheel[WHEEL_RR] = co.rr_torque;
		out_torque_per_wheel[WHEEL_FL] = co.fl_torque;
		out_torque_per_wheel[WHEEL_FR] = co.fr_torque;

		// Kinematically-driven steering wheels:
		// Ackermann formulas for inner&outer weels turning angles wrt the
		// equivalent (central) one:
		computeFrontWheelAngles(
			co.steer_ang, m_wheels_info[WHEEL_FL].yaw,
			m_wheels_info[WHEEL_FR].yaw);
	}
}

void DynamicsAckermann::computeFrontWheelAngles(
	const double desired_equiv_steer_ang, double& out_fl_ang,
	double& out_fr_ang) const
{
	// EQ1: cot(d)+0.5*w/l = cot(do)
	// EQ2: cot(di)=cot(do)-w/l
	const double w = m_wheels_info[WHEEL_FL].y - m_wheels_info[WHEEL_FR].y;
	const double l = m_wheels_info[WHEEL_FL].x - m_wheels_info[WHEEL_RL].x;
	ASSERT_(l > 0);
	const double w_l = w / l;
	const double delta =
		b2Clamp(std::abs(desired_equiv_steer_ang), 0.0, m_max_steer_ang);

	const bool delta_neg = (desired_equiv_steer_ang < 0);
	ASSERT_LT_(delta, 0.5 * M_PI - 0.01);
	const double cot_do = 1.0 / tan(delta) + 0.5 * w_l;
	const double cot_di = cot_do - w_l;
	// delta>0: do->right, di->left wheel
	// delta<0: do->left , di->right wheel
	(delta_neg ? out_fr_ang : out_fl_ang) =
		atan(1.0 / cot_di) * (delta_neg ? -1.0 : 1.0);
	(delta_neg ? out_fl_ang : out_fr_ang) =
		atan(1.0 / cot_do) * (delta_neg ? -1.0 : 1.0);
}

// See docs in base class:
mrpt::math::TTwist2D DynamicsAckermann::getVelocityLocalOdoEstimate() const
{
	mrpt::math::TTwist2D odo_vel;
	// Equations:

	// Velocities in local +X at each wheel i={0,1}:
	// v_i = vx - w_veh * wheel_{i,y}  =  w_i * R_i
	// Re-arranging:
	const double w0 = m_wheels_info[WHEEL_RL].getW();
	const double w1 = m_wheels_info[WHEEL_RR].getW();
	const double R0 = m_wheels_info[WHEEL_RL].diameter * 0.5;
	const double R1 = m_wheels_info[WHEEL_RR].diameter * 0.5;

	const double Ay = m_wheels_info[WHEEL_RL].y - m_wheels_info[WHEEL_RR].y;
	ASSERTMSG_(
		Ay != 0.0,
		"The two wheels of a differential vehicle CAN'T by at the same Y "
		"coordinate!");

	const double w_veh = (w1 * R1 - w0 * R0) / Ay;
	const double vx_veh = w0 * R0 + w_veh * m_wheels_info[WHEEL_RL].y;

	odo_vel.vx = vx_veh;
	odo_vel.vy = 0.0;
	odo_vel.omega = w_veh;

#if 0  // Debug
	{
		mrpt::math::TTwist2D gt_vel = this->getVelocityLocal();
		printf("\n gt: vx=%7.03f, vy=%7.03f, w= %7.03fdeg\n", gt_vel.vx, gt_vel.vy, mrpt::RAD2DEG(gt_vel.omega));
		printf("odo: vx=%7.03f, vy=%7.03f, w= %7.03fdeg\n", odo_vel.vx, odo_vel.vy, mrpt::RAD2DEG(odo_vel.omega));
	}
#endif
	return odo_vel;
}
