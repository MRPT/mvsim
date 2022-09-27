/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>
#include <mvsim/World.h>

#include <cmath>
#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

// Ctor:
DynamicsAckermannDrivetrain::DynamicsAckermannDrivetrain(World* parent)
	: VehicleBase(parent, 4 /*num wheels*/), m_max_steer_ang(mrpt::DEG2RAD(30))
{
	using namespace mrpt::math;

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

	m_FrontRearSplit = 0.5;
	m_FrontLRSplit = 0.5;
	m_RearLRSplit = 0.5;

	m_FrontRearBias = 1.5;
	m_FrontLRBias = 1.5;
	m_RearLRBias = 1.5;

	m_diff_type = DIFF_TORSEN_4WD;
}

/** The derived-class part of load_params_from_xml() */
void DynamicsAckermannDrivetrain::dynamics_load_params_from_xml(
	const rapidxml::xml_node<char>* xml_node)
{
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
			*xml_chassis, attribs, {},
			"[DynamicsAckermannDrivetrain::dynamics_load_params_from_xml]");

		// Shape node (optional, fallback to default shape if none found)
		const rapidxml::xml_node<char>* xml_shape =
			xml_chassis->first_node("shape");
		if (xml_shape)
			mvsim::parse_xmlnode_shape(
				*xml_shape, m_chassis_poly,
				"[DynamicsAckermannDrivetrain::dynamics_load_params_from_xml]");
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
			*xml_node, ack_ps, {},
			"[DynamicsAckermannDrivetrain::dynamics_load_params_from_xml]");

		// Front-left:
		m_wheels_info[WHEEL_FL].x = front_x;
		m_wheels_info[WHEEL_FL].y = 0.5 * front_d;
		// Front-right:
		m_wheels_info[WHEEL_FR].x = front_x;
		m_wheels_info[WHEEL_FR].y = -0.5 * front_d;
	}

	// Drivetrain parameters
	// Watch the order of DifferentialType enum!
	const char* drive_names[6] = {"open_front",	  "open_rear",	 "open_4wd",
								  "torsen_front", "torsen_rear", "torsen_4wd"};

	const rapidxml::xml_node<char>* xml_drive =
		xml_node->first_node("drivetrain");
	if (xml_drive)
	{
		TParameterDefinitions attribs;
		std::string diff_type;
		attribs["type"] = TParamEntry("%s", &diff_type);

		parse_xmlnode_attribs(
			*xml_drive, attribs, {},
			"[DynamicsAckermannDrivetrain::dynamics_load_params_from_xml]");

		for (size_t i = 0; i < DifferentialType::DIFF_MAX; ++i)
		{
			if (diff_type == drive_names[i])
			{
				m_diff_type = (DifferentialType)i;
				break;
			}
		}

		TParameterDefinitions drive_params;
		drive_params["front_rear_split"] =
			TParamEntry("%lf", &m_FrontRearSplit);
		drive_params["front_rear_bias"] = TParamEntry("%lf", &m_FrontRearBias);
		drive_params["front_left_right_split"] =
			TParamEntry("%lf", &m_FrontLRSplit);
		drive_params["front_left_right_bias"] =
			TParamEntry("%lf", &m_FrontLRBias);
		drive_params["rear_left_right_split"] =
			TParamEntry("%lf", &m_RearLRSplit);
		drive_params["rear_left_right_bias"] =
			TParamEntry("%lf", &m_RearLRBias);

		parse_xmlnode_children_as_param(
			*xml_drive, drive_params, {},
			"[DynamicsAckermannDrivetrain::dynamics_load_params_from_xml]");
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
					"[DynamicsAckermannDrivetrain] Missing 'class' attribute "
					"in <controller> XML node");

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
					"[DynamicsAckermannDrivetrain] Unknown 'class'='%s' in "
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
void DynamicsAckermannDrivetrain::invoke_motor_controllers(
	const TSimulContext& context, std::vector<double>& out_torque_per_wheel)
{
	// Longitudinal forces at each wheel:
	out_torque_per_wheel.assign(4, 0.0);
	double torque_split_per_wheel[4] = {0.0};

	if (m_controller)
	{
		// Invoke controller:
		TControllerInput ci;
		ci.context = context;
		TControllerOutput co;
		m_controller->control_step(ci, co);
		// Take its output:

		switch (m_diff_type)
		{
			case DIFF_OPEN_FRONT:
			{
				torque_split_per_wheel[WHEEL_FL] = m_FrontLRSplit;
				torque_split_per_wheel[WHEEL_FR] = 1. - m_FrontLRSplit;
				torque_split_per_wheel[WHEEL_RL] = 0.0;
				torque_split_per_wheel[WHEEL_RR] = 0.0;
			}
			break;
			case DIFF_OPEN_REAR:
			{
				torque_split_per_wheel[WHEEL_FL] = 0.0;
				torque_split_per_wheel[WHEEL_FR] = 0.0;
				torque_split_per_wheel[WHEEL_RL] = m_RearLRSplit;
				torque_split_per_wheel[WHEEL_RR] = 1. - m_RearLRSplit;
			}
			break;
			case DIFF_OPEN_4WD:
			{
				const double front = m_FrontRearSplit;
				const double rear = 1. - m_FrontRearSplit;

				torque_split_per_wheel[WHEEL_FL] = front * m_FrontLRSplit;
				torque_split_per_wheel[WHEEL_FR] =
					front * (1. - m_FrontLRSplit);
				torque_split_per_wheel[WHEEL_RL] = rear * m_RearLRSplit;
				torque_split_per_wheel[WHEEL_RR] = rear * (1. - m_RearLRSplit);
			}
			break;
			case DIFF_TORSEN_FRONT:
			{
				// no torque to rear
				torque_split_per_wheel[WHEEL_RL] = 0.0;
				torque_split_per_wheel[WHEEL_RR] = 0.0;

				computeDiffTorqueSplit(
					m_wheels_info[WHEEL_FL].getW(),
					m_wheels_info[WHEEL_FR].getW(), m_FrontLRBias,
					m_FrontLRSplit, torque_split_per_wheel[WHEEL_FL],
					torque_split_per_wheel[WHEEL_FR]);
			}
			break;
			case DIFF_TORSEN_REAR:
			{
				// no torque to front
				torque_split_per_wheel[WHEEL_FL] = 0.0;
				torque_split_per_wheel[WHEEL_FR] = 0.0;

				computeDiffTorqueSplit(
					m_wheels_info[WHEEL_RL].getW(),
					m_wheels_info[WHEEL_RR].getW(), m_RearLRBias, m_RearLRSplit,
					torque_split_per_wheel[WHEEL_RL],
					torque_split_per_wheel[WHEEL_RR]);
			}
			break;
			case DIFF_TORSEN_4WD:
			{
				// here we have magic
				const double w_FL = m_wheels_info[WHEEL_FL].getW();
				const double w_FR = m_wheels_info[WHEEL_FR].getW();
				const double w_RL = m_wheels_info[WHEEL_RL].getW();
				const double w_RR = m_wheels_info[WHEEL_RR].getW();
				const double w_F = w_FL + w_FR;
				const double w_R = w_RL + w_RR;

				double t_F = 0.0;
				double t_R = 0.0;
				// front-rear
				computeDiffTorqueSplit(
					w_F, w_R, m_FrontRearBias, m_FrontRearSplit, t_F, t_R);

				double t_FL = 0.0;
				double t_FR = 0.0;
				// front left-right
				computeDiffTorqueSplit(
					w_FL, w_FR, m_FrontLRBias, m_FrontLRSplit, t_FL, t_FR);

				double t_RL = 0.0;
				double t_RR = 0.0;
				// rear left-right
				computeDiffTorqueSplit(
					w_RL, w_RR, m_RearLRBias, m_RearLRSplit, t_RL, t_RR);

				torque_split_per_wheel[WHEEL_FL] = t_F * t_FL;
				torque_split_per_wheel[WHEEL_FR] = t_F * t_FR;
				torque_split_per_wheel[WHEEL_RL] = t_R * t_RL;
				torque_split_per_wheel[WHEEL_RR] = t_R * t_RR;
			}
			break;
			default:
			{
				// fatal - unknown diff!
				ASSERTMSG_(
					0,
					"DynamicsAckermannDrivetrain::invoke_motor_controllers: \
				       Unknown differential type!");
			}
			break;
		}

		ASSERT_(out_torque_per_wheel.size() == 4);
		for (int i = 0; i < 4; i++)
		{
			out_torque_per_wheel[i] =
				co.drive_torque * torque_split_per_wheel[i];
		}

		// Kinematically-driven steering wheels:
		// Ackermann formulas for inner&outer weels turning angles wrt the
		// equivalent (central) one:
		computeFrontWheelAngles(
			co.steer_ang, m_wheels_info[WHEEL_FL].yaw,
			m_wheels_info[WHEEL_FR].yaw);
	}
}

void DynamicsAckermannDrivetrain::computeFrontWheelAngles(
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

void DynamicsAckermannDrivetrain::computeDiffTorqueSplit(
	const double w1, const double w2, const double diffBias,
	const double splitRatio, double& t1, double& t2)
{
	if (mrpt::signWithZero(w1) == 0.0 || mrpt::signWithZero(w2) == 0.0)
	{
		t1 = splitRatio;
		t2 = 1.0 - splitRatio;
		return;
	}

	const double w1Abs = std::abs(w1);
	const double w2Abs = std::abs(w2);
	const double omegaMax = std::max(w1Abs, w2Abs);
	const double omegaMin = std::min(w1Abs, w2Abs);

	const double delta = omegaMax - diffBias * omegaMin;

	const double deltaTorque = (delta > 0) ? delta / omegaMax : 0.0f;
	const double f1 = (w1Abs - w2Abs > 0) ? splitRatio * (1.0f - deltaTorque)
										  : splitRatio * (1.0f + deltaTorque);
	const double f2 = (w1Abs - w2Abs > 0)
						  ? (1.0f - splitRatio) * (1.0f + deltaTorque)
						  : (1.0f - splitRatio) * (1.0f - deltaTorque);
	const double denom = 1.0f / (f1 + f2);

	t1 = f1 * denom;
	t2 = f2 * denom;
}

// See docs in base class:
mrpt::math::TTwist2D DynamicsAckermannDrivetrain::getVelocityLocalOdoEstimate()
	const
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

	return odo_vel;
}
