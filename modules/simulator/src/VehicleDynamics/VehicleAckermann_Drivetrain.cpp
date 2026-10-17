/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/viz/Scene.h>
#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>
#include <mvsim/World.h>

#include <cmath>
#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

// Ctor:
DynamicsAckermannDrivetrain::DynamicsAckermannDrivetrain(World* parent)
	: VehicleBase(parent, 4 /*num wheels*/), max_steer_ang_(mrpt::DEG2RAD(30))
{
	using namespace mrpt::math;

	chassis_mass_ = 500.0;
	chassis_z_min_ = 0.20;
	chassis_z_max_ = 1.40;
	chassis_color_ = mrpt::img::TColor(0xe8, 0x30, 0x00);

	// Default shape:
	chassis_poly_.clear();
	chassis_poly_.emplace_back(-0.8, -1.0);
	chassis_poly_.emplace_back(-0.8, 1.0);
	chassis_poly_.emplace_back(1.5, 0.9);
	chassis_poly_.emplace_back(1.8, 0.8);
	chassis_poly_.emplace_back(1.8, -0.8);
	chassis_poly_.emplace_back(1.5, -0.9);
	updateMaxRadiusFromPoly();

	fixture_chassis_ = nullptr;
	for (int i = 0; i < 4; i++) fixture_wheels_[i] = nullptr;

	// Default values:
	// rear-left:
	wheels_info_[WHEEL_RL].x = 0;
	wheels_info_[WHEEL_RL].y = 0.9;
	// rear-right:
	wheels_info_[WHEEL_RR].x = 0;
	wheels_info_[WHEEL_RR].y = -0.9;
	// Front-left:
	wheels_info_[WHEEL_FL].x = 1.3;
	wheels_info_[WHEEL_FL].y = 0.9;
	// Front-right:
	wheels_info_[WHEEL_FR].x = 1.3;
	wheels_info_[WHEEL_FR].y = -0.9;

	FrontRearSplit_ = 0.5;
	FrontLRSplit_ = 0.5;
	RearLRSplit_ = 0.5;

	FrontRearBias_ = 1.5;
	FrontLRBias_ = 1.5;
	RearLRBias_ = 1.5;

	diff_type_ = DIFF_TORSEN_4WD;
}

/** The derived-class part of load_params_from_xml() */
void DynamicsAckermannDrivetrain::dynamics_load_params_from_xml(
	const rapidxml::xml_node<char>* xml_node)
{
	// <chassis ...> </chassis>
	const rapidxml::xml_node<char>* xml_chassis = xml_node->first_node("chassis");
	if (xml_chassis)
	{
		// Attribs:
		TParameterDefinitions attribs;
		attribs["mass"] = TParamEntry("%lf", &this->chassis_mass_);
		attribs["zmin"] = TParamEntry("%lf", &this->chassis_z_min_);
		attribs["zmax"] = TParamEntry("%lf", &this->chassis_z_max_);
		attribs["color"] = TParamEntry("%color", &this->chassis_color_);

		parse_xmlnode_attribs(
			*xml_chassis, attribs, {},
			"[DynamicsAckermannDrivetrain::dynamics_load_params_from_xml]");

		// Shape node (optional, fallback to default shape if none found)
		const rapidxml::xml_node<char>* xml_shape = xml_chassis->first_node("shape");
		if (xml_shape)
		{
			mvsim::parse_xmlnode_shape(
				*xml_shape, chassis_poly_,
				"[DynamicsAckermannDrivetrain::dynamics_load_params_from_xml]");
		}
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
			wheels_info_[i].loadFromXML(xml_wheel);
		}
		else
		{
			world_->logFmt(
				mrpt::system::LVL_WARN, "No XML entry '%s' found: using defaults for wheel #%u",
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
		ack_ps["max_steer_ang_deg"] = TParamEntry("%lf_deg", &max_steer_ang_);
		ack_ps["max_steer_rate_deg"] = TParamEntry("%lf_deg", &max_steer_rate_);

		parse_xmlnode_children_as_param(
			*xml_node, ack_ps, {}, "[DynamicsAckermannDrivetrain::dynamics_load_params_from_xml]");

		// Front-left:
		wheels_info_[WHEEL_FL].x = front_x;
		wheels_info_[WHEEL_FL].y = 0.5 * front_d;
		// Front-right:
		wheels_info_[WHEEL_FR].x = front_x;
		wheels_info_[WHEEL_FR].y = -0.5 * front_d;
	}

	// Drivetrain parameters
	// Watch the order of DifferentialType enum!
	const char* drive_names[6] = {"open_front",	  "open_rear",	 "open_4wd",
								  "torsen_front", "torsen_rear", "torsen_4wd"};

	const rapidxml::xml_node<char>* xml_drive = xml_node->first_node("drivetrain");
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
				diff_type_ = (DifferentialType)i;
				break;
			}
		}

		TParameterDefinitions drive_params;
		drive_params["front_rear_split"] = TParamEntry("%lf", &FrontRearSplit_);
		drive_params["front_rear_bias"] = TParamEntry("%lf", &FrontRearBias_);
		drive_params["front_left_right_split"] = TParamEntry("%lf", &FrontLRSplit_);
		drive_params["front_left_right_bias"] = TParamEntry("%lf", &FrontLRBias_);
		drive_params["rear_left_right_split"] = TParamEntry("%lf", &RearLRSplit_);
		drive_params["rear_left_right_bias"] = TParamEntry("%lf", &RearLRBias_);

		parse_xmlnode_children_as_param(
			*xml_drive, drive_params, {},
			"[DynamicsAckermannDrivetrain::dynamics_load_params_from_xml]");
	}

	// Vehicle controller:
	// -------------------------------------------------
	{
		const rapidxml::xml_node<char>* xml_control = xml_node->first_node("controller");
		if (xml_control)
		{
			rapidxml::xml_attribute<char>* control_class = xml_control->first_attribute("class");
			if (!control_class || control_class->value_size() == 0)
			{
				THROW_EXCEPTION(
					"[DynamicsAckermannDrivetrain] Missing 'class' attribute in "
					"<controller> XML node");
			}
			const std::string sCtrlClass = std::string(control_class->value());
			if (sCtrlClass == ControllerRawForces::class_name())
			{
				controller_ = std::make_shared<ControllerRawForces>(*this);
			}
			else if (sCtrlClass == ControllerTwistFrontSteerPID::class_name())
			{
				controller_ = std::make_shared<ControllerTwistFrontSteerPID>(*this);
			}
			else if (sCtrlClass == ControllerFrontSteerPID::class_name())
			{
				controller_ = std::make_shared<ControllerFrontSteerPID>(*this);
			}
			else
			{
				THROW_EXCEPTION_FMT(
					"[DynamicsAckermannDrivetrain] Unknown 'class'='%s' in "
					"<controller> XML node",
					sCtrlClass.c_str());
			}

			controller_->load_config(*xml_control);
		}
	}

	// Default controller:
	if (!controller_)
	{
		controller_ = std::make_shared<ControllerRawForces>(*this);
	}
}

// See docs in base class:
std::vector<double> DynamicsAckermannDrivetrain::invoke_motor_controllers(
	const TSimulContext& context)
{
	// Longitudinal forces at each wheel:
	std::vector<double> out_torque_per_wheel;
	out_torque_per_wheel.assign(4, 0.0);

	if (controller_)
	{
		// Reset before invoking , the ideal controller will re-set it if active:
		idealControllerActive_ = false;

		// Invoke controller:
		TControllerInput ci;
		ci.context = context;
		TControllerOutput co;
		controller_->control_step(ci, co);
		// Take its output:

		const bool torsen = diff_type_ == DIFF_TORSEN_FRONT || diff_type_ == DIFF_TORSEN_REAR ||
							diff_type_ == DIFF_TORSEN_4WD;
		const bool frontDriven = diff_type_ != DIFF_OPEN_REAR && diff_type_ != DIFF_TORSEN_REAR;
		const bool rearDriven = diff_type_ != DIFF_OPEN_FRONT && diff_type_ != DIFF_TORSEN_FRONT;
		const double frontSplit =
			frontDriven && rearDriven ? FrontRearSplit_ : (frontDriven ? 1.0 : 0.0);

		// Locking stiffness of Torsen differentials: inertia of each output / dt
		double Iwheel = 0;
		for (const auto& w : wheels_info_)
		{
			Iwheel += 0.25 * w.Iyy;
		}
		const double kAxle = Iwheel / context.dt;
		const double kCenter = 2 * Iwheel / context.dt;

		// Drive torque, forward positive:
		const double T = -co.drive_torque;
		double fwd[4] = {0, 0, 0, 0};  // wheel torques, forward positive
		double split[4] = {0, 0, 0, 0};	 // d(wheel torque)/d(drive torque)
		double extraDamping[4] = {0, 0, 0, 0};	// from the Torsen locking

		// Center differential:
		double Tfront = frontSplit * T;
		double Trear = (1 - frontSplit) * T;
		if (torsen && frontDriven && rearDriven)
		{
			const auto axleW = [this](int l, int r)
			{ return 0.5 * (wheels_info_[l].getW() + wheels_info_[r].getW()); };
			double lock = 0;
			const bool sat = torsenLockTorque(
				axleW(WHEEL_FL, WHEEL_FR), axleW(WHEEL_RL, WHEEL_RR), T, FrontRearSplit_,
				FrontRearBias_, kCenter, lock);
			Tfront -= lock;
			Trear += lock;
			if (!sat)
			{
				extraDamping[WHEEL_FL] += 0.25 * kCenter;
				extraDamping[WHEEL_FR] += 0.25 * kCenter;
				extraDamping[WHEEL_RL] += 0.25 * kCenter;
				extraDamping[WHEEL_RR] += 0.25 * kCenter;
			}
		}

		// Axle differentials:
		const auto axle = [&](int l, int r, double Taxle, double axleSplit, double lrSplit,
							  double bias, bool driven)
		{
			if (!driven)
			{
				return;
			}
			double lock = 0;
			bool sat = true;
			if (torsen)
			{
				sat = torsenLockTorque(
					wheels_info_[l].getW(), wheels_info_[r].getW(), Taxle, lrSplit, bias, kAxle,
					lock);
			}
			fwd[l] = lrSplit * Taxle - lock;
			fwd[r] = (1 - lrSplit) * Taxle + lock;
			split[l] = axleSplit * lrSplit;
			split[r] = axleSplit * (1 - lrSplit);
			if (!sat)
			{
				extraDamping[l] += kAxle;
				extraDamping[r] += kAxle;
			}
		};
		axle(WHEEL_FL, WHEEL_FR, Tfront, frontSplit, FrontLRSplit_, FrontLRBias_, frontDriven);
		axle(WHEEL_RL, WHEEL_RR, Trear, 1 - frontSplit, RearLRSplit_, RearLRBias_, rearDriven);

		ASSERT_(out_torque_per_wheel.size() == 4);
		std::array<double, 4> dSpeed_dw;
		driveshaftSpeed(&dSpeed_dw);
		wheelMotors_.assign(4, MotorTorqueModel());
		for (int i = 0; i < 4; i++)
		{
			out_torque_per_wheel[i] = -fwd[i];

			// Each wheel reaction to its own spin only (the rest are neglected):
			const double lockPart = fwd[i] - split[i] * T;
			auto& m = wheelMotors_[i];
			m.damping = co.drive_motor.damping * split[i] * dSpeed_dw[i] + extraDamping[i];
			m.unsaturated = co.drive_motor.unsaturated * split[i] + lockPart;
			m.limit = co.drive_motor.limit * split[i] + std::abs(lockPart);
		}

		// Kinematically-driven steering wheels:
		// Ackermann formulas for inner&outer weels turning angles wrt the
		// equivalent (central) one:
		computeFrontWheelAngles(
			rateLimitedSteering(co.steer_ang, context.dt), wheels_info_[WHEEL_FL].yaw,
			wheels_info_[WHEEL_FR].yaw);
	}
	return out_torque_per_wheel;
}

void DynamicsAckermannDrivetrain::computeFrontWheelAngles(
	const double desired_equiv_steer_ang, double& out_fl_ang, double& out_fr_ang) const
{
	// EQ1: cot(d)+0.5*w/l = cot(do)
	// EQ2: cot(di)=cot(do)-w/l
	const double w = wheels_info_[WHEEL_FL].y - wheels_info_[WHEEL_FR].y;
	const double l = wheels_info_[WHEEL_FL].x - wheels_info_[WHEEL_RL].x;
	ASSERT_(l > 0);
	const double w_l = w / l;
	const double delta = std::clamp(std::abs(desired_equiv_steer_ang), 0.0, max_steer_ang_);

	const bool delta_neg = (desired_equiv_steer_ang < 0);
	ASSERT_LT_(delta, 0.5 * M_PI - 0.01);
	const double cot_do = 1.0 / tan(delta) + 0.5 * w_l;
	const double cot_di = cot_do - w_l;
	// delta>0: do->right, di->left wheel
	// delta<0: do->left , di->right wheel
	(delta_neg ? out_fr_ang : out_fl_ang) = atan(1.0 / cot_di) * (delta_neg ? -1.0 : 1.0);
	(delta_neg ? out_fl_ang : out_fr_ang) = atan(1.0 / cot_do) * (delta_neg ? -1.0 : 1.0);
}

bool DynamicsAckermannDrivetrain::torsenLockTorque(
	double wa, double wb, double inputTorque, double split, double biasRatio, double k,
	double& lockTorque)
{
	// Bias ratio B = max/min output torques. With outputs split*T - lock and
	// (1-split)*T + lock, this limits |lock| to beta*2*min(split,1-split)*|T|:
	const double beta = (std::max(1.0, biasRatio) - 1.0) / (std::max(1.0, biasRatio) + 1.0);
	const double maxLock = beta * 2 * std::min(split, 1 - split) * std::abs(inputTorque);
	lockTorque = k * (wa - wb);
	if (std::abs(lockTorque) > maxLock)
	{
		lockTorque = lockTorque > 0 ? maxLock : -maxLock;
		return true;
	}
	return false;
}

// See docs in base class:
mrpt::math::TTwist2D DynamicsAckermannDrivetrain::getVelocityLocalOdoEstimate() const
{
	mrpt::math::TTwist2D odo_vel;
	// Equations:

	// Velocities in local +X at each wheel i={0,1}:
	// v_i = vx - w_veh * wheel_{i,y}  =  w_i * R_i
	// Re-arranging:
	const double w0 = wheels_info_[WHEEL_RL].getW();
	const double w1 = wheels_info_[WHEEL_RR].getW();
	const double R0 = wheels_info_[WHEEL_RL].diameter * 0.5;
	const double R1 = wheels_info_[WHEEL_RR].diameter * 0.5;

	const double Ay = wheels_info_[WHEEL_RL].y - wheels_info_[WHEEL_RR].y;
	ASSERTMSG_(
		Ay != 0.0,
		"The two wheels of a differential vehicle cannot be at the same Y "
		"coordinate!");

	const double w_veh = (w1 * R1 - w0 * R0) / Ay;
	const double vx_veh = w0 * R0 + w_veh * wheels_info_[WHEEL_RL].y;

	odo_vel.vx = vx_veh;
	odo_vel.vy = 0.0;
	odo_vel.omega = w_veh;

	return odo_vel;
}

double DynamicsAckermannDrivetrain::driveshaftSpeed(std::array<double, 4>* dSpeed_dw) const
{
	double wFront = 0;
	double wRear = 0;
	switch (diff_type_)
	{
		case DIFF_OPEN_FRONT:
		case DIFF_TORSEN_FRONT:
			wFront = 1.0;
			break;
		case DIFF_OPEN_REAR:
		case DIFF_TORSEN_REAR:
			wRear = 1.0;
			break;
		default:
			wFront = 0.5;
			wRear = 0.5;
			break;
	};

	double speed = 0;
	for (int i = 0; i < 4; i++)
	{
		const auto& w = wheels_info_[i];
		const double axleWeight = (i == WHEEL_FL || i == WHEEL_FR) ? wFront : wRear;
		// Each wheel contributes half of its axle speed:
		const double d = axleWeight * 0.5 * 0.5 * w.diameter * std::cos(w.yaw);
		speed += d * w.getW();
		if (dSpeed_dw)
		{
			(*dSpeed_dw)[i] = d;
		}
	}
	return speed;
}
