/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2025-2026 Fernando Cañadas Aránega                        |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/math/geometry.h>
#include <mvsim/FrictionModels/AdaptativeFriction.h>
#include <mvsim/FrictionModels/DefaultFriction.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/World.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>  // std::clamp
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "benchmark_msg/msg/benchmark_params.hpp"
#include "xml_utils.h"

using namespace mvsim;

double torqueL = 0.0;
double torqueR = 0.0;

// Variables estáticas para el nodo ROS (compartido entre todas las instancias)
static std::shared_ptr<rclcpp::Node> ros_node_static = nullptr;
static rclcpp::Publisher<benchmark_msg::msg::BenchmarkParams>::SharedPtr benchmark_pub_static =
	nullptr;
static std::once_flag ros_init_flag;

// Constructor implementation
DynamicsDifferential::ControllerTwistPID::ControllerTwistPID(DynamicsDifferential& veh)
	: ControllerBase(veh),
	  KP(0.0),
	  KI(0.0),
	  KD(0.0),
	  N(0.0),
	  pitch(0.0),
	  tau_ff(0.0),
	  K_ff(0.0),
	  tau_ff1(0.0),
	  K_ff1(0.0),
	  tau_ff2(0.0),
	  K_ff2(0.0),
	  tau_ff3(0.0),
	  K_ff3(0.0),
	  tau_ff4(0.0),
	  K_ff4(0.0),
	  tau_ff5(0.0),
	  K_ff5(0.0),
	  tau_ff6(0.0),
	  K_ff6(0.0),
	  tau_f(0.0),
	  n_f(1),
	  torque_slope(0.0),
	  new_vel_max(0.0),
	  new_w_max(0.0),
	  dist_obst(0.0),
	  enable_antiwindup(false),
	  enable_feedforward(false),
	  enable_referencefilter(false),
	  enable_adaptative(false),
	  full_payload(false),
	  max_torque(0.0),
	  joyMaxAngSpeed(M_PI / 4),
	  distWheels_(veh.wheels_info_[0].y - veh.wheels_info_[1].y)
{
	ASSERT_(distWheels_ > 0);

	// Inicializar ROS solo una vez usando std::call_once
	std::call_once(
		ros_init_flag,
		[]()
		{
			if (!rclcpp::ok())
			{
				rclcpp::init(0, nullptr);
			}
			ros_node_static = std::make_shared<rclcpp::Node>("pid_controller");

			benchmark_pub_static =
				ros_node_static->create_publisher<benchmark_msg::msg::BenchmarkParams>(
					"benchmark_params", 10);
		});

	// Asignar las variables estáticas a los miembros de la clase
	ros_node_ = ros_node_static;
	benchmark_pub_ = benchmark_pub_static;
}

// Control step implementation
void DynamicsDifferential::ControllerTwistPID::control_step(
	const DynamicsDifferential::TControllerInput& ci, DynamicsDifferential::TControllerOutput& co)
{
	const auto sp = setpoint();

	// For each wheel:
	// 1) Compute desired velocity set-point (in m/s)
	// 2) Run the PI/PID for that wheel independently (in newtons)
	const double spVelL = sp.vx - 0.5 * sp.omega * distWheels_;
	const double spVelR = sp.vx + 0.5 * sp.omega * distWheels_;

	// Compute each wheel actual velocity (from an "odometry" estimation of
	// velocity, not ground-truth!):
	const mrpt::math::TTwist2D vehVelOdo = veh_.getVelocityLocalOdoEstimate();
	const double actVelL = vehVelOdo.vx - 0.5 * vehVelOdo.omega * distWheels_;
	const double actVelR = vehVelOdo.vx + 0.5 * vehVelOdo.omega * distWheels_;

	// ---------------------------  Slope  Disturbance  -----------------------------

	// 1. DECLARE ARGUMENTS
	std::string friction_model = "Unknown";
	std::vector<mvsim::AdaptativeFriction::ZoneParams> zones;
	double yaw = 0.0, pitch_rad = 0.0, roll = 0.0;
	double mu = 0.0, damping = 0.0, Crr = 0.0;
	double zone1 = 0.0, zone2 = 0.0, zone3 = 0.0;
	double Fx_max = 0.0;

	// 2. PITCH
	veh_.getCPose3D().getYawPitchRoll(yaw, pitch_rad, roll);

	// 3. FRICTION MODEL - ADAPTATIVE OR DEFAULT
	if (!veh_.frictions_.empty() && veh_.frictions_[0])
	{
		auto base_fric = veh_.frictions_[0];

		if (auto fric_adapt = std::dynamic_pointer_cast<mvsim::AdaptativeFriction>(base_fric))
		{
			friction_model = "AdaptativeFriction";
			mu = fric_adapt->getMu();
			damping = fric_adapt->getCdamping();
			Crr = fric_adapt->getCrr();
			Fx_max = fric_adapt->getLongitudinalForce();
			zones = fric_adapt->getZones();
			if (zones.size() >= 3)
			{
				zone1 = zones[0].mu;
				zone2 = zones[1].mu;
				zone3 = zones[2].mu;
			}
		}
		else if (auto fric_def = std::dynamic_pointer_cast<mvsim::DefaultFriction>(base_fric))
		{
			friction_model = "DefaultFriction";
			mu = fric_def->getMu();
			damping = fric_def->getCdamping();
			Crr = fric_def->getCrr();
		}
		else
		{
			RCLCPP_WARN_ONCE(
				ros_node_->get_logger(), "Unknown friction model type. Using default parameters.");
		}
	}
	else
	{
		RCLCPP_ERROR_ONCE(ros_node_->get_logger(), "No friction model found.");
	}
	RCLCPP_INFO_ONCE(ros_node_->get_logger(), "Friction model: DefaultFriction");

	// 4. MVSIM VALUES
	const int nW = static_cast<int>(veh_.getNumWheels());  // Number wheels
	double m = veh_.getChassisMass();  // Total Robot Mass
	const double g = veh_.parent()->get_gravity();	// Gravity
	const double r = veh_.getWheelInfo(0).diameter * 0.5;  // Radius Wheels
	const double v = veh_.getVelocityLocalOdoEstimate().vx;	 // Velocidad longitudinal

	double massPerWheel = m / nW;
	double F_grade = massPerWheel * g * std::sin(pitch_rad);  // Gravity force [N]
	double F_roll_Crr = Crr * massPerWheel * g * std::cos(pitch_rad);  // Rolling force [N]
	double max_friction = mu * massPerWheel * g;
	double F_total = F_grade + F_roll_Crr;
	double F_friction_lon = std::clamp(F_total, -max_friction, max_friction);

	double torque_dampinR = damping * actVelR;	// Additional torque [Nm]
	double torque_dampingL = damping * actVelL;	 // Additional torque [Nm]
	double torque_damping = (torque_dampinR + torque_dampingL) / 2.0;
	torque_slope = (F_friction_lon * r) + torque_damping;

	if (v > -0.01 && v < 0.01)
	{
		torque_slope = 0.0;
	}

	// Configure PID controllers
	for (auto& pid : PIDs_)
	{
		pid.KP = KP;
		pid.KI = KI;
		pid.KD = KD;
		pid.N = N;

		if (friction_model == "AdaptativeFriction")
		{
			if (enable_adaptative)
			{
				if (full_payload)
				{
					if (mu == zone1)
					{
						pid.tau_ff = tau_ff1;
						pid.K_ff = K_ff1;
					}
					else if (mu == zone2)
					{
						pid.tau_ff = tau_ff2;
						pid.K_ff = K_ff2;
					}
					else if (mu == zone3)
					{
						pid.tau_ff = tau_ff3;
						pid.K_ff = K_ff3;
					}
				}
				else
				{
					if (mu == zone1)
					{
						pid.tau_ff = tau_ff4;
						pid.K_ff = K_ff4;
					}
					else if (mu == zone2)
					{
						pid.tau_ff = tau_ff5;
						pid.K_ff = K_ff5;
					}
					else if (mu == zone3)
					{
						pid.tau_ff = tau_ff6;
						pid.K_ff = K_ff6;
					}
				}
			}
			else
			{
				pid.tau_ff = tau_ff;
				pid.K_ff = K_ff;
			}
		}
		else
		{
			pid.tau_ff = tau_ff;
			pid.K_ff = K_ff;
		}

		pid.tau_f = tau_f;
		pid.n_f = n_f;
		pid.enable_antiwindup = enable_antiwindup;
		pid.enable_feedforward = enable_feedforward;
		pid.enable_referencefilter = enable_referencefilter;
		pid.max_out = max_torque;
	}

	// Publish benchmark parameters
	benchmark_msg::msg::BenchmarkParams message;
	message.kp = KP;
	message.ki = KI;
	message.kd = KD;
	message.n = N;
	message.tau_ff = tau_ff;
	message.k_ff = K_ff;
	message.tau_ff1 = tau_ff1;
	message.k_ff1 = K_ff1;
	message.tau_ff2 = tau_ff2;
	message.k_ff2 = K_ff2;
	message.tau_ff3 = tau_ff3;
	message.k_ff3 = K_ff3;
	message.tau_ff4 = tau_ff4;
	message.k_ff4 = K_ff4;
	message.tau_ff5 = tau_ff5;
	message.k_ff5 = K_ff5;
	message.tau_ff6 = tau_ff6;
	message.k_ff6 = K_ff6;
	message.tau_f = tau_f;
	message.n_f = n_f;
	message.enable_antiwindup = enable_antiwindup;
	message.enable_feedforward = enable_feedforward;
	message.enable_referencefilter = enable_referencefilter;
	message.enable_adaptative = enable_adaptative;
	message.full_payload = full_payload;
	message.max_torque = max_torque;
	message.mu = mu;
	message.crr = Crr;
	message.damping = damping;
	message.mass = m;
	message.mu_zone1 = zone1;
	message.mu_zone2 = zone2;
	message.mu_zone3 = zone3;
	message.new_vel_max = new_vel_max;
	message.new_w_max = new_w_max;
	message.dist_obst = dist_obst;
	message.sp_vel_l = spVelL;
	message.sp_vel_r = spVelR;
	message.act_vel_l = actVelL;
	message.act_vel_r = actVelR;
	message.error_l = spVelL - actVelL;
	message.error_r = spVelR - actVelR;
	message.torque_l = torqueL;
	message.torque_r = torqueR;
	message.pitch = pitch_rad;
	message.torque_slope = torque_slope;
	message.fmax = Fx_max;
	message.max_friction = max_friction;

	const double zero_threshold = 0.001;  // m/s
	const double stop_threshold = 0.05;	 // m/s , wider threshold for stop detection

	const bool setpointIsZero =
		std::abs(spVelL) < zero_threshold && std::abs(spVelR) < zero_threshold;

	if (setpointIsZero && std::abs(actVelL) < stop_threshold && std::abs(actVelR) < stop_threshold)
	{
		// Near-zero velocity with zero setpoint: full stop, reset PIDs
		co.wheel_torque_l = 0;
		co.wheel_torque_r = 0;
		for (auto& pid : PIDs_)
		{
			pid.reset();
		}
	}
	else
	{
		torqueL = -PIDs_[0].compute(spVelL, actVelL, torque_slope, ci.context.dt);
		torqueR = -PIDs_[1].compute(spVelR, actVelR, torque_slope, ci.context.dt);
		co.wheel_torque_l = torqueL;
		co.wheel_torque_r = torqueR;

		if (setpointIsZero)
		{
			// Braking toward zero: clamp torque so it can only oppose current
			// motion direction, preventing the PID integral from causing rebound.
			// Sign convention: positive co.wheel_torque = backward motor force.
			if (actVelL > 0)
			{
				co.wheel_torque_l = std::max(0.0, co.wheel_torque_l);
			}
			else if (actVelL < 0)
			{
				co.wheel_torque_l = std::min(0.0, co.wheel_torque_l);
			}

			if (actVelR > 0)
			{
				co.wheel_torque_r = std::max(0.0, co.wheel_torque_r);
			}
			else if (actVelR < 0)
			{
				co.wheel_torque_r = std::min(0.0, co.wheel_torque_r);
			}
		}
	}

	if (benchmark_pub_)
	{
		benchmark_pub_->publish(message);
	}
}

void DynamicsDifferential::ControllerTwistPID::load_config(const rapidxml::xml_node<char>& node)
{
	TParameterDefinitions params;
	params["KP"] = TParamEntry("%lf", &KP);
	params["KI"] = TParamEntry("%lf", &KI);
	params["KD"] = TParamEntry("%lf", &KD);
	params["N"] = TParamEntry("%lf", &N);
	params["tau_ff"] = TParamEntry("%lf", &tau_ff);
	params["K_ff"] = TParamEntry("%lf", &K_ff);
	params["tau_ff1"] = TParamEntry("%lf", &tau_ff1);
	params["K_ff1"] = TParamEntry("%lf", &K_ff1);
	params["tau_ff2"] = TParamEntry("%lf", &tau_ff2);
	params["K_ff2"] = TParamEntry("%lf", &K_ff2);
	params["tau_ff3"] = TParamEntry("%lf", &tau_ff3);
	params["K_ff3"] = TParamEntry("%lf", &K_ff3);
	params["tau_ff4"] = TParamEntry("%lf", &tau_ff4);
	params["K_ff4"] = TParamEntry("%lf", &K_ff4);
	params["tau_ff5"] = TParamEntry("%lf", &tau_ff5);
	params["K_ff5"] = TParamEntry("%lf", &K_ff5);
	params["tau_ff6"] = TParamEntry("%lf", &tau_ff6);
	params["K_ff6"] = TParamEntry("%lf", &K_ff6);
	params["tau_f"] = TParamEntry("%lf", &tau_f);
	params["n_f"] = TParamEntry("%d", &n_f);
	params["new_vel_max"] = TParamEntry("%lf", &new_vel_max);
	params["new_w_max"] = TParamEntry("%lf", &new_w_max);
	params["dist_obst"] = TParamEntry("%lf", &dist_obst);
	params["enable_antiwindup"] = TParamEntry("%bool", &enable_antiwindup);
	params["enable_feedforward"] = TParamEntry("%bool", &enable_feedforward);
	params["enable_referencefilter"] = TParamEntry("%bool", &enable_referencefilter);
	params["enable_adaptative"] = TParamEntry("%bool", &enable_adaptative);
	params["full_payload"] = TParamEntry("%bool", &full_payload);
	params["max_torque"] = TParamEntry("%lf", &max_torque);

	// Initial speed
	params["V"] = TParamEntry("%lf", &setpoint_.vx);
	params["W"] = TParamEntry("%lf_deg", &setpoint_.omega);

	parse_xmlnode_children_as_param(node, params);

	RCLCPP_INFO(
		ros_node_->get_logger(),
		"PID loaded from XML: KP=%.3f, KI=%.3f, KD=%.3f, enable_adaptative=%s", KP, KI, KD,
		enable_adaptative ? "true" : "false");
}

void DynamicsDifferential::ControllerTwistPID::teleop_interface(
	const TeleopInput& in, TeleopOutput& out)
{
	ControllerBase::teleop_interface(in, out);

	auto lck = mrpt::lockHelper(setpointMtx_);

	switch (in.keycode)
	{
		case 'W':
		case 'w':
			setpoint_.vx += 0.1;
			break;

		case 'S':
		case 's':
			setpoint_.vx -= 0.1;
			break;

		case 'A':
		case 'a':
			setpoint_.omega += 2.0 * M_PI / 180;
			break;

		case 'D':
		case 'd':
			setpoint_.omega -= 2.0 * M_PI / 180;
			break;

		case ' ':
		{
			setpoint_ = {0, 0, 0};
			for (auto& pid : PIDs_)
			{
				pid.reset();
			}
		}
		break;
	};

	out.append_gui_lines += "[Controller=" + std::string(class_name()) + "]";

	if (in.js && in.js->axes.size() >= 2)
	{
		const auto& js = in.js.value();
		const float js_x = js.axes[0];
		const float js_y = js.axes[1];

		setpoint_.vx = -js_y * joyMaxLinSpeed;
		setpoint_.omega = -js_x * joyMaxAngSpeed;

		if (js.buttons.size() > 7)
		{
			if (js.buttons[5]) joyMaxLinSpeed *= 1.01;
			if (js.buttons[7]) joyMaxLinSpeed /= 1.01;

			if (js.buttons[4]) joyMaxAngSpeed *= 1.01;
			if (js.buttons[6]) joyMaxAngSpeed /= 1.01;

			if (js.buttons[3])	// brake
			{
				setpoint_ = {0, 0, 0};
				for (auto& pid : PIDs_)
				{
					pid.reset();
				}
			}
		}

		out.append_gui_lines += mrpt::format(
			"Teleop joystick:\n"
			"maxLinSpeed=%.03f m/s\n"
			"maxAngSpeed=%.03f deg/s\n",
			joyMaxLinSpeed, mrpt::RAD2DEG(joyMaxAngSpeed));
	}
	else
	{
		out.append_gui_lines +=
			"Teleop keys:\n"
			"w/s=forward/backward.\n"
			"a/d=left/right.\n"
			"spacebar=stop.\n";
	}

	out.append_gui_lines += mrpt::format(
		"setpoint: lin=%.03f ang=%.03f deg/s\n", setpoint_.vx, 180.0 / M_PI * setpoint_.omega);
}
