/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/WheelJointsInterface.h>
#include <mvsim/World.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

#include "xml_utils.h"

using namespace mvsim;

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
// Command of joints never commanded: zero speed (traction) or angle (steering)
constexpr double kNoCommandYet = 0.0;

bool isTraction(WheelJointsInterface::JointCommand c)
{
	return c == WheelJointsInterface::JointCommand::Velocity ||
		   c == WheelJointsInterface::JointCommand::Effort;
}
}  // namespace

void WheelJointsInterface::loadConfig(const rapidxml::xml_node<char>& node, const VehicleBase& veh)
{
	std::string commandInterface = "velocity";
	std::string robotDescription = "generated";

	TParameterDefinitions params;
	params["KP"] = TParamEntry("%lf", &KP);
	params["KI"] = TParamEntry("%lf", &KI);
	params["KD"] = TParamEntry("%lf", &KD);
	params["max_torque"] = TParamEntry("%lf", &max_torque);
	params["command_interface"] = TParamEntry("%s", &commandInterface);
	params["controllers_yaml"] = TParamEntry("%s", &controllersYaml_);
	params["robot_description"] = TParamEntry("%s", &robotDescription);

	parse_xmlnode_children_as_param(node, params, veh.parent()->user_defined_variables());

	if (commandInterface == "velocity")
	{
		commandMode_ = CommandMode::Velocity;
	}
	else if (commandInterface == "effort")
	{
		commandMode_ = CommandMode::Effort;
	}
	else
	{
		THROW_EXCEPTION_FMT(
			"Invalid <command_interface>: '%s' (valid: 'velocity', 'effort')",
			commandInterface.c_str());
	}

	if (robotDescription == "generated")
	{
		robotDescriptionFromTopic_ = false;
	}
	else if (robotDescription == "topic")
	{
		robotDescriptionFromTopic_ = true;
	}
	else
	{
		THROW_EXCEPTION_FMT(
			"Invalid <robot_description>: '%s' (valid: 'generated', 'topic')",
			robotDescription.c_str());
	}

	if (!controllersYaml_.empty())
	{
		controllersYaml_ = veh.parent()->local_to_abs_path(controllersYaml_);
	}

	const auto tractionCmd =
		commandMode_ == CommandMode::Velocity ? JointCommand::Velocity : JointCommand::Effort;
	joints_.clear();
	for (size_t i = 0; i < veh.getNumWheels(); i++)
	{
		joints_.push_back({veh.getWheelInfo(i).joint_name, JointKind::WheelSpin, i, tractionCmd});
	}
}

void WheelJointsInterface::loadConfigAckermann(
	const rapidxml::xml_node<char>& node, const VehicleBase& veh, const AckermannLayout& layout)
{
	loadConfig(node, veh);

	std::string steeringJoints = layout.centralTraction ? "central" : "per_wheel";
	std::string flSteerName = "fl_steer_joint";
	std::string frSteerName = "fr_steer_joint";
	std::string steerName = "steer_joint";
	std::string tractionName = "traction_joint";

	TParameterDefinitions params;
	params["steering_joints"] = TParamEntry("%s", &steeringJoints);
	params["fl_steer_joint_name"] = TParamEntry("%s", &flSteerName);
	params["fr_steer_joint_name"] = TParamEntry("%s", &frSteerName);
	params["steer_joint_name"] = TParamEntry("%s", &steerName);
	params["traction_joint_name"] = TParamEntry("%s", &tractionName);
	parse_xmlnode_children_as_param(node, params, veh.parent()->user_defined_variables());

	if (steeringJoints != "per_wheel" && steeringJoints != "central")
	{
		THROW_EXCEPTION_FMT(
			"Invalid <steering_joints>: '%s' (valid: 'per_wheel', 'central')",
			steeringJoints.c_str());
	}
	const bool central = steeringJoints == "central";

	if (layout.centralTraction)
	{
		// The wheels are driven by the drivetrain: state only
		const auto tractionCmd = joints_.front().command;
		for (auto& j : joints_)
		{
			j.command = JointCommand::None;
		}
		joints_.push_back({tractionName, JointKind::CentralTraction, 0, tractionCmd});
	}
	if (central)
	{
		joints_.push_back(
			{steerName, JointKind::CentralSteering, 0, JointCommand::Position, -layout.maxSteer,
			 layout.maxSteer, layout.maxSteerRate});
	}
	const auto wheelSteerCmd = central ? JointCommand::None : JointCommand::Position;
	for (const auto& [name, wheel, range] :
		 {std::tuple{flSteerName, layout.fl, layout.flSteerRange},
		  std::tuple{frSteerName, layout.fr, layout.frSteerRange}})
	{
		joints_.push_back(
			{name, JointKind::WheelSteering, wheel, wheelSteerCmd, range.first, range.second,
			 layout.maxSteerRate});
	}

	// Joint names must be unique:
	for (size_t i = 0; i < joints_.size(); i++)
	{
		for (size_t k = i + 1; k < joints_.size(); k++)
		{
			if (joints_[i].name == joints_[k].name)
			{
				THROW_EXCEPTION_FMT("Repeated joint name: '%s'", joints_[i].name.c_str());
			}
		}
	}
}

std::optional<size_t> WheelJointsInterface::findJoint(JointKind kind, size_t wheel) const
{
	for (size_t i = 0; i < joints_.size(); i++)
	{
		const auto& j = joints_[i];
		const bool wheelJoint = kind == JointKind::WheelSpin || kind == JointKind::WheelSteering;
		if (j.kind == kind && (!wheelJoint || j.wheel == wheel))
		{
			return i;
		}
	}
	return std::nullopt;
}

void WheelJointsInterface::setCommands(const std::vector<double>& commands)
{
	auto lck = std::lock_guard(mtx_);
	commands_ = commands;
}

void WheelJointsInterface::setCommand(size_t jointIndex, double command)
{
	auto lck = std::lock_guard(mtx_);
	if (commands_.size() <= jointIndex)
	{
		commands_.resize(jointIndex + 1, kNoCommandYet);
	}
	commands_[jointIndex] = command;
}

std::vector<double> WheelJointsInterface::getCommands() const
{
	auto lck = std::lock_guard(mtx_);
	auto cmds = commands_;
	cmds.resize(joints_.size(), kNoCommandYet);
	for (size_t i = 0; i < joints_.size(); i++)
	{
		if (joints_[i].command == JointCommand::None)
		{
			cmds[i] = kNaN;
		}
	}
	return cmds;
}

std::vector<WheelJointsInterface::JointState> WheelJointsInterface::getStates() const
{
	auto lck = std::lock_guard(mtx_);
	return states_;
}

double WheelJointsInterface::tractionTorque(
	size_t joint, double cmd, double speed, double radius, double dampingScale, double dt,
	MotorTorqueModel& motor)
{
	auto& pid = pids_[joint];

	double torque = 0;	// positive = forward (joint convention)
	if (!std::isfinite(cmd))
	{
		// No command: free wheel
		pid.reset();
	}
	else if (commandMode_ == CommandMode::Effort)
	{
		torque = cmd;
		motor.unsaturated = cmd;
	}
	else
	{
		// Velocity PID, with the error in m/s at the wheel rim:
		const double spVel = cmd * radius;

		pid.KP = KP;
		pid.KI = KI;
		pid.KD = KD;
		pid.max_out = max_torque;

		constexpr double zeroThreshold = 1e-3;	// m/s
		if (std::abs(spVel) < zeroThreshold)
		{
			// Braking toward a full stop: proportional action only, which
			// always opposes the motion and vanishes at zero speed, so the
			// vehicle neither coasts nor rebounds due to the integral term.
			pid.reset();
			torque = -KP * speed;
			motor.unsaturated = torque;
			motor.damping = KP * dampingScale;
		}
		else
		{
			torque = pid.compute(spVel - speed, dt);
			// Its reaction to the wheel own spin, d(torque)/d(w):
			motor.unsaturated = pid.lastUnsaturatedOutput();
			motor.damping = pid.lastOutputSensitivity() * dampingScale;
		}
	}
	motor.limit = max_torque;

	if (max_torque > 0)
	{
		torque = std::clamp(torque, -max_torque, max_torque);
	}
	lastTorques_[joint] = torque;
	return torque;
}

std::vector<double> WheelJointsInterface::computeWheelTorques(
	const VehicleBase& veh, double dt, std::vector<MotorTorqueModel>& motors)
{
	const size_t nW = veh.getNumWheels();
	const size_t nJ = joints_.size();

	auto lck = std::lock_guard(mtx_);

	commands_.resize(nJ, kNoCommandYet);
	pids_.resize(nJ);
	lastTorques_.resize(nJ, 0.0);
	motors.assign(nW, MotorTorqueModel());

	std::vector<double> torques(nW, 0.0);
	for (size_t j = 0; j < nJ; j++)
	{
		const auto& joint = joints_[j];
		if (joint.kind != JointKind::WheelSpin || !isTraction(joint.command) || joint.wheel >= nW)
		{
			continue;
		}
		const auto& wheel = veh.getWheelInfo(joint.wheel);
		const double R = 0.5 * wheel.diameter;
		const double torque =
			tractionTorque(j, commands_[j], wheel.getW() * R, R, R, dt, motors[joint.wheel]);

		// mvsim convention: positive torque makes the vehicle move backwards
		torques[joint.wheel] = -torque;
	}
	return torques;
}

double WheelJointsInterface::computeCentralTractionTorque(
	double speed, double radius, double dt, MotorTorqueModel& motor)
{
	const size_t nJ = joints_.size();
	const auto j = findJoint(JointKind::CentralTraction);
	ASSERT_(j.has_value());

	auto lck = std::lock_guard(mtx_);
	commands_.resize(nJ, kNoCommandYet);
	pids_.resize(nJ);
	lastTorques_.resize(nJ, 0.0);

	motor = MotorTorqueModel();
	// The speed is already the drivetrain one, so its damping is per unit of it:
	return -tractionTorque(*j, commands_[*j], speed, radius, 1.0, dt, motor);
}

void WheelJointsInterface::ackermannSteering(
	double currentSteer, const std::array<double, 2>& currentWheels, double& steer,
	std::optional<std::array<double, 2>>& wheelsSteer) const
{
	const auto cmds = getCommands();
	const auto cmdOr = [&cmds](std::optional<size_t> j, double current)
	{ return j && std::isfinite(cmds[*j]) ? cmds[*j] : current; };

	if (const auto j = findJoint(JointKind::CentralSteering); j)
	{
		steer = cmdOr(j, currentSteer);
		wheelsSteer.reset();
		return;
	}
	// The front wheel steering joints, FL then FR (see loadConfigAckermann()):
	std::array<double, 2> angles = currentWheels;
	size_t k = 0;
	for (size_t j = 0; j < joints_.size() && k < 2; j++)
	{
		if (joints_[j].kind == JointKind::WheelSteering)
		{
			angles[k] = cmdOr(j, currentWheels[k]);
			k++;
		}
	}
	wheelsSteer = angles;
}

void WheelJointsInterface::updateStates(
	const VehicleBase& veh, double dt, const CentralJointValues& central)
{
	const size_t nJ = joints_.size();

	auto lck = std::lock_guard(mtx_);
	const bool firstTime = states_.size() != nJ;
	states_.resize(nJ);
	lastTorques_.resize(nJ, 0.0);

	// Angle joint state from its new position:
	const auto setAngle = [&](JointState& st, double position)
	{
		st.velocity = (!firstTime && dt > 0) ? (position - st.position) / dt : 0.0;
		st.position = position;
	};

	for (size_t j = 0; j < nJ; j++)
	{
		const auto& joint = joints_[j];
		auto& st = states_[j];
		switch (joint.kind)
		{
			case JointKind::WheelSpin:
			{
				const auto& wheel = veh.getWheelInfo(joint.wheel);
				st.position = wheel.getPhiContinuous();
				st.velocity = wheel.getW();
			}
			break;
			case JointKind::WheelSteering:
				setAngle(st, veh.getWheelInfo(joint.wheel).yaw);
				break;
			case JointKind::CentralSteering:
				setAngle(st, central.steering);
				break;
			case JointKind::CentralTraction:
				st.velocity = central.tractionVelocity;
				st.position += st.velocity * dt;
				break;
		};
		st.effort = joint.hasEffortState() ? lastTorques_[j] : 0.0;
	}
}
