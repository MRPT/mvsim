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

#include "xml_utils.h"

using namespace mvsim;

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
}

void WheelJointsInterface::setCommands(const std::vector<double>& commands)
{
	auto lck = std::lock_guard(mtx_);
	commands_ = commands;
}

std::vector<WheelJointsInterface::JointState> WheelJointsInterface::getStates() const
{
	auto lck = std::lock_guard(mtx_);
	return states_;
}

std::vector<double> WheelJointsInterface::computeWheelTorques(const VehicleBase& veh, double dt)
{
	const size_t nW = veh.getNumWheels();

	auto lck = std::lock_guard(mtx_);

	commands_.resize(nW, 0.0);
	pids_.resize(nW);
	lastTorques_.assign(nW, 0.0);

	for (size_t i = 0; i < nW; i++)
	{
		const auto& wheel = veh.getWheelInfo(i);
		const double cmd = std::isfinite(commands_[i]) ? commands_[i] : 0.0;

		double torque = 0;	// positive = forward (joint convention)
		if (commandMode_ == CommandMode::Effort)
		{
			torque = cmd;
		}
		else
		{
			// Velocity PID, with the error in m/s at the wheel rim:
			const double R = 0.5 * wheel.diameter;
			const double spVel = cmd * R;
			const double actVel = wheel.getW() * R;

			auto& pid = pids_[i];
			pid.KP = KP;
			pid.KI = KI;
			pid.KD = KD;
			pid.max_out = max_torque;

			constexpr double zeroThreshold = 1e-3;	// m/s
			constexpr double stopThreshold = 0.05;	// m/s
			if (std::abs(spVel) < zeroThreshold && std::abs(actVel) < stopThreshold)
			{
				// Full stop, avoiding integral-term creeping:
				pid.reset();
				torque = 0;
			}
			else
			{
				torque = pid.compute(spVel - actVel, dt);
			}
		}

		if (max_torque > 0)
		{
			torque = std::clamp(torque, -max_torque, max_torque);
		}

		// mvsim convention: positive torque makes the vehicle move backwards
		lastTorques_[i] = -torque;
	}
	return lastTorques_;
}

void WheelJointsInterface::updateStates(const VehicleBase& veh)
{
	const size_t nW = veh.getNumWheels();

	auto lck = std::lock_guard(mtx_);
	states_.resize(nW);
	lastTorques_.resize(nW, 0.0);
	for (size_t i = 0; i < nW; i++)
	{
		const auto& wheel = veh.getWheelInfo(i);
		states_[i].position = wheel.getPhiContinuous();
		states_[i].velocity = wheel.getW();
		states_[i].effort = -lastTorques_[i];
	}
}
