/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/format.h>
#include <mvsim/mvsim_ros2_control.h>

#include <algorithm>
#include <cmath>
#include <controller_manager/controller_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <set>
#include <sstream>

using namespace mvsim_node;

namespace
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using mvsim::WheelJointsInterface;

const char* commandInterfaceName(WheelJointsInterface::CommandMode mode)
{
	return mode == WheelJointsInterface::CommandMode::Velocity ? hardware_interface::HW_IF_VELOCITY
															   : hardware_interface::HW_IF_EFFORT;
}

/** The hardware component: the wheels of a simulated vehicle */
class MvsimSystem : public hardware_interface::SystemInterface
{
   public:
	MvsimSystem(WheelJointsInterface& joints, std::vector<size_t> wheelIndices)
		: joints_(joints), wheelIndices_(std::move(wheelIndices))
	{
	}

	CallbackReturn on_init(
		const hardware_interface::HardwareComponentInterfaceParams& params) override
	{
		if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
		{
			return CallbackReturn::ERROR;
		}
		const std::string cmdIf = commandInterfaceName(joints_.commandMode());
		for (const auto& j : info_.joints)
		{
			for (const auto& ci : j.command_interfaces)
			{
				if (ci.name != cmdIf)
				{
					RCLCPP_ERROR(
						get_logger(),
						"Joint '%s': command interface '%s' does not match the MVSim vehicle "
						"<command_interface> ('%s')",
						j.name.c_str(), ci.name.c_str(), cmdIf.c_str());
					return CallbackReturn::ERROR;
				}
			}
			for (const auto& si : j.state_interfaces)
			{
				if (si.name != hardware_interface::HW_IF_POSITION &&
					si.name != hardware_interface::HW_IF_VELOCITY &&
					si.name != hardware_interface::HW_IF_EFFORT)
				{
					RCLCPP_ERROR(
						get_logger(), "Joint '%s': unsupported state interface '%s'",
						j.name.c_str(), si.name.c_str());
					return CallbackReturn::ERROR;
				}
			}
		}
		return CallbackReturn::SUCCESS;
	}

	hardware_interface::return_type read(
		[[maybe_unused]] const rclcpp::Time& time,
		[[maybe_unused]] const rclcpp::Duration& period) override
	{
		const auto states = joints_.getStates();
		for (size_t k = 0; k < info_.joints.size(); k++)
		{
			const size_t wi = wheelIndices_[k];
			if (wi >= states.size())
			{
				continue;  // no simulation step yet
			}
			const auto& name = info_.joints[k].name;
			const auto& st = states[wi];
			setIfExists(name + "/" + hardware_interface::HW_IF_POSITION, st.position);
			setIfExists(name + "/" + hardware_interface::HW_IF_VELOCITY, st.velocity);
			setIfExists(name + "/" + hardware_interface::HW_IF_EFFORT, st.effort);
		}
		return hardware_interface::return_type::OK;
	}

	hardware_interface::return_type write(
		[[maybe_unused]] const rclcpp::Time& time,
		[[maybe_unused]] const rclcpp::Duration& period) override
	{
		// Only the wheels of this system, since others may belong to other
		// systems:
		const std::string cmdIf = commandInterfaceName(joints_.commandMode());
		for (size_t k = 0; k < info_.joints.size(); k++)
		{
			const auto ifName = info_.joints[k].name + "/" + cmdIf;
			const double c = has_command(ifName) ? get_command<double>(ifName) : 0.0;
			joints_.setCommand(wheelIndices_[k], std::isfinite(c) ? c : 0.0);
		}
		return hardware_interface::return_type::OK;
	}

   private:
	WheelJointsInterface& joints_;
	const std::vector<size_t> wheelIndices_;  //!< joint index -> wheel index

	void setIfExists(const std::string& name, double value)
	{
		if (has_state(name))
		{
			set_state(name, value);
		}
	}
};

/** A resource manager that binds <ros2_control> systems to the vehicle
 * wheels (by joint name), whatever hardware plugin they specify. */
class MvsimResourceManager : public hardware_interface::ResourceManager
{
   public:
	MvsimResourceManager(
		rclcpp::Clock::SharedPtr clock, rclcpp::Logger logger, mvsim::VehicleBase& veh,
		WheelJointsInterface& joints, rclcpp::Executor::WeakPtr executor)
		: ResourceManager(clock, logger), veh_(veh), joints_(joints), executor_(executor)
	{
	}

	bool load_and_initialize_components(
		const hardware_interface::ResourceManagerParams& params) override
	{
		std::vector<hardware_interface::HardwareInfo> infos;
		try
		{
			infos = hardware_interface::parse_control_resources_from_urdf(params.robot_description);
		}
		catch (const std::exception& e)
		{
			RCLCPP_ERROR(get_logger(), "Error parsing the robot description: %s", e.what());
			components_are_loaded_and_initialized_ = false;
			return false;
		}

		size_t numBound = 0;
		std::set<size_t> boundWheels;  // each wheel can be in one system only
		for (const auto& info : infos)
		{
			std::vector<size_t> wheelIndices;
			std::string missing;
			std::string repeated;
			for (const auto& j : info.joints)
			{
				const auto idx = findWheel(j.name);
				if (!idx)
				{
					missing += (missing.empty() ? "" : ", ") + j.name;
					continue;
				}
				if (boundWheels.count(*idx) ||
					std::find(wheelIndices.begin(), wheelIndices.end(), *idx) != wheelIndices.end())
				{
					repeated += (repeated.empty() ? "" : ", ") + j.name;
				}
				wheelIndices.push_back(*idx);
			}
			if (info.type != "system" || info.joints.empty() || !missing.empty() ||
				!repeated.empty())
			{
				RCLCPP_WARN(
					get_logger(),
					"Ignoring <ros2_control> '%s' (type '%s'): only systems whose joints are "
					"all wheels of vehicle '%s', each in one system only, are supported. Unknown "
					"joints: [%s]. Repeated joints: [%s]",
					info.name.c_str(), info.type.c_str(), veh_.getName().c_str(), missing.c_str(),
					repeated.c_str());
				continue;
			}
			boundWheels.insert(wheelIndices.begin(), wheelIndices.end());

			auto system = std::make_unique<MvsimSystem>(joints_, wheelIndices);
			hardware_interface::HardwareComponentParams hp;
			hp.hardware_info = info;
			hp.clock = get_clock();
			hp.logger = get_logger();
			hp.executor = executor_;
			import_component(std::move(system), hp);
			numBound++;

			RCLCPP_INFO(
				get_logger(), "Bound <ros2_control> '%s' (%zu joints) to MVSim vehicle '%s'",
				info.name.c_str(), info.joints.size(), veh_.getName().c_str());
		}
		if (numBound == 0)
		{
			RCLCPP_ERROR(
				get_logger(), "No <ros2_control> system could be bound to vehicle '%s'",
				veh_.getName().c_str());
		}
		components_are_loaded_and_initialized_ = numBound > 0;
		return components_are_loaded_and_initialized_;
	}

   private:
	mvsim::VehicleBase& veh_;
	WheelJointsInterface& joints_;
	rclcpp::Executor::WeakPtr executor_;

	std::optional<size_t> findWheel(const std::string& jointName) const
	{
		for (size_t i = 0; i < veh_.getNumWheels(); i++)
		{
			if (veh_.getWheelInfo(i).joint_name == jointName)
			{
				return i;
			}
		}
		return std::nullopt;
	}
};

/** Link name for a wheel joint: "<name>_joint" -> "<name>", else "<name>_link" */
std::string wheelLinkName(const std::string& jointName)
{
	const std::string suffix = "_joint";
	if (jointName.size() > suffix.size() &&
		jointName.compare(jointName.size() - suffix.size(), suffix.size(), suffix) == 0)
	{
		return jointName.substr(0, jointName.size() - suffix.size());
	}
	return jointName + "_link";
}

}  // namespace

std::string mvsim_node::generateVehicleUrdf(
	const mvsim::VehicleBase& veh, mvsim::WheelJointsInterface::CommandMode mode)
{
	std::stringstream ss;
	ss << "<?xml version=\"1.0\"?>\n";
	ss << "<robot name=\"" << veh.getName() << "\">\n";
	ss << "  <link name=\"base_link\"/>\n";
	for (size_t i = 0; i < veh.getNumWheels(); i++)
	{
		const auto& w = veh.getWheelInfo(i);
		const auto link = wheelLinkName(w.joint_name);
		ss << "  <link name=\"" << link << "\"/>\n";
		ss << "  <joint name=\"" << w.joint_name << "\" type=\"continuous\">\n";
		ss << "    <parent link=\"base_link\"/>\n";
		ss << "    <child link=\"" << link << "\"/>\n";
		ss << mrpt::format(
			"    <origin xyz=\"%f %f %f\" rpy=\"0 0 %f\"/>\n", w.x, w.y, 0.5 * w.diameter, w.yaw);
		ss << "    <axis xyz=\"0 1 0\"/>\n";
		ss << "  </joint>\n";
	}
	ss << "  <ros2_control name=\"mvsim_" << veh.getName() << "\" type=\"system\">\n";
	ss << "    <hardware><plugin>mvsim/MvsimSystem</plugin></hardware>\n";
	for (size_t i = 0; i < veh.getNumWheels(); i++)
	{
		ss << "    <joint name=\"" << veh.getWheelInfo(i).joint_name << "\">\n";
		ss << "      <command_interface name=\"" << commandInterfaceName(mode) << "\"/>\n";
		ss << "      <state_interface name=\"position\"/>\n";
		ss << "      <state_interface name=\"velocity\"/>\n";
		ss << "      <state_interface name=\"effort\"/>\n";
		ss << "    </joint>\n";
	}
	ss << "  </ros2_control>\n";
	ss << "</robot>\n";
	return ss.str();
}

Ros2ControlVehicle::Ros2ControlVehicle(
	mvsim::World& world, mvsim::VehicleBase& veh, mvsim::WheelJointsInterface& joints,
	const std::string& ns)
	: world_(world), veh_(veh), joints_(joints)
{
	executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

	// Controller manager node options: sim time and user parameters file
	auto options = controller_manager::get_cm_node_options();
	std::vector<std::string> args = {"--ros-args"};
	if (!joints_.controllersYaml().empty())
	{
		args.push_back("--params-file");
		args.push_back(joints_.controllersYaml());
	}
	options.arguments(args);
	// Do not apply the mvsim node remappings (e.g. its node name) to this node:
	options.use_global_arguments(false);
	options.append_parameter_override("use_sim_time", true);

	// The resource manager logs and timing use the (sim) clock of the CM node,
	// which does not exist yet: use a standalone ROS clock instead.
	auto rm = std::make_unique<MvsimResourceManager>(
		std::make_shared<rclcpp::Clock>(RCL_ROS_TIME),
		rclcpp::get_logger("mvsim_resource_manager." + veh.getName()), veh, joints, executor_);

	cm_ = std::make_shared<controller_manager::ControllerManager>(
		std::move(rm), executor_, "controller_manager", ns, options);
	executor_->add_node(cm_);

	// Let each controller declared in the YAML file (as "<name>.type") read
	// its parameters from the same file, unless already set:
	if (!joints_.controllersYaml().empty())
	{
		const std::string typeSuffix = ".type";
		for (const auto& name : cm_->list_parameters({}, 0).names)
		{
			if (name.size() <= typeSuffix.size() ||
				name.compare(name.size() - typeSuffix.size(), typeSuffix.size(), typeSuffix) != 0)
			{
				continue;
			}
			const auto paramsFile =
				name.substr(0, name.size() - typeSuffix.size()) + ".params_file";
			if (!cm_->has_parameter(paramsFile))
			{
				cm_->declare_parameter(paramsFile, joints_.controllersYaml());
			}
		}
	}

	const unsigned int rate = cm_->get_update_rate();
	updatePeriod_ = 1.0 / std::max(1U, rate);
	const double dt = world_.get_simul_timestep();
	stepsPerUpdate_ = std::max(1U, static_cast<unsigned int>(std::lround(updatePeriod_ / dt)));
	if (std::abs(stepsPerUpdate_ * dt - updatePeriod_) > 1e-6)
	{
		RCLCPP_WARN(
			cm_->get_logger(),
			"Controller manager update period (%.4f s) is not a multiple of the "
			"simulation time step (%.4f s): using %u steps (%.4f s)",
			updatePeriod_, dt, stepsPerUpdate_, stepsPerUpdate_ * dt);
	}
	updatePeriod_ = stepsPerUpdate_ * dt;

	// Robot description:
	if (!joints_.robotDescriptionFromTopic())
	{
		descriptionNode_ = std::make_shared<rclcpp::Node>(
			"mvsim_robot_description", ns, rclcpp::NodeOptions().use_global_arguments(false));
		pubDescription_ = descriptionNode_->create_publisher<std_msgs::msg::String>(
			"robot_description", rclcpp::QoS(1).transient_local().reliable());
		std_msgs::msg::String msg;
		msg.data = generateVehicleUrdf(veh_, joints_.commandMode());
		pubDescription_->publish(msg);
		executor_->add_node(descriptionNode_);
	}

	executorThread_ = std::thread([this]() { executor_->spin(); });

	postStepCallbackId_ =
		world_.addPostStepCallback([this](double simTime) { onSimulationStep(simTime); });

	RCLCPP_INFO(
		cm_->get_logger(),
		"ros2_control for vehicle '%s': controller manager at %.1f Hz (every %u simulation "
		"steps), robot description from %s",
		veh_.getName().c_str(), 1.0 / updatePeriod_, stepsPerUpdate_,
		joints_.robotDescriptionFromTopic() ? "topic" : "the vehicle (generated)");
}

Ros2ControlVehicle::~Ros2ControlVehicle()
{
	// First, make sure the simulation thread does not call us anymore:
	world_.removePostStepCallback(postStepCallbackId_);

	if (executor_)
	{
		executor_->cancel();
	}
	if (executorThread_.joinable())
	{
		executorThread_.join();
	}
	// Note: the controller manager shuts down its controllers by itself.
}

void Ros2ControlVehicle::onSimulationStep([[maybe_unused]] double simTime)
{
	if (++stepCount_ < stepsPerUpdate_)
	{
		return;
	}
	stepCount_ = 0;

	if (!cm_->is_resource_manager_initialized())
	{
		return;
	}

	// Simulation time, as ROS time (the clock of all nodes using sim time):
	const auto t = rclcpp::Time(
		static_cast<int64_t>(
			std::llround(mrpt::Clock::toDouble(world_.get_simul_timestamp()) * 1e9)),
		RCL_ROS_TIME);
	const auto period = rclcpp::Duration::from_seconds(updatePeriod_);

	cm_->read(t, period);
	cm_->update(t, period);
	cm_->write(t, period);
}
