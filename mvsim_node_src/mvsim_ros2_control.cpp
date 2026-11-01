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

#if defined(MVSIM_ROS2_CONTROL_LEGACY_API)
#include <array>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#endif

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

#if defined(MVSIM_ROS2_CONTROL_LEGACY_API)
	CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override
	{
		if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
		{
			return CallbackReturn::ERROR;
		}
		states_.assign(info_.joints.size(), {0, 0, 0});
		// NaN: no command yet, as ros2_control does in newer versions
		commands_.assign(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
		return checkInterfaces();
	}

	std::vector<hardware_interface::StateInterface> export_state_interfaces() override
	{
		std::vector<hardware_interface::StateInterface> ifs;
		for (size_t k = 0; k < info_.joints.size(); k++)
		{
			for (const auto& si : info_.joints[k].state_interfaces)
			{
				ifs.emplace_back(info_.joints[k].name, si.name, &states_[k][stateIndex(si.name)]);
			}
		}
		return ifs;
	}

	std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
	{
		std::vector<hardware_interface::CommandInterface> ifs;
		for (size_t k = 0; k < info_.joints.size(); k++)
		{
			for (const auto& ci : info_.joints[k].command_interfaces)
			{
				ifs.emplace_back(info_.joints[k].name, ci.name, &commands_[k]);
			}
		}
		return ifs;
	}
#else
	CallbackReturn on_init(
		const hardware_interface::HardwareComponentInterfaceParams& params) override
	{
		if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
		{
			return CallbackReturn::ERROR;
		}
		return checkInterfaces();
	}
#endif

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
			const auto& st = states[wi];
			setState(k, hardware_interface::HW_IF_POSITION, st.position);
			setState(k, hardware_interface::HW_IF_VELOCITY, st.velocity);
			setState(k, hardware_interface::HW_IF_EFFORT, st.effort);
		}
		return hardware_interface::return_type::OK;
	}

	hardware_interface::return_type write(
		[[maybe_unused]] const rclcpp::Time& time,
		[[maybe_unused]] const rclcpp::Duration& period) override
	{
		// Only the wheels of this system, since others may belong to other
		// systems:
		for (size_t k = 0; k < info_.joints.size(); k++)
		{
			// NaN (e.g. not claimed by any controller) leaves the wheel free:
			joints_.setCommand(wheelIndices_[k], getCommand(k));
		}
		return hardware_interface::return_type::OK;
	}

   private:
	WheelJointsInterface& joints_;
	const std::vector<size_t> wheelIndices_;  //!< joint index -> wheel index

	CallbackReturn checkInterfaces()
	{
		const std::string cmdIf = commandInterfaceName(joints_.commandMode());
		for (const auto& j : info_.joints)
		{
			for (const auto& ci : j.command_interfaces)
			{
				if (ci.name != cmdIf)
				{
					RCLCPP_ERROR(
						logger(),
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
						logger(), "Joint '%s': unsupported state interface '%s'", j.name.c_str(),
						si.name.c_str());
					return CallbackReturn::ERROR;
				}
			}
		}
		return CallbackReturn::SUCCESS;
	}

#if defined(MVSIM_ROS2_CONTROL_LEGACY_API)
	// The interfaces point to these values (position, velocity, effort):
	std::vector<std::array<double, 3>> states_;
	std::vector<double> commands_;

	static size_t stateIndex(const std::string& name)
	{
		if (name == hardware_interface::HW_IF_POSITION)
		{
			return 0;
		}
		return name == hardware_interface::HW_IF_VELOCITY ? 1 : 2;
	}

	rclcpp::Logger logger() const { return rclcpp::get_logger("mvsim_system." + info_.name); }

	void setState(size_t joint, const std::string& name, double value)
	{
		states_[joint][stateIndex(name)] = value;
	}

	double getCommand(size_t joint) const { return commands_[joint]; }
#else
	rclcpp::Logger logger() const { return get_logger(); }

	void setState(size_t joint, const std::string& name, double value)
	{
		const auto ifName = info_.joints[joint].name + "/" + name;
		if (has_state(ifName))
		{
			set_state(ifName, value);
		}
	}

	double getCommand(size_t joint) const
	{
		const auto ifName =
			info_.joints[joint].name + "/" + commandInterfaceName(joints_.commandMode());
		return has_command(ifName) ? get_command<double>(ifName) : 0.0;
	}
#endif
};

/** A <ros2_control> system of the robot description, and the vehicle wheel of
 * each of its joints */
struct BoundSystem
{
	hardware_interface::HardwareInfo info;
	std::vector<size_t> wheelIndices;
};

std::optional<size_t> findWheel(const mvsim::VehicleBase& veh, const std::string& jointName)
{
	for (size_t i = 0; i < veh.getNumWheels(); i++)
	{
		if (veh.getWheelInfo(i).joint_name == jointName)
		{
			return i;
		}
	}
	return std::nullopt;
}

/** The <ros2_control> systems whose joints are all wheels of the vehicle,
 * each wheel in one system only. The rest are ignored, with a warning. */
std::vector<BoundSystem> bindSystemsToWheels(
	const std::string& robotDescription, const mvsim::VehicleBase& veh,
	const rclcpp::Logger& logger)
{
	std::vector<hardware_interface::HardwareInfo> infos;
	try
	{
		infos = hardware_interface::parse_control_resources_from_urdf(robotDescription);
	}
	catch (const std::exception& e)
	{
		RCLCPP_ERROR(logger, "Error parsing the robot description: %s", e.what());
		return {};
	}

	std::vector<BoundSystem> bound;
	std::set<size_t> boundWheels;  // each wheel can be in one system only
	for (const auto& info : infos)
	{
		std::vector<size_t> wheelIndices;
		std::string missing;
		std::string repeated;
		for (const auto& j : info.joints)
		{
			const auto idx = findWheel(veh, j.name);
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
		if (info.type != "system" || info.joints.empty() || !missing.empty() || !repeated.empty())
		{
			RCLCPP_WARN(
				logger,
				"Ignoring <ros2_control> '%s' (type '%s'): only systems whose joints are "
				"all wheels of vehicle '%s', each in one system only, are supported. Unknown "
				"joints: [%s]. Repeated joints: [%s]",
				info.name.c_str(), info.type.c_str(), veh.getName().c_str(), missing.c_str(),
				repeated.c_str());
			continue;
		}
		boundWheels.insert(wheelIndices.begin(), wheelIndices.end());
		bound.push_back({info, wheelIndices});

		RCLCPP_INFO(
			logger, "Bound <ros2_control> '%s' (%zu joints) to MVSim vehicle '%s'",
			info.name.c_str(), info.joints.size(), veh.getName().c_str());
	}
	if (bound.empty())
	{
		RCLCPP_ERROR(
			logger, "No <ros2_control> system could be bound to vehicle '%s'",
			veh.getName().c_str());
	}
	return bound;
}

#if !defined(MVSIM_ROS2_CONTROL_LEGACY_API)

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
		const auto bound = bindSystemsToWheels(params.robot_description, veh_, get_logger());
		for (const auto& b : bound)
		{
			hardware_interface::HardwareComponentParams hp;
			hp.hardware_info = b.info;
			hp.clock = get_clock();
			hp.logger = get_logger();
			hp.executor = executor_;
			import_component(std::make_unique<MvsimSystem>(joints_, b.wheelIndices), hp);
		}
		components_are_loaded_and_initialized_ = !bound.empty();
		return components_are_loaded_and_initialized_;
	}

   private:
	mvsim::VehicleBase& veh_;
	WheelJointsInterface& joints_;
	rclcpp::Executor::WeakPtr executor_;
};
#endif

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
	: world_(world), veh_(veh), joints_(joints), ns_(ns)
{
	executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

	descriptionNode_ = std::make_shared<rclcpp::Node>(
		"mvsim_robot_description", ns, rclcpp::NodeOptions().use_global_arguments(false));
	const auto qosDescription = rclcpp::QoS(1).transient_local().reliable();

#if defined(MVSIM_ROS2_CONTROL_LEGACY_API)
	// This resource manager version cannot bind the systems by itself: do it
	// here when the robot description arrives, then create the controller
	// manager with it.
	subDescription_ = descriptionNode_->create_subscription<std_msgs::msg::String>(
		"robot_description", qosDescription,
		[this](const std_msgs::msg::String& msg) { onRobotDescription(msg.data); });
#else
	// The resource manager logs and timing use the (sim) clock of the CM node,
	// which does not exist yet: use a standalone ROS clock instead.
	createControllerManager(std::make_unique<MvsimResourceManager>(
		std::make_shared<rclcpp::Clock>(RCL_ROS_TIME),
		rclcpp::get_logger("mvsim_resource_manager." + veh.getName()), veh, joints, executor_));
#endif

	// Robot description generated from the vehicle:
	if (!joints_.robotDescriptionFromTopic())
	{
		pubDescription_ = descriptionNode_->create_publisher<std_msgs::msg::String>(
			"robot_description", qosDescription);
		std_msgs::msg::String msg;
		msg.data = generateVehicleUrdf(veh_, joints_.commandMode());
		pubDescription_->publish(msg);
	}
	executor_->add_node(descriptionNode_);

	executorThread_ = std::thread([this]() { executor_->spin(); });

	postStepCallbackId_ =
		world_.addPostStepCallback([this](double simTime) { onSimulationStep(simTime); });

	RCLCPP_INFO(
		descriptionNode_->get_logger(), "ros2_control for vehicle '%s': robot description from %s",
		veh_.getName().c_str(),
		joints_.robotDescriptionFromTopic() ? "topic" : "the vehicle (generated)");
}

Ros2ControlVehicle::~Ros2ControlVehicle()
{
	detach();
	// Note: the controller manager shuts down its controllers by itself.
}

void Ros2ControlVehicle::detach()
{
	if (detached_)
	{
		return;
	}
	detached_ = true;

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
}

void Ros2ControlVehicle::createControllerManager(
	std::unique_ptr<hardware_interface::ResourceManager> rm)
{
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

	auto cm = std::make_shared<controller_manager::ControllerManager>(
		std::move(rm), executor_, "controller_manager", ns_, options);
	executor_->add_node(cm);

	// Let each controller declared in the YAML file (as "<name>.type") read
	// its parameters from the same file, unless already set:
	if (!joints_.controllersYaml().empty())
	{
		const std::string typeSuffix = ".type";
		for (const auto& name : cm->list_parameters({}, 0).names)
		{
			if (name.size() <= typeSuffix.size() ||
				name.compare(name.size() - typeSuffix.size(), typeSuffix.size(), typeSuffix) != 0)
			{
				continue;
			}
			const auto paramsFile =
				name.substr(0, name.size() - typeSuffix.size()) + ".params_file";
			if (!cm->has_parameter(paramsFile))
			{
				cm->declare_parameter(paramsFile, joints_.controllersYaml());
			}
		}
	}

	const unsigned int rate = cm->get_update_rate();
	const double period = 1.0 / std::max(1U, rate);
	const double dt = world_.get_simul_timestep();
	const unsigned int steps = std::max(1U, static_cast<unsigned int>(std::lround(period / dt)));
	if (std::abs(steps * dt - period) > 1e-6)
	{
		RCLCPP_WARN(
			cm->get_logger(),
			"Controller manager update period (%.4f s) is not a multiple of the "
			"simulation time step (%.4f s): using %u steps (%.4f s)",
			period, dt, steps, steps * dt);
	}

	RCLCPP_INFO(
		cm->get_logger(),
		"ros2_control for vehicle '%s': controller manager at %.1f Hz (every %u simulation "
		"steps)",
		veh_.getName().c_str(), 1.0 / (steps * dt), steps);

	std::lock_guard<std::mutex> lck(cmMtx_);
	stepsPerUpdate_ = steps;
	updatePeriod_ = steps * dt;
	cm_ = cm;
}

#if defined(MVSIM_ROS2_CONTROL_LEGACY_API)
void Ros2ControlVehicle::onRobotDescription(const std::string& robotDescription)
{
	const auto logger = rclcpp::get_logger("mvsim_resource_manager." + veh_.getName());
	{
		std::lock_guard<std::mutex> lck(cmMtx_);
		if (cm_)
		{
			RCLCPP_WARN(logger, "Robot description already loaded: ignoring a new one");
			return;
		}
	}

	const auto bound = bindSystemsToWheels(robotDescription, veh_, logger);
	if (bound.empty())
	{
		return;
	}

	// This runs in the executor thread, which must not end with an exception:
	try
	{
		auto rm = std::make_unique<hardware_interface::ResourceManager>();
		for (const auto& b : bound)
		{
			rm->import_component(std::make_unique<MvsimSystem>(joints_, b.wheelIndices), b.info);
		}
		// Only marks the description as loaded (its systems were imported above):
		rm->load_urdf(robotDescription, false /*validate*/, false /*load components*/);

		for (const auto& entry : rm->get_components_status())
		{
			rclcpp_lifecycle::State active(
				lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
				hardware_interface::lifecycle_state_names::ACTIVE);
			rm->set_component_state(entry.first, active);
		}

		createControllerManager(std::move(rm));
	}
	catch (const std::exception& e)
	{
		RCLCPP_ERROR(
			logger, "Could not set up ros2_control for vehicle '%s': %s", veh_.getName().c_str(),
			e.what());
	}
}
#endif

void Ros2ControlVehicle::onSimulationStep([[maybe_unused]] double simTime)
{
	std::lock_guard<std::mutex> lck(cmMtx_);
	if (!cm_)
	{
		return;	 // no robot description yet
	}
	if (++stepCount_ < stepsPerUpdate_)
	{
		return;
	}
	stepCount_ = 0;

#if !defined(MVSIM_ROS2_CONTROL_LEGACY_API)
	if (!cm_->is_resource_manager_initialized())
	{
		return;
	}
#endif

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
