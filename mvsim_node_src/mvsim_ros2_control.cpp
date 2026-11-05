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
#include <rcl/remap.h>

#include <array>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#endif

using namespace mvsim_node;

namespace
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using mvsim::WheelJointsInterface;
using JointCommand = WheelJointsInterface::JointCommand;
using JointKind = WheelJointsInterface::JointKind;

/** Command interface name of a joint ("" if state only) */
std::string commandInterfaceName(JointCommand c)
{
	switch (c)
	{
		case JointCommand::Velocity:
			return hardware_interface::HW_IF_VELOCITY;
		case JointCommand::Effort:
			return hardware_interface::HW_IF_EFFORT;
		case JointCommand::Position:
			return hardware_interface::HW_IF_POSITION;
		default:
			return {};
	}
}

/** The hardware component: some joints (wheels, steering) of a simulated
 * vehicle */
class MvsimSystem : public hardware_interface::SystemInterface
{
   public:
	MvsimSystem(WheelJointsInterface& joints, std::vector<size_t> jointIndices)
		: joints_(joints), jointIndices_(std::move(jointIndices))
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
			const size_t ji = jointIndices_[k];
			if (ji >= states.size())
			{
				continue;  // no simulation step yet
			}
			const auto& st = states[ji];
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
		// Only the joints of this system, since others may belong to other
		// systems:
		for (size_t k = 0; k < info_.joints.size(); k++)
		{
			// NaN (e.g. not claimed by any controller, or no command
			// interface) leaves a wheel free, or the steering as is:
			joints_.setCommand(jointIndices_[k], getCommand(k));
		}
		return hardware_interface::return_type::OK;
	}

   private:
	WheelJointsInterface& joints_;
	/// System joint index -> vehicle joint index (WheelJointsInterface::joints())
	const std::vector<size_t> jointIndices_;

	const WheelJointsInterface::Joint& vehicleJoint(size_t joint) const
	{
		return joints_.joints().at(jointIndices_.at(joint));
	}

	CallbackReturn checkInterfaces()
	{
		for (size_t k = 0; k < info_.joints.size(); k++)
		{
			const auto& j = info_.joints[k];
			const std::string cmdIf = commandInterfaceName(vehicleJoint(k).command);
			for (const auto& ci : j.command_interfaces)
			{
				if (ci.name != cmdIf)
				{
					RCLCPP_ERROR(
						logger(),
						"Joint '%s': command interface '%s' not supported by MVSim (expected: "
						"'%s'; see <command_interface> and <steering_joints> in the vehicle "
						"<controller>)",
						j.name.c_str(), ci.name.c_str(),
						cmdIf.empty() ? "none, state only" : cmdIf.c_str());
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
			info_.joints[joint].name + "/" + commandInterfaceName(vehicleJoint(joint).command);
		return has_command(ifName) ? get_command<double>(ifName)
								   : std::numeric_limits<double>::quiet_NaN();
	}
#endif
};

#if defined(MVSIM_ROS2_CONTROL_LEGACY_API)
/** Whether the process has a global node name remap (e.g. "name" in a launch
 * file). In this ros2_control version, it also renames the controller nodes,
 * which then do not get their parameters. */
bool hasGlobalNodeNameRemap()
{
	auto ctx = rclcpp::contexts::get_global_default_context()->get_rcl_context();
	const auto alloc = rcl_get_default_allocator();
	char* newName = nullptr;
	if (rcl_remap_node_name(nullptr, &ctx->global_arguments, "controller", alloc, &newName) !=
		RCL_RET_OK)
	{
		return false;
	}
	const bool remapped = newName != nullptr;
	if (newName)
	{
		alloc.deallocate(newName, alloc.state);
	}
	return remapped;
}
#endif

/** A <ros2_control> system of the robot description, and the vehicle joint
 * of each of its joints */
struct BoundSystem
{
	hardware_interface::HardwareInfo info;
	std::vector<size_t> jointIndices;
};

std::optional<size_t> findJoint(const WheelJointsInterface& joints, const std::string& jointName)
{
	const auto& js = joints.joints();
	for (size_t i = 0; i < js.size(); i++)
	{
		if (js[i].name == jointName)
		{
			return i;
		}
	}
	return std::nullopt;
}

/** The <ros2_control> systems whose joints are all joints of the vehicle,
 * each joint in one system only. The rest are ignored, with a warning. */
std::vector<BoundSystem> bindSystemsToJoints(
	const std::string& robotDescription, const mvsim::VehicleBase& veh,
	const WheelJointsInterface& joints, const rclcpp::Logger& logger)
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
	std::set<size_t> boundJoints;  // each joint can be in one system only
	for (const auto& info : infos)
	{
		std::vector<size_t> jointIndices;
		std::string missing;
		std::string repeated;
		for (const auto& j : info.joints)
		{
			const auto idx = findJoint(joints, j.name);
			if (!idx)
			{
				missing += (missing.empty() ? "" : ", ") + j.name;
				continue;
			}
			if (boundJoints.count(*idx) ||
				std::find(jointIndices.begin(), jointIndices.end(), *idx) != jointIndices.end())
			{
				repeated += (repeated.empty() ? "" : ", ") + j.name;
			}
			jointIndices.push_back(*idx);
		}
		if (info.type != "system" || info.joints.empty() || !missing.empty() || !repeated.empty())
		{
			RCLCPP_WARN(
				logger,
				"Ignoring <ros2_control> '%s' (type '%s'): only systems whose joints are "
				"all joints of vehicle '%s', each in one system only, are supported. Unknown "
				"joints: [%s]. Repeated joints: [%s]",
				info.name.c_str(), info.type.c_str(), veh.getName().c_str(), missing.c_str(),
				repeated.c_str());
			continue;
		}
		boundJoints.insert(jointIndices.begin(), jointIndices.end());
		bound.push_back({info, jointIndices});

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
 * joints (by name), whatever hardware plugin they specify. */
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
		const auto bound =
			bindSystemsToJoints(params.robot_description, veh_, joints_, get_logger());
		for (const auto& b : bound)
		{
			hardware_interface::HardwareComponentParams hp;
			hp.hardware_info = b.info;
			hp.clock = get_clock();
			hp.logger = get_logger();
			hp.executor = executor_;
			import_component(std::make_unique<MvsimSystem>(joints_, b.jointIndices), hp);
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

/** Link name for a joint: "<name>_joint" -> "<name>", else "<name>_link" */
std::string jointLinkName(const std::string& jointName)
{
	const std::string suffix = "_joint";
	if (jointName.size() > suffix.size() &&
		jointName.compare(jointName.size() - suffix.size(), suffix.size(), suffix) == 0)
	{
		return jointName.substr(0, jointName.size() - suffix.size());
	}
	return jointName + "_link";
}

/** A URDF joint and its child link */
void writeUrdfJoint(
	std::stringstream& ss, const std::string& name, const std::string& link,
	const std::string& parent, const mrpt::math::TPoint3D& xyz, double yaw,
	const WheelJointsInterface::Joint* steering)
{
	ss << "  <link name=\"" << link << "\"/>\n";
	ss << "  <joint name=\"" << name << "\" type=\"" << (steering ? "revolute" : "continuous")
	   << "\">\n";
	ss << "    <parent link=\"" << parent << "\"/>\n";
	ss << "    <child link=\"" << link << "\"/>\n";
	ss << mrpt::format("    <origin xyz=\"%f %f %f\" rpy=\"0 0 %f\"/>\n", xyz.x, xyz.y, xyz.z, yaw);
	ss << "    <axis xyz=\"" << (steering ? "0 0 1" : "0 1 0") << "\"/>\n";
	if (steering)
	{
		// The steering rate limit (none: a high one, as the URDF needs one).
		// The effort limit is not used by position commands.
		const double maxRate = steering->velocityLimit > 0 ? steering->velocityLimit : 100.0;
		ss << mrpt::format(
			"    <limit lower=\"%f\" upper=\"%f\" effort=\"1000\" velocity=\"%f\"/>\n",
			steering->positionMin, steering->positionMax, maxRate);
	}
	ss << "  </joint>\n";
}

}  // namespace

std::string mvsim_node::generateVehicleUrdf(
	const mvsim::VehicleBase& veh, const mvsim::WheelJointsInterface& joints)
{
	const auto& js = joints.joints();
	const auto wheelPos = [&veh](size_t i)
	{
		const auto& w = veh.getWheelInfo(i);
		return mrpt::math::TPoint3D(w.x, w.y, 0.5 * w.diameter);
	};

	const std::string& base = veh.getBaseFrameId();

	// The child link of each joint, with unique names:
	std::set<std::string> usedLinks = {base};
	std::vector<std::string> links;
	for (const auto& j : js)
	{
		std::string link = jointLinkName(j.name);
		while (!usedLinks.insert(link).second)
		{
			link += "_link";
		}
		links.push_back(link);
	}

	std::stringstream ss;
	ss << "<?xml version=\"1.0\"?>\n";
	ss << "<robot name=\"" << veh.getName() << "\">\n";
	ss << "  <link name=\"" << base << "\"/>\n";
	for (size_t i = 0; i < js.size(); i++)
	{
		const auto& j = js[i];
		switch (j.kind)
		{
			case JointKind::WheelSpin:
			{
				// Steered wheels hang from their steering joint link:
				const auto steer = joints.findJoint(JointKind::WheelSteering, j.wheel);
				if (steer)
				{
					writeUrdfJoint(ss, j.name, links[i], links[*steer], {0, 0, 0}, 0, nullptr);
				}
				else
				{
					writeUrdfJoint(
						ss, j.name, links[i], base, wheelPos(j.wheel),
						veh.getWheelInfo(j.wheel).yaw, nullptr);
				}
			}
			break;
			case JointKind::WheelSteering:
				writeUrdfJoint(ss, j.name, links[i], base, wheelPos(j.wheel), 0, &j);
				break;
			case JointKind::CentralSteering:
			case JointKind::CentralTraction:
			{
				// Virtual joints of the bicycle model, at the center of the
				// front (steering) or rear (traction) axle. Ackermann vehicles
				// have the front wheels last:
				const size_t n = veh.getNumWheels();
				const bool front = j.kind == JointKind::CentralSteering;
				const size_t i0 = front ? n - 2 : 0;
				const auto pos = (wheelPos(i0) + wheelPos(i0 + 1)) * 0.5;
				writeUrdfJoint(ss, j.name, links[i], base, pos, 0, front ? &j : nullptr);
			}
			break;
		};
	}
	ss << "  <ros2_control name=\"mvsim_" << veh.getName() << "\" type=\"system\">\n";
	ss << "    <hardware><plugin>mvsim/MvsimSystem</plugin></hardware>\n";
	for (const auto& j : js)
	{
		ss << "    <joint name=\"" << j.name << "\">\n";
		if (j.command != JointCommand::None)
		{
			ss << "      <command_interface name=\"" << commandInterfaceName(j.command) << "\"/>\n";
		}
		ss << "      <state_interface name=\"position\"/>\n";
		ss << "      <state_interface name=\"velocity\"/>\n";
		if (j.hasEffortState())
		{
			ss << "      <state_interface name=\"effort\"/>\n";
		}
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
	if (hasGlobalNodeNameRemap())
	{
		RCLCPP_ERROR(
			descriptionNode_->get_logger(),
			"The node name of mvsim_node is set by a global remap (e.g. 'name' in its launch "
			"file Node action). In this ROS 2 version, it renames the ros2_control controller "
			"nodes too, so they will not get their parameters: remove it (the default node "
			"name is 'mvsim').");
	}

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
		msg.data = generateVehicleUrdf(veh_, joints_);
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

	const auto bound = bindSystemsToJoints(robotDescription, veh_, joints_, logger);
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
			rm->import_component(std::make_unique<MvsimSystem>(joints_, b.jointIndices), b.info);
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
