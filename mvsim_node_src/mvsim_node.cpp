/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>	 // kbhit()
#include <mrpt/version.h>
#include <mvsim/Sensors/DepthCameraSensor.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>
#include <mvsim/mvsim_node_core.h>

#if MRPT_VERSION < 0x020f00	 // 2.15.0 support legacy classes
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#endif

#include <cmath>
#include <limits>

#include "rapidxml_utils.hpp"

#if PACKAGE_ROS_VERSION == 1
// ===========================================
//                    ROS 1
// ===========================================
#include <mrpt/ros1bridge/gps.h>
#include <mrpt/ros1bridge/image.h>
#include <mrpt/ros1bridge/imu.h>
#include <mrpt/ros1bridge/laser_scan.h>
#include <mrpt/ros1bridge/map.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/pose.h>
#include <mrpt/ros1bridge/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// usings:
using ros::ok;

using Msg_Header = std_msgs::Header;

using Msg_Pose = geometry_msgs::Pose;
using Msg_TransformStamped = geometry_msgs::TransformStamped;

using Msg_GPS = sensor_msgs::NavSatFix;
using Msg_Image = sensor_msgs::Image;
using Msg_Imu = sensor_msgs::Imu;
using Msg_LaserScan = sensor_msgs::LaserScan;
using Msg_PointCloud2 = sensor_msgs::PointCloud2;

using Msg_Marker = visualization_msgs::Marker;
#else
// ===========================================
//                    ROS 2
// ===========================================
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/ros2bridge/time.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// see: https://github.com/ros2/geometry2/pull/416
#if defined(MVSIM_HAS_TF2_GEOMETRY_MSGS_HPP)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <tf2_ros/qos.hpp>	// DynamicBroadcasterQoS(), etc.

// usings:
using rclcpp::ok;

using Msg_Header = std_msgs::msg::Header;

using Msg_Pose = geometry_msgs::msg::Pose;
using Msg_TransformStamped = geometry_msgs::msg::TransformStamped;

using Msg_GPS = sensor_msgs::msg::NavSatFix;
using Msg_Image = sensor_msgs::msg::Image;
using Msg_Imu = sensor_msgs::msg::Imu;
using Msg_LaserScan = sensor_msgs::msg::LaserScan;
using Msg_PointCloud2 = sensor_msgs::msg::PointCloud2;

using Msg_Marker = visualization_msgs::msg::Marker;
#endif

#if PACKAGE_ROS_VERSION == 1
namespace mrpt2ros = mrpt::ros1bridge;
#else
namespace mrpt2ros = mrpt::ros2bridge;
#endif

#if PACKAGE_ROS_VERSION == 1
#define ROS12_INFO(...) ROS_INFO(__VA_ARGS__)
#define ROS12_WARN_THROTTLE(...) ROS_WARN_THROTTLE(__VA_ARGS__)
#define ROS12_WARN_STREAM_THROTTLE(...) ROS_WARN_STREAM_THROTTLE(__VA_ARGS__)
#define ROS12_ERROR(...) ROS_ERROR(__VA_ARGS__)
#else
#define ROS12_INFO(...) RCLCPP_INFO(n_->get_logger(), __VA_ARGS__)
#define ROS12_WARN_THROTTLE(...) RCLCPP_WARN_THROTTLE(n_->get_logger(), *clock_, __VA_ARGS__)
#define ROS12_WARN_STREAM_THROTTLE(...) \
	RCLCPP_WARN_STREAM_THROTTLE(n_->get_logger(), *clock_, __VA_ARGS__)
#define ROS12_ERROR(...) RCLCPP_ERROR(n_->get_logger(), __VA_ARGS__)
#endif

const double MAX_CMD_VEL_AGE_SECONDS = 1.0;

/*------------------------------------------------------------------------------
 * MVSimNode()
 * Constructor.
 *----------------------------------------------------------------------------*/
#if PACKAGE_ROS_VERSION == 1
MVSimNode::MVSimNode(ros::NodeHandle& n)
#else
MVSimNode::MVSimNode(rclcpp::Node::SharedPtr& n)
#endif
	: n_(n)
{
	// Node parameters:
#if PACKAGE_ROS_VERSION == 1
	double t;
	if (!localn_.getParam("base_watchdog_timeout", t)) t = 0.2;
	base_watchdog_timeout_.fromSec(t);
	localn_.param("realtime_factor", realtime_factor_, 1.0);
	localn_.param("gui_refresh_period", gui_refresh_period_ms_, gui_refresh_period_ms_);
	localn_.param("headless", headless_, headless_);
	localn_.param("period_ms_publish_tf", period_ms_publish_tf_, period_ms_publish_tf_);
	localn_.param("do_fake_localization", do_fake_localization_, do_fake_localization_);
	localn_.param("publish_tf_odom2baselink", publish_tf_odom2baselink_, publish_tf_odom2baselink_);
	localn_.param(
		"force_publish_vehicle_namespace", force_publish_vehicle_namespace_,
		force_publish_vehicle_namespace_);
	localn_.param("disable_sim_time_clock", disable_sim_time_clock_, disable_sim_time_clock_);

	// mvsim is the ROS *time source*: it publishes "/clock" and stamps all
	// outgoing messages with simulation time. The mvsim node itself therefore
	// normally runs with use_sim_time:=false (it drives the clock); it is the
	// downstream nodes that should set use_sim_time:=true. This does not apply
	// when disable_sim_time_clock_ is set, since the node no longer drives the
	// clock in that case.
	if (!disable_sim_time_clock_ && true == n_.param("use_sim_time", false))
	{
		ROS_WARN(
			"use_sim_time=true was set on the mvsim node itself. mvsim is the "
			"/clock time source and normally runs with use_sim_time:=false; set "
			"use_sim_time:=true on downstream nodes instead.");
	}
#else
	clock_ = n_->get_clock();
	ts_.attachClock(clock_);

	// ROS2: needs to declare parameters:
	n_->declare_parameter<std::string>("world_file", "default.world.xml");
	n_->declare_parameter<double>("simul_rate", 100);
	n_->declare_parameter<double>("base_watchdog_timeout", 0.2);
	{
		double t;
		base_watchdog_timeout_ =
			std::chrono::milliseconds(1000 * n_->get_parameter_or("base_watchdog_timeout", t, 0.2));
	}

	realtime_factor_ = n_->declare_parameter<double>("realtime_factor", realtime_factor_);

	max_simul_catchup_time_ =
		n_->declare_parameter<double>("max_simul_catchup_time", max_simul_catchup_time_);

	gui_refresh_period_ms_ = static_cast<int>(
		n_->declare_parameter<double>("gui_refresh_period", gui_refresh_period_ms_));

	headless_ = n_->declare_parameter<bool>("headless", headless_);

	period_ms_publish_tf_ =
		n_->declare_parameter<double>("period_ms_publish_tf", period_ms_publish_tf_);

	do_fake_localization_ =
		n_->declare_parameter<bool>("do_fake_localization", do_fake_localization_);

	publish_tf_odom2baselink_ =
		n_->declare_parameter<bool>("publish_tf_odom2baselink", publish_tf_odom2baselink_);

	publisher_history_len_ = static_cast<int>(
		n_->declare_parameter<int>("publisher_history_len", publisher_history_len_));

	force_publish_vehicle_namespace_ = n_->declare_parameter<bool>(
		"force_publish_vehicle_namespace", force_publish_vehicle_namespace_);

	publish_log_topics_ = n_->declare_parameter<bool>("publish_log_topics", publish_log_topics_);

	disable_sim_time_clock_ =
		n_->declare_parameter<bool>("disable_sim_time_clock", disable_sim_time_clock_);

	objects_ground_truth_rate_ =
		n_->declare_parameter<double>("objects_ground_truth_rate", objects_ground_truth_rate_);

	world_frame_id_ = n_->declare_parameter<std::string>("world_frame_id", world_frame_id_);

	// mvsim is the ROS *time source*: it publishes "/clock" and stamps all
	// outgoing messages with simulation time. The mvsim node itself therefore
	// normally runs with use_sim_time:=false (it drives the clock); it is the
	// downstream nodes that should set use_sim_time:=true. This does not apply
	// when disable_sim_time_clock_ is set, since the node no longer drives the
	// clock in that case.
	{
		bool use_sim_time = false;
		n_->get_parameter_or("use_sim_time", use_sim_time, false);
		if (!disable_sim_time_clock_ && use_sim_time)
		{
			RCLCPP_WARN(
				n_->get_logger(),
				"use_sim_time=true was set on the mvsim node itself. mvsim is "
				"the /clock time source and normally runs with "
				"use_sim_time:=false; set use_sim_time:=true on downstream nodes "
				"instead.");
		}
	}
#endif

	// Launch GUI thread:
	thread_params_.obj = this;
	thGUI_ = std::thread(&MVSimNode::thread_update_GUI, std::ref(thread_params_));

	// Init ROS publishers:
#if PACKAGE_ROS_VERSION == 1
	pub_clock_ = mvsim_node::make_shared<ros::Publisher>(
		n_.advertise<Msg_Clock>("/clock", publisher_history_len_));
#else
	pub_clock_ = n_->create_publisher<Msg_Clock>("/clock", rclcpp::ClockQoS());
#endif

#if PACKAGE_ROS_VERSION == 1
	base_last_cmd_.fromSec(0.0);
#else
	base_last_cmd_ = rclcpp::Time(0);
#endif

	mvsim_world_->registerCallbackOnEntityChange([this](const mvsim::World::EntityChange& c)
												 { onWorldEntityChange(c); });

	mvsim_world_->registerCallbackOnObservation(
		[this](const mvsim::Simulable& veh, const mrpt::obs::CObservation::Ptr& obs)
		{
			if (!obs)
			{
				return;
			}
			mrpt::system::CTimeLoggerEntry tle(profiler_, "lambda_onNewObservation");

			// Only vehicles have ROS interfaces. Keep the vehicle alive until
			// published, even if it is removed at runtime meanwhile:
			std::shared_ptr<TPubSubPerVehicle> pubsPtr;
			if (const auto* v = dynamic_cast<const mvsim::VehicleBase*>(&veh); v)
			{
				auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
				pubsPtr = findPubSubs(*v);
			}
			if (!pubsPtr)
			{
				return;
			}
			const mrpt::obs::CObservation::Ptr obsCopy = obs;
			const auto fut = ros_publisher_workers_.enqueue(
				[this, pubsPtr, obsCopy]()
				{
					try
					{
						onNewObservation(*pubsPtr->vehicle, obsCopy);
					}
					catch (const std::exception& e)
					{
						ROS12_ERROR(
							"[MVSimNode] Error processing observation with "
							"label "
							"'%s':\n%s",
							obsCopy ? obsCopy->sensorLabel.c_str() : "(nullptr)", e.what());
					}
				});
			(void)fut;
		});
}

void MVSimNode::launch_mvsim_server()
{
	ROS12_INFO("[MVSimNode] launch_mvsim_server()");
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

	ASSERT_(!mvsim_server_);

	// Start network server:
	mvsim_server_ = mvsim_node::make_shared<mvsim::Server>();

	mvsim_server_->start();
#endif
}

void MVSimNode::loadWorldModel(const std::string& world_xml_file)
{
	ROS12_INFO("[MVSimNode] Loading world file: %s", world_xml_file.c_str());

	ASSERT_FILE_EXISTS_(world_xml_file);

	if (!headless_)
	{
		// Give feedback while loading the world:
		mvsim_world_->open_GUI_while_loading();
	}

	// Load from XML:
	rapidxml::file<> fil_xml(world_xml_file.c_str());
	mvsim_world_->load_from_XML(fil_xml.data(), world_xml_file);

	// Headless mode can be requested from the ROS parameter or the world file:
	if (mvsim_world_->headless())
	{
		headless_ = true;
	}

	ROS12_INFO("[MVSimNode] World file load done.");
	world_init_ok_ = true;

	// Notify the ROS system about the good news:
	notifyROSWorldIsUpdated();
}

/*------------------------------------------------------------------------------
 * ~MVSimNode()
 * Destructor.
 *----------------------------------------------------------------------------*/
MVSimNode::~MVSimNode()
{  // dtor
	terminateSimulation();
}

void MVSimNode::terminateSimulation()
{
	if (!mvsim_world_)
	{
		return;
	}
	mvsim_world_->simulator_must_close(true);

	thread_params_.closing = true;
	if (thGUI_.joinable())
	{
		thGUI_.join();
	}

	mvsim_world_->free_opengl_resources();

	ros_publisher_workers_.clear();
	// Don't destroy mvsim_server_ yet, since "world" needs to unregister.
	mvsim_world_.reset();
	std::cout << "[MVSimNode::terminateSimulation] All done." << std::endl;
}

#if PACKAGE_ROS_VERSION == 1
/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/
void MVSimNode::configCallback(mvsim::mvsimNodeConfig& config, [[maybe_unused]] uint32_t level)
{
	// Set class variables to new values. They should match what is input at the
	// dynamic reconfigure GUI.
	//  message = config.message.c_str();
	ROS12_INFO("MVSimNode::configCallback() called.");

	if (mvsim_world_->is_GUI_open() && !config.show_gui) mvsim_world_->close_GUI();
}
#endif

// Process pending msgs, run real-time simulation, etc.
void MVSimNode::spin()
{
	using namespace mvsim;
	using namespace std::string_literals;

	if (!mvsim_world_)
	{
		return;
	}
	// Do simulation itself:
	// ========================================================================
	// Handle 1st iter:
	if (t_old_ < 0)
	{
		t_old_ = realtime_tictac_.Tac();
	}
	// Compute how much time has passed to simulate in real-time:
	const double t_new = realtime_tictac_.Tac();
	const double wall_gap = t_new - t_old_;
	double incr_time = realtime_factor_ * wall_gap;

	// Just in case the computer is *really fast*...
	if (incr_time < mvsim_world_->get_simul_timestep())
	{
		return;
	}

	// Bound the per-iteration catch-up to avoid the fixed-timestep "spiral of
	// death": if a previous spin blocked (e.g. waiting for OpenGL sensor
	// rendering, a slow subscription callback, or the machine being starved),
	// this spin fires late and incr_time balloons; run_simulation() would then
	// integrate that whole span as many blocking sub-steps in one call, making
	// the *next* spin even later -> the simulation publishes odometry/TF/sensors
	// in multi-hundred-ms bursts instead of smoothly. Capping lets sim time fall
	// slightly behind wall-clock under load and recover, rather than cascading.
	if (max_simul_catchup_time_ > 0 && incr_time > max_simul_catchup_time_)
	{
		ROS12_WARN_THROTTLE(
			10000,
			"Simulation slower than real time: capping catch-up %.3f s -> %.3f "
			"s (this spin fired %.3f s late). Reduce sensor/GUI load or raise "
			"'max_simul_catchup_time' if this persists.",
			incr_time, max_simul_catchup_time_, wall_gap);
		incr_time = max_simul_catchup_time_;
	}

	// Simulate:
	mvsim_world_->run_simulation(incr_time);

	// t_old_simul = world.get_simul_time();
	t_old_ = t_new;

	const auto& vehs = mvsim_world_->getListOfVehicles();

	// Publish new state to ROS
	// ========================================================================
	this->spinNotifyROS();

	// GUI msgs, teleop, etc.
	// ========================================================================
	if (tim_teleop_refresh_.Tac() > period_ms_teleop_refresh_ * 1e-3)
	{
		tim_teleop_refresh_.Tic();

		std::string txt2gui_tmp;
		World::GUIKeyEvent keyevent = gui_key_events_;

		// Global keys:
		switch (keyevent.keycode)
		{
			// case 27: do_exit=true; break;
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
				teleop_idx_veh_ = keyevent.keycode - '1';
				break;
			default:
				// do nothing
				break;
		};

		{  // Test: Differential drive: Control raw forces
			txt2gui_tmp += mrpt::format(
				"Selected vehicle: %u/%u\n", static_cast<unsigned>(teleop_idx_veh_ + 1),
				static_cast<unsigned>(vehs.size()));
			if (vehs.size() > teleop_idx_veh_)
			{
				// Get iterator to selected vehicle:
				auto it_veh = vehs.begin();
				std::advance(it_veh, teleop_idx_veh_);

				// Get speed: ground truth
				txt2gui_tmp += "gt. vel: "s + it_veh->second->getRefVelocityLocal().asString();

				// Get speed: ground truth
				txt2gui_tmp +=
					"\nodo vel: "s + it_veh->second->getVelocityLocalOdoEstimate().asString();

				// Generic teleoperation interface for any controller that
				// supports it:
				{
					auto* controller = it_veh->second->getControllerInterface();
					ControllerBaseInterface::TeleopInput teleop_in;
					ControllerBaseInterface::TeleopOutput teleop_out;
					teleop_in.keycode = keyevent.keycode;
					teleop_in.js = mvsim_world_->getJoystickState();
					controller->teleop_interface(teleop_in, teleop_out);
					txt2gui_tmp += teleop_out.append_gui_lines;
				}
			}
		}

		msg2gui_ = txt2gui_tmp;	 // send txt msgs to show in the GUI

		// Clear the keystroke buffer
		if (keyevent.keycode != 0)
		{
			gui_key_events_ = World::GUIKeyEvent();
		}

	}  // end refresh teleop stuff

	// Check cmd_vel timeout:
	const double rosNow = myNowSec();
	std::set<mvsim::VehicleBase*> toRemove;
	for (const auto& [veh, cmdVelTimestamp] : lastCmdVelTimestamp_)
	{
		if (rosNow - cmdVelTimestamp > MAX_CMD_VEL_AGE_SECONDS)
		{
			auto* controller = veh->getControllerInterface();

			controller->setTwistCommand({0, 0, 0});
			toRemove.insert(veh);
		}
	}
	for (auto* veh : toRemove)
	{
		lastCmdVelTimestamp_.erase(veh);
	}
}

/*------------------------------------------------------------------------------
 * thread_update_GUI()
 *----------------------------------------------------------------------------*/
void MVSimNode::thread_update_GUI(TThreadParams& thread_params)
{
	try
	{
		MVSimNode* obj = thread_params.obj;

		while (!thread_params.closing)
		{
			if (obj->world_init_ok_ && !obj->headless_)
			{
				mvsim::World::TUpdateGUIParams guiparams;
				guiparams.msg_lines = obj->msg2gui_;

				obj->mvsim_world_->update_GUI(&guiparams);

				// Send key-strokes to the main thread:
				if (guiparams.keyevent.keycode != 0)
				{
					obj->gui_key_events_ = guiparams.keyevent;
				}

				std::this_thread::sleep_for(std::chrono::milliseconds(obj->gui_refresh_period_ms_));
			}
			else if (obj->world_init_ok_ && obj->headless_)
			{
				obj->mvsim_world_->internalGraphicsLoopTasksForSimulation();

				std::this_thread::sleep_for(std::chrono::microseconds(
					static_cast<size_t>(obj->mvsim_world_->get_simul_timestep() * 1000000)));
			}
			else
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(obj->gui_refresh_period_ms_));
			}
		}

		// OpenGL resources must be freed from the thread that created them:
		if (obj->world_init_ok_ && obj->headless_)
		{
			obj->mvsim_world_->internalFreeOpenGLResourcesForSimulation();
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "[MVSimNode::thread_update_GUI] Exception:\n" << e.what();
	}
}

// Visitor: Vehicles
// ----------------------------------------
void MVSimNode::publishVehicles([[maybe_unused]] mvsim::VehicleBase& veh)
{
	//
}

// Visitor: World elements
// ----------------------------------------
void MVSimNode::publishWorldElements(mvsim::WorldElementBase& obj)
{
	// GridMaps --------------
	if (mvsim::OccupancyGridMap* grid = dynamic_cast<mvsim::OccupancyGridMap*>(&obj); grid)
	{
		Msg_OccupancyGrid ros_map;
		mrpt2ros::toROS(grid->getOccGrid(), ros_map);

#if PACKAGE_ROS_VERSION == 1
		static size_t loop_count = 0;
		ros_map.header.seq = loop_count++;
#else
		ros_map.header.frame_id = "map";
#endif
		ros_map.header.stamp = myNow();

		worldPubs_.pub_map_ros->publish(ros_map);
		worldPubs_.pub_map_metadata->publish(ros_map.info);

	}  // end gridmap

}  // end visit(World Elements)

// ROS: Publish grid map for visualization purposes:
void MVSimNode::notifyROSWorldIsUpdated()
{
	mvsim_world_->runVisitorOnVehicles([this](mvsim::VehicleBase& v) { publishVehicles(v); });

	// Create subscribers & publishers for each vehicle's stuff:
	// ----------------------------------------------------
	{
		auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
		pubsub_vehicles_.clear();
	}
	for (const auto& [name, veh] : mvsim_world_->getListOfVehicles())
	{
		addVehiclePubSubs(veh, false);
	}

#if PACKAGE_ROS_VERSION == 1
	// pub: simul_map, simul_map_metadata
	worldPubs_.pub_map_ros = mvsim_node::make_shared<ros::Publisher>(
		n_.advertise<Msg_OccupancyGrid>("simul_map", 1 /*queue len*/, true /*latch*/));
	worldPubs_.pub_map_metadata = mvsim_node::make_shared<ros::Publisher>(
		n_.advertise<Msg_MapMetaData>("simul_map_metadata", 1 /*queue len*/, true /*latch*/));
#else
	// pub: <VEH>/simul_map, <VEH>/simul_map_metadata
	// REP-2003: https://ros.org/reps/rep-2003.html
	// Maps:  reliable transient-local
	auto qosLatched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

	worldPubs_.pub_map_ros = n_->create_publisher<Msg_OccupancyGrid>("simul_map", qosLatched);
	worldPubs_.pub_map_metadata =
		n_->create_publisher<Msg_MapMetaData>("simul_map_metadata", qosLatched);

	// pub: objects_ground_truth
	if (objects_ground_truth_rate_ > 0)
	{
		pub_objects_ground_truth_ =
			n_->create_publisher<Msg_TFMessage>("objects_ground_truth", publisher_history_len_);
	}

	// sub: runtime_objects, runtime_overlays
	// Keep all messages, since each one may add or delete objects:
	const auto qosMarkers = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
	sub_runtime_objects_ = n_->create_subscription<Msg_MarkerArray>(
		"runtime_objects", qosMarkers,
		[this](const Msg_MarkerArray& msg) { onRuntimeObjectMarkers(msg, true); });
	sub_runtime_overlays_ = n_->create_subscription<Msg_MarkerArray>(
		"runtime_overlays", qosMarkers,
		[this](const Msg_MarkerArray& msg) { onRuntimeObjectMarkers(msg, false); });

#if defined(MVSIM_HAS_SIMULATION_INTERFACES)
	initSimulationInterfacesServices();
#endif
#endif

	// Publish maps and static stuff:
	mvsim_world_->runVisitorOnWorldElements([this](mvsim::WorldElementBase& obj)
											{ publishWorldElements(obj); });
}

ros_Time MVSimNode::myNow() const
{
	// mvsim is the ROS time authority: header stamps use *simulation* time
	// (wall-clock at sim start + elapsed simulated seconds) so they stay
	// coherent regardless of the real-time factor or transient CPU load, and
	// match the "/clock" topic consumed by downstream nodes running with
	// use_sim_time:=true. Fall back to wall-clock only before the first
	// simulation step, when no sim timestamp exists yet, or when the user
	// explicitly opted out via disable_sim_time_clock_.
	if (!disable_sim_time_clock_ && mvsim_world_ && mvsim_world_->has_simul_timestamp())
	{
		return mrpt2ros::toROS(mvsim_world_->get_simul_timestamp());
	}
#if PACKAGE_ROS_VERSION == 1
	return ros::Time::now();
#else
	return clock_->now();
#endif
}

double MVSimNode::myNowSec() const
{
	if (!disable_sim_time_clock_ && mvsim_world_ && mvsim_world_->has_simul_timestamp())
	{
		return mrpt::Clock::toDouble(mvsim_world_->get_simul_timestamp());
	}
#if PACKAGE_ROS_VERSION == 1
	return ros::Time::now().toSec();
#else
	return static_cast<double>(clock_->now().nanoseconds()) * 1e-9;
#endif
}

ros_Time MVSimNode::myObsStamp(const mrpt::system::TTimeStamp& obsTimestamp) const
{
	// Normally, stamp with the observation's own simulation timestamp (set
	// when the sensor was sampled) so the stamp is unaffected by any latency
	// in the asynchronous publisher worker thread. When the sim clock is
	// disabled, fall back to wall-clock time instead, as done before
	// simulation time support was added.
	if (disable_sim_time_clock_)
	{
		return myNow();
	}
	return mrpt2ros::toROS(obsTimestamp);
}

std::shared_ptr<MVSimNode::TPubSubPerVehicle> MVSimNode::findPubSubs(
	const mvsim::VehicleBase& veh)
{
	const auto it = pubsub_vehicles_.find(veh.getVehicleIndex());
	if (it == pubsub_vehicles_.end() || it->second->vehicle.get() != &veh)
	{
		return {};
	}
	return it->second;
}

void MVSimNode::addVehiclePubSubs(const std::shared_ptr<mvsim::VehicleBase>& veh, bool atRuntime)
{
	ASSERT_(veh);
	{
		// Fixed from now on, so topics do not change if vehicles are inserted
		// or removed later:
		auto lck = mrpt::lockHelper(vehicleUsesNamespaceMtx_);
		vehicleUsesNamespace_[veh->getVehicleIndex()] = atRuntime ||
														 force_publish_vehicle_namespace_ ||
														 mvsim_world_->getListOfVehicles().size() > 1;
	}
	auto pubsubs = std::make_shared<TPubSubPerVehicle>();
	pubsubs->vehicle = veh;
	initPubSubs(*pubsubs, veh.get());
	initLoggerTopicCallbacks(pubsubs, veh.get());

	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	pubsub_vehicles_[veh->getVehicleIndex()] = pubsubs;
}

void MVSimNode::removeVehiclePubSubs(const mvsim::VehicleBase& veh)
{
	{
		auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
		pubsub_vehicles_.erase(veh.getVehicleIndex());
	}
	auto lck = mrpt::lockHelper(vehicleUsesNamespaceMtx_);
	vehicleUsesNamespace_.erase(veh.getVehicleIndex());
}

void MVSimNode::onWorldEntityChange(const mvsim::World::EntityChange& c)
{
	using Kind = mvsim::World::EntityKind;
	if (c.kind == Kind::Vehicle)
	{
		const auto veh = std::dynamic_pointer_cast<mvsim::VehicleBase>(c.object);
		if (!veh)
		{
			return;
		}
		if (c.added)
		{
			addVehiclePubSubs(veh, true);
		}
		else
		{
			removeVehiclePubSubs(*veh);
		}
	}
	else if (c.kind == Kind::Element && c.added)
	{
		if (const auto e = std::dynamic_pointer_cast<mvsim::WorldElementBase>(c.object); e)
		{
			publishWorldElements(*e);
		}
	}
}

/** Initialize all pub/subs required for each vehicle, for the specific vehicle
 * \a veh */
void MVSimNode::initPubSubs(TPubSubPerVehicle& pubsubs, mvsim::VehicleBase* veh)
{
	// sub: <VEH>/cmd_vel
#if PACKAGE_ROS_VERSION == 1
	pubsubs.sub_cmd_vel = mvsim_node::make_shared<ros::Subscriber>(n_.subscribe<Msg_Twist>(
		vehVarName("cmd_vel", *veh), 10,
		[this, veh](Msg_Twist_CSPtr msg) { return this->onROSMsgCmdVel(msg, veh); }));
#else
	pubsubs.sub_cmd_vel = n_->create_subscription<Msg_Twist>(
		vehVarName("cmd_vel", *veh), 10,
		[this, veh](Msg_Twist_CSPtr msg) { return this->onROSMsgCmdVel(msg, veh); });
#endif

#if PACKAGE_ROS_VERSION == 1
	// pub: <VEH>/odom
	pubsubs.pub_odom = mvsim_node::make_shared<ros::Publisher>(
		n_.advertise<Msg_Odometry>(vehVarName("odom", *veh), publisher_history_len_));

	// pub: <VEH>/base_pose_ground_truth
	pubsubs.pub_ground_truth = mvsim_node::make_shared<ros::Publisher>(n_.advertise<Msg_Odometry>(
		vehVarName("base_pose_ground_truth", *veh), publisher_history_len_));

	// pub: <VEH>/collision
	pubsubs.pub_collision = mvsim_node::make_shared<ros::Publisher>(
		n_.advertise<Msg_Bool>(vehVarName("collision", *veh), publisher_history_len_));

	// pub: <VEH>/tf, <VEH>/tf_static
	pubsubs.pub_tf = mvsim_node::make_shared<ros::Publisher>(
		n_.advertise<Msg_TFMessage>(vehVarName("tf", *veh), publisher_history_len_));
	pubsubs.pub_tf_static = mvsim_node::make_shared<ros::Publisher>(
		n_.advertise<Msg_TFMessage>(vehVarName("tf_static", *veh), publisher_history_len_));
#else
	// pub: <VEH>/odom
	pubsubs.pub_odom =
		n_->create_publisher<Msg_Odometry>(vehVarName("odom", *veh), publisher_history_len_);

	// pub: <VEH>/base_pose_ground_truth
	pubsubs.pub_ground_truth = n_->create_publisher<Msg_Odometry>(
		vehVarName("base_pose_ground_truth", *veh), publisher_history_len_);

	// pub: <VEH>/collision
	pubsubs.pub_collision =
		n_->create_publisher<Msg_Bool>(vehVarName("collision", *veh), publisher_history_len_);

	// pub: <VEH>/tf, <VEH>/tf_static
	const auto qos = tf2_ros::DynamicBroadcasterQoS();
	const auto qos_static = tf2_ros::StaticBroadcasterQoS();

	pubsubs.pub_tf = n_->create_publisher<Msg_TFMessage>(vehVarName("tf", *veh), qos);
	pubsubs.pub_tf_static =
		n_->create_publisher<Msg_TFMessage>(vehVarName("tf_static", *veh), qos_static);
#endif

	// pub: <VEH>/chassis_markers
	{
#if PACKAGE_ROS_VERSION == 1
		pubsubs.pub_chassis_markers = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_MarkerArray>(vehVarName("chassis_markers", *veh), 5, true /*latch*/));
#else
		rclcpp::QoS qosLatched5(rclcpp::KeepLast(5));
		qosLatched5.durability(
			rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

		pubsubs.pub_chassis_markers =
			n_->create_publisher<Msg_MarkerArray>(vehVarName("chassis_markers", *veh), qosLatched5);
#endif
		const auto& poly = veh->getChassisShape();

		// Create one "ROS marker" for each wheel + 1 for the chassis:
		auto& msg_shapes = pubsubs.chassis_shape_msg;
		msg_shapes.markers.resize(1 + veh->getNumWheels());

		// [0] Chassis shape:
		auto& chassis_shape_msg = msg_shapes.markers[0];

		chassis_shape_msg.pose = mrpt2ros::toROS_Pose(mrpt::poses::CPose3D::Identity());

		chassis_shape_msg.action = Msg_Marker::MODIFY;
		chassis_shape_msg.type = Msg_Marker::LINE_STRIP;

		chassis_shape_msg.header.frame_id = "base_link";
		chassis_shape_msg.ns = "mvsim.chassis_shape";
		chassis_shape_msg.id = static_cast<int>(veh->getVehicleIndex());
		chassis_shape_msg.scale.x = 0.05;
		chassis_shape_msg.scale.y = 0.05;
		chassis_shape_msg.scale.z = 0.02;
		chassis_shape_msg.points.resize(poly.size() + 1);
		for (size_t i = 0; i <= poly.size(); i++)
		{
			size_t k = i % poly.size();
			chassis_shape_msg.points[i].x = poly[k].x;
			chassis_shape_msg.points[i].y = poly[k].y;
			chassis_shape_msg.points[i].z = 0;
		}
		chassis_shape_msg.color.a = 0.9;
		chassis_shape_msg.color.r = 0.9;
		chassis_shape_msg.color.g = 0.9;
		chassis_shape_msg.color.b = 0.9;
		chassis_shape_msg.frame_locked = true;

		// [1:N] Wheel shapes
		for (size_t i = 0; i < veh->getNumWheels(); i++)
		{
			auto& wheel_shape_msg = msg_shapes.markers[1 + i];
			const auto& w = veh->getWheelInfo(i);

			const double lx = w.diameter * 0.5, ly = w.width * 0.5;

			// Init values. Copy the contents from the chassis msg
			wheel_shape_msg = msg_shapes.markers[0];

			wheel_shape_msg.ns =
				mrpt::format("mvsim.chassis_shape.wheel%u", static_cast<unsigned int>(i));
			wheel_shape_msg.points.resize(5);
			wheel_shape_msg.points[0].x = lx;
			wheel_shape_msg.points[0].y = ly;
			wheel_shape_msg.points[0].z = 0;
			wheel_shape_msg.points[1].x = lx;
			wheel_shape_msg.points[1].y = -ly;
			wheel_shape_msg.points[1].z = 0;
			wheel_shape_msg.points[2].x = -lx;
			wheel_shape_msg.points[2].y = -ly;
			wheel_shape_msg.points[2].z = 0;
			wheel_shape_msg.points[3].x = -lx;
			wheel_shape_msg.points[3].y = ly;
			wheel_shape_msg.points[3].z = 0;
			wheel_shape_msg.points[4] = wheel_shape_msg.points[0];

			wheel_shape_msg.color.r = mrpt::u8tof(w.color.R);
			wheel_shape_msg.color.g = mrpt::u8tof(w.color.G);
			wheel_shape_msg.color.b = mrpt::u8tof(w.color.B);
			wheel_shape_msg.color.a = 1.0f;

			// Set local pose of the wheel wrt the vehicle:
			wheel_shape_msg.pose = mrpt2ros::toROS_Pose(w.pose());
		}  // end for each wheel

		// Publish Initial pose
		pubsubs.pub_chassis_markers->publish(msg_shapes);
	}

	// pub: <VEH>/chassis_polygon
	{
#if PACKAGE_ROS_VERSION == 1
		pubsubs.pub_chassis_shape = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_Polygon>(vehVarName("chassis_polygon", *veh), 1, true /*latch*/));
#else
		rclcpp::QoS qosLatched1(rclcpp::KeepLast(1));
		qosLatched1.durability(
			rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

		pubsubs.pub_chassis_shape =
			n_->create_publisher<Msg_Polygon>(vehVarName("chassis_polygon", *veh), qosLatched1);
#endif
		Msg_Polygon poly_msg;

		// Do the first (and unique) publish:
		const auto& poly = veh->getChassisShape();
		poly_msg.points.resize(poly.size());
		for (size_t i = 0; i < poly.size(); i++)
		{
			poly_msg.points[i].x = static_cast<float>(poly[i].x);
			poly_msg.points[i].y = static_cast<float>(poly[i].y);
			poly_msg.points[i].z = 0;
		}
		pubsubs.pub_chassis_shape->publish(poly_msg);
	}

	if (do_fake_localization_)
	{
#if PACKAGE_ROS_VERSION == 1
		// pub: <VEH>/amcl_pose
		pubsubs.pub_amcl_pose =
			mvsim_node::make_shared<ros::Publisher>(n_.advertise<Msg_PoseWithCovarianceStamped>(
				vehVarName("amcl_pose", *veh), 1, true /*latch*/));
		// pub: <VEH>/particlecloud
		pubsubs.pub_particlecloud = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_PoseArray>(vehVarName("particlecloud", *veh), 1));
#else
		rclcpp::QoS qosLatched1(rclcpp::KeepLast(1));
		qosLatched1.durability(
			rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

		// pub: <VEH>/amcl_pose
		pubsubs.pub_amcl_pose = n_->create_publisher<Msg_PoseWithCovarianceStamped>(
			vehVarName("amcl_pose", *veh), qosLatched1);
		// pub: <VEH>/particlecloud
		pubsubs.pub_particlecloud =
			n_->create_publisher<Msg_PoseArray>(vehVarName("particlecloud", *veh), 1);
#endif
	}

	// TF STATIC(namespace <Ri>): /base_link -> /base_footprint
	Msg_TransformStamped tx;
	tx.header.frame_id = "base_link";
	tx.child_frame_id = "base_footprint";
	tx.header.stamp = myNow();
	tx.transform = tf2::toMsg(tfIdentity_);

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tx);
	pubsubs.pub_tf_static->publish(tfMsg);
}

void MVSimNode::onROSMsgCmdVel(Msg_Twist_CSPtr cmd, mvsim::VehicleBase* veh)
{
	auto* controller = veh->getControllerInterface();

	// Update cmd_vel timestamp:
	lastCmdVelTimestamp_[veh] = myNowSec();

	const bool ctrlAcceptTwist =
		controller->setTwistCommand({cmd->linear.x, cmd->linear.y, cmd->angular.z});

	if (!ctrlAcceptTwist)
	{
		ROS12_WARN_THROTTLE(
			1.0, "*Warning* Vehicle's controller ['%s'] refuses Twist commands!",
			veh->getName().c_str());
	}
}

/** Publish everything to be published at each simulation iteration */
void MVSimNode::spinNotifyROS()
{
	if (!mvsim_world_)
	{
		return;
	}
	const auto& vehs = mvsim_world_->getListOfVehicles();

	// skip if the node is already shutting down:
	if (!ok())
	{
		return;
	}
	// Publish "/clock" so downstream nodes can run with use_sim_time:=true.
	// mvsim is the simulation time authority; all header stamps below use the
	// same simulation time base (see myNow()). Skipped entirely when
	// disable_sim_time_clock_ is set, restoring pre-sim-clock behavior.
	// ----------------------------------------------------------------
	if (!disable_sim_time_clock_ && pub_clock_ && mvsim_world_->has_simul_timestamp())
	{
		Msg_Clock clockMsg;
		clockMsg.clock = mrpt2ros::toROS(mvsim_world_->get_simul_timestamp());
		pub_clock_->publish(clockMsg);
	}

#if PACKAGE_ROS_VERSION == 2
	publishObjectsGroundTruth();
#endif

	// Publish all TFs for each vehicle:
	// ---------------------------------------------------------------------
	if (tim_publish_tf_.Tac() > period_ms_publish_tf_ * 1e-3)
	{
		tim_publish_tf_.Tic();

		for (auto it = vehs.begin(); it != vehs.end(); ++it)
		{
			const auto& veh = it->second;
			const auto pubsPtr = findPubSubs(*veh);
			if (!pubsPtr)
			{
				continue;
			}
			auto& pubs = *pubsPtr;

			// 1) Ground-truth pose and velocity
			// --------------------------------------------
			const mrpt::math::TPose3D& gh_veh_pose = veh->getPose();
			const auto veh_odom_pose = veh->getOdometry();

			// [vx,vy,w] in global frame
			const auto& gh_veh_vel = veh->getRefVelocityLocal();

			{
				Msg_Odometry gtOdoMsg;
				gtOdoMsg.pose.pose = mrpt2ros::toROS_Pose(gh_veh_pose);

				gtOdoMsg.twist.twist.linear.x = gh_veh_vel.vx;
				gtOdoMsg.twist.twist.linear.y = gh_veh_vel.vy;
				gtOdoMsg.twist.twist.linear.z = 0;
				gtOdoMsg.twist.twist.angular.z = gh_veh_vel.omega;

				gtOdoMsg.header.stamp = myNow();
				gtOdoMsg.header.frame_id = "odom";
				gtOdoMsg.child_frame_id = "base_link";

				pubs.pub_ground_truth->publish(gtOdoMsg);

				if (do_fake_localization_)
				{
					Msg_PoseWithCovarianceStamped currentPos;
					Msg_PoseArray particleCloud;

					// topic: <Ri>/particlecloud
					{
						particleCloud.header.stamp = myNow();
						particleCloud.header.frame_id = "map";
						particleCloud.poses.resize(1);
						particleCloud.poses[0] = gtOdoMsg.pose.pose;
						pubs.pub_particlecloud->publish(particleCloud);
					}

					// topic: <Ri>/amcl_pose
					{
						currentPos.header = gtOdoMsg.header;
						currentPos.pose.pose = gtOdoMsg.pose.pose;
						pubs.pub_amcl_pose->publish(currentPos);
					}

					// TF(namespace <Ri>): /map -> /odom
					{
						Msg_TransformStamped tx;
						tx.header.frame_id = "map";
						tx.child_frame_id = "odom";
						tx.header.stamp =
							myNow() + std::chrono::milliseconds(50);  // Fix deay Fernando
						tx.transform = tf2::toMsg(tf2::Transform::getIdentity());

						Msg_TFMessage tfMsg;
						tfMsg.transforms.push_back(tx);
						pubs.pub_tf->publish(tfMsg);
					}
				}
			}

			// 2) Chassis markers (for rviz visualization)
			// --------------------------------------------
			// pub: <VEH>/chassis_markers
			{
				// visualization_msgs::MarkerArray
				auto& msg_shapes = pubs.chassis_shape_msg;
				ASSERT_EQUAL_(msg_shapes.markers.size(), (1 + veh->getNumWheels()));

				// [0] Chassis shape: static no need to update.
				// [1:N] Wheel shapes: may move
				for (size_t j = 0; j < veh->getNumWheels(); j++)
				{
					// visualization_msgs::Marker
					auto& wheel_shape_msg = msg_shapes.markers[1 + j];
					const auto& w = veh->getWheelInfo(j);

					// Set local pose of the wheel wrt the vehicle:
					wheel_shape_msg.pose = mrpt2ros::toROS_Pose(w.pose());

				}  // end for each wheel

				// Publish Initial pose
				pubs.pub_chassis_markers->publish(msg_shapes);
			}

			// 3) odometry transform
			// --------------------------------------------
			{
				// TF(namespace <Ri>): /odom -> /base_link
				if (publish_tf_odom2baselink_)
				{
					Msg_TransformStamped tx;
					tx.header.frame_id = "odom";
					tx.child_frame_id = "base_link";
					tx.header.stamp = myNow();
					tx.transform = tf2::toMsg(mrpt2ros::toROS_tfTransform(veh_odom_pose));

					Msg_TFMessage tfMsg;
					tfMsg.transforms.push_back(tx);
					pubs.pub_tf->publish(tfMsg);
				}

				// Apart from TF, publish to the "odom" topic as well
				{
					Msg_Odometry odoMsg;
					odoMsg.pose.pose = mrpt2ros::toROS_Pose(veh_odom_pose);

					// twist is given in the child_frame_id (base_link) frame, as
					// reconstructed from wheels spinning velocities and geometry:
					const auto veh_odo_vel = veh->getVelocityLocalOdoEstimate();
					odoMsg.twist.twist.linear.x = veh_odo_vel.vx;
					odoMsg.twist.twist.linear.y = veh_odo_vel.vy;
					odoMsg.twist.twist.linear.z = 0;
					odoMsg.twist.twist.angular.z = veh_odo_vel.omega;

					// first, we'll populate the header for the odometry msg
					odoMsg.header.stamp = myNow();
					odoMsg.header.frame_id = "odom";
					odoMsg.child_frame_id = "base_link";

					// publish:
					pubs.pub_odom->publish(odoMsg);
				}
			}

			// 4) Collision status
			// --------------------------------------------
			const bool col = veh->hadCollision();
			veh->resetCollisionFlag();
			{
				Msg_Bool colMsg;
				colMsg.data = col;

				// publish:
				pubs.pub_collision->publish(colMsg);
			}

		}  // end for each vehicle

	}  // end publish tf

}  // end spinNotifyROS()

#if PACKAGE_ROS_VERSION == 1
void MVSimNode::initLoggerTopicCallbacks(
	const std::shared_ptr<TPubSubPerVehicle>& /*pubsubs*/, mvsim::VehicleBase* /*veh*/)
{
	// Log-topic publishing is only supported in ROS2.
}
#else
void MVSimNode::initLoggerTopicCallbacks(
	const std::shared_ptr<TPubSubPerVehicle>& pubsubs, mvsim::VehicleBase* veh)
{
	if (!publish_log_topics_)
	{
		return;
	}

	const size_t nLoggers = 1 + veh->getNumWheels();
	size_t registeredCount = 0;

	for (size_t li = 0; li < nLoggers; li++)
	{
		auto logger = veh->getLoggerPtr(li);
		if (!logger)
		{
			continue;
		}
		++registeredCount;

		// Determine a human-readable label for this logger
		std::string loggerLabel;
		if (li == mvsim::VehicleBase::LOGGER_IDX_POSE)
		{
			loggerLabel = "log/pose";
		}
		else
		{
			loggerLabel =
				"log/wheel_" + std::to_string(li - mvsim::VehicleBase::LOGGER_IDX_WHEELS + 1);
		}

		const size_t loggerIdx = li;

		// Register callback that publishes every column as a Float64 topic.
		// The logger belongs to the vehicle, so "veh" outlives it.
		const std::weak_ptr<TPubSubPerVehicle> weakPubSubs = pubsubs;
		logger->registerOnRowCallback(
			[this, weakPubSubs, veh, loggerLabel,
			 loggerIdx](const std::map<std::string_view, double>& columns)
			{
				const auto pubsubsPtr = weakPubSubs.lock();
				if (!pubsubsPtr)
				{
					return;	 // removed at runtime
				}
				auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
				auto& pubMap = pubsubsPtr->pub_log_topics[loggerIdx];

				for (const auto& [colName, value] : columns)
				{
					const std::string colStr(colName);

					// Lazily create publisher on first encounter
					auto it = pubMap.find(colStr);
					if (it == pubMap.end())
					{
						const std::string topicName = vehVarName(loggerLabel + "/" + colStr, *veh);

						it = pubMap
								 .emplace(
									 colStr, n_->create_publisher<Msg_Float64>(
												 topicName, publisher_history_len_))
								 .first;

						RCLCPP_INFO(
							n_->get_logger(), "[MVSimNode] Publishing log topic: %s",
							topicName.c_str());
					}

					Msg_Float64 msg;
					msg.data = value;
					it->second->publish(msg);
				}
			});
	}

	RCLCPP_INFO(
		n_->get_logger(), "[MVSimNode] Registered %zu log-topic callbacks for vehicle '%s'",
		registeredCount, veh->getName().c_str());
}
#endif

void MVSimNode::onNewObservation(
	const mvsim::Simulable& sim, const mrpt::obs::CObservation::Ptr& obs)
{
	mrpt::system::CTimeLoggerEntry tle(profiler_, "onNewObservation");

	// skip if the node is already shutting down:
	if (!ok())
	{
		return;
	}
	ASSERT_(obs);
	ASSERT_(!obs->sensorLabel.empty());

	const auto& vehPtr = dynamic_cast<const mvsim::VehicleBase*>(&sim);
	if (!vehPtr)
	{
		return;	 // for example, if obs from invisible aux block.
	}
	const auto& veh = *vehPtr;

	// -----------------------------
	// Observation: 2d laser scans
	// -----------------------------
	if (const auto* o2DLidar = dynamic_cast<const mrpt::obs::CObservation2DRangeScan*>(obs.get());
		o2DLidar)
	{
		internalOn(veh, *o2DLidar);
	}
	else if (const auto* oImage = dynamic_cast<const mrpt::obs::CObservationImage*>(obs.get());
			 oImage)
	{
		internalOn(veh, *oImage);
	}
	else if (const auto* oRGBD = dynamic_cast<const mrpt::obs::CObservation3DRangeScan*>(obs.get());
			 oRGBD)
	{
		internalOn(veh, *oRGBD);
	}
	else if (const auto* oPC = dynamic_cast<const mrpt::obs::CObservationPointCloud*>(obs.get());
			 oPC)
	{
		internalOn(veh, *oPC);
	}
	else if (const auto* oIMU = dynamic_cast<const mrpt::obs::CObservationIMU*>(obs.get()); oIMU)
	{
		internalOn(veh, *oIMU);
	}
	else if (const auto* oGPS = dynamic_cast<const mrpt::obs::CObservationGPS*>(obs.get()); oGPS)
	{
		internalOn(veh, *oGPS);
	}
	else
	{
		// Don't know how to emit this observation to ROS!
		ROS12_WARN_STREAM_THROTTLE(
			1.0, "Do not know how to publish this observation to ROS: '"
					 << obs->sensorLabel << "', class: " << obs->GetRuntimeClass()->className);
	}

}  // end of onNewObservation()

/** Creates the string "/<VEH_NAME>/<VAR_NAME>" if there're more than one
 * vehicle in the World, or "/<VAR_NAME>" otherwise. */
std::string MVSimNode::vehVarName(const std::string& sVarName, const mvsim::VehicleBase& veh) const
{
	bool useNamespace = force_publish_vehicle_namespace_ ||
						mvsim_world_->getListOfVehicles().size() > 1;
	{
		auto lck = mrpt::lockHelper(vehicleUsesNamespaceMtx_);
		if (auto it = vehicleUsesNamespace_.find(veh.getVehicleIndex());
			it != vehicleUsesNamespace_.end())
		{
			useNamespace = it->second;
		}
	}
	if (!useNamespace)
	{
		return sVarName;
	}
	return veh.getName() + std::string("/") + sVarName;
}

void MVSimNode::internalOn(
	const mvsim::VehicleBase& veh, const mrpt::obs::CObservation2DRangeScan& obs)
{
	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	const auto pubsPtr = findPubSubs(veh);
	if (!pubsPtr)
	{
		return;	 // removed at runtime
	}
	auto& pubs = *pubsPtr;

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub = pubs.pub_sensors.find(obs.sensorLabel) == pubs.pub_sensors.end();
	auto& pub = pubs.pub_sensors[obs.sensorLabel];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pub = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_LaserScan>(vehVarName(obs.sensorLabel, veh), publisher_history_len_));
#else
		pub = mvsim_node::make_shared<PublisherWrapper<Msg_LaserScan>>(
			n_, vehVarName(obs.sensorLabel, veh), publisher_history_len_);
#endif
	}
	lck.unlock();

	// Stamp with the observation's own simulation timestamp (set when the
	// sensor was sampled), not "now", so the stamp is unaffected by any latency
	// in the asynchronous publisher worker thread (see myObsStamp()).
	const auto obsStamp = myObsStamp(obs.timestamp);

	// Send TF:
	mrpt::poses::CPose3D sensorPose = obs.sensorPose;
	auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.header.frame_id = "base_link";
	tfStmp.child_frame_id = obs.sensorLabel;
	tfStmp.header.stamp = obsStamp;

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tfStmp);
	pubs.pub_tf->publish(tfMsg);

	// Send observation:
	{
		// Convert observation MRPT -> ROS
		Msg_Pose msg_pose_laser;
		Msg_LaserScan msg_laser;
		msg_laser.header.stamp = obsStamp;
		msg_laser.header.frame_id = obs.sensorLabel;
		mrpt2ros::toROS(obs, msg_laser, msg_pose_laser);
		pub->publish(mvsim_node::make_shared<Msg_LaserScan>(msg_laser));
	}
}

void MVSimNode::internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservationIMU& obs)
{
	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	const auto pubsPtr = findPubSubs(veh);
	if (!pubsPtr)
	{
		return;	 // removed at runtime
	}
	auto& pubs = *pubsPtr;

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub = pubs.pub_sensors.find(obs.sensorLabel) == pubs.pub_sensors.end();
	auto& pub = pubs.pub_sensors[obs.sensorLabel];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pub = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_Imu>(vehVarName(obs.sensorLabel, veh), publisher_history_len_));
#else
		pub = mvsim_node::make_shared<PublisherWrapper<Msg_Imu>>(
			n_, vehVarName(obs.sensorLabel, veh), publisher_history_len_);
#endif
	}
	lck.unlock();

	// Stamp with the observation's own simulation timestamp (see note in the
	// 2D LiDAR handler).
	const auto obsStamp = myObsStamp(obs.timestamp);

	// Send TF:
	mrpt::poses::CPose3D sensorPose = obs.sensorPose;
	auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.header.frame_id = "base_link";
	tfStmp.child_frame_id = obs.sensorLabel;
	tfStmp.header.stamp = obsStamp;

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tfStmp);
	pubs.pub_tf->publish(tfMsg);

	// Send observation:
	{
		// Convert observation MRPT -> ROS
		Msg_Imu msg_imu;
		Msg_Header msg_header;
		msg_header.stamp = obsStamp;
		msg_header.frame_id = obs.sensorLabel;
		mrpt2ros::toROS(obs, msg_header, msg_imu);
		pub->publish(mvsim_node::make_shared<Msg_Imu>(msg_imu));
	}
}

void MVSimNode::internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservationGPS& obs)
{
	if (!obs.has_GGA_datum())
	{
		ROS12_WARN_THROTTLE(5.0, "Ignoring GPS observation without GGA field (!)");
		return;
	}

	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	const auto pubsPtr = findPubSubs(veh);
	if (!pubsPtr)
	{
		return;	 // removed at runtime
	}
	auto& pubs = *pubsPtr;

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub = pubs.pub_sensors.find(obs.sensorLabel) == pubs.pub_sensors.end();
	auto& pub = pubs.pub_sensors[obs.sensorLabel];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pub = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_GPS>(vehVarName(obs.sensorLabel, veh), publisher_history_len_));
#else
		pub = mvsim_node::make_shared<PublisherWrapper<Msg_GPS>>(
			n_, vehVarName(obs.sensorLabel, veh), publisher_history_len_);
#endif
	}
	lck.unlock();

	// Stamp with the observation's own simulation timestamp (see note in the
	// 2D LiDAR handler).
	const auto obsStamp = myObsStamp(obs.timestamp);

	// Send TF:
	mrpt::poses::CPose3D sensorPose = obs.sensorPose;
	auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.header.frame_id = "base_link";
	tfStmp.child_frame_id = obs.sensorLabel;
	tfStmp.header.stamp = obsStamp;

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tfStmp);
	pubs.pub_tf->publish(tfMsg);

	// Send observation:
	{
		// Convert observation MRPT -> ROS. Delegate to the library conversion
		// (as the IMU handler above already does) instead of reimplementing it
		// field-by-field: the hand-rolled version here never set
		// msg->status.status, which ROS leaves at NavSatStatus::STATUS_UNKNOWN
		// (-2) by IDL default; consumers that treat anything other than a
		// definite fix as invalid (e.g. mola_state_estimation_smoother's GNSS
		// fusion) would then silently reject every single reading, even
		// though the simulated fix quality (mrpt::obs::gnss::Message_NMEA_GGA
		// ::fields::fix_quality) was perfectly valid.
		Msg_Header msg_header;
		msg_header.stamp = obsStamp;
		msg_header.frame_id = obs.sensorLabel;

		auto msg = mvsim_node::make_shared<Msg_GPS>();
		mrpt2ros::toROS(obs, msg_header, *msg);

		pub->publish(msg);
	}
}

namespace
{
/** Fills all CameraInfo fields from an MRPT calibration struct.
 *  Header must be filled in by caller.
 */
Msg_CameraInfo camInfoToRos(const mrpt::img::TCamera& c)
{
	Msg_CameraInfo ci;
	ci.height = c.nrows;
	ci.width = c.ncols;

#if PACKAGE_ROS_VERSION == 1
	auto& dist = ci.D;
	auto& K = ci.K;
	auto& P = ci.P;
#else
	auto& dist = ci.d;
	auto& K = ci.k;
	auto& P = ci.p;
#endif

	switch (c.distortion)
	{
		case mrpt::img::DistortionModel::kannala_brandt:
			ci.distortion_model = "kannala_brandt";
			dist.resize(4);
			dist[0] = c.k1();
			dist[1] = c.k2();
			dist[2] = c.k3();
			dist[3] = c.k4();
			break;

		case mrpt::img::DistortionModel::plumb_bob:
			ci.distortion_model = "plumb_bob";
			dist.resize(5);
			for (size_t i = 0; i < dist.size(); i++)
			{
				dist[i] = c.dist[i];
			}
			break;

		case mrpt::img::DistortionModel::none:
			ci.distortion_model = "plumb_bob";
			dist.resize(5);
			for (size_t i = 0; i < dist.size(); i++)
			{
				dist[i] = 0;
			}
			break;

		default:
			THROW_EXCEPTION("Unexpected distortion model!");
	}

	K.fill(0);
	K[0] = c.fx();
	K[4] = c.fy();
	K[2] = c.cx();
	K[5] = c.cy();
	K[8] = 1.0;

	P.fill(0);
	P[0] = 1;
	P[5] = 1;
	P[10] = 1;

	return ci;
}

/** Look up the DepthCameraSensor that generated an observation, by matching
 *  sensorLabel against the vehicle's sensor list.
 *  Returns nullptr if not found or not a DepthCameraSensor.
 */
static const mvsim::DepthCameraSensor* findDepthCameraSensor(
	const mvsim::VehicleBase& veh, const std::string& sensorLabel)
{
	for (const auto& s : veh.getSensors())
	{
		if (s && s->getName() == sensorLabel)
		{
			return dynamic_cast<const mvsim::DepthCameraSensor*>(s.get());
		}
	}
	return nullptr;
}

}  // namespace

void MVSimNode::internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservationImage& obs)
{
	using namespace std::string_literals;

	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	const auto pubsPtr = findPubSubs(veh);
	if (!pubsPtr)
	{
		return;	 // removed at runtime
	}
	auto& pubs = *pubsPtr;

	const std::string img_topic = obs.sensorLabel + "/image_raw"s;
	const std::string camInfo_topic = obs.sensorLabel + "/camera_info"s;

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub = pubs.pub_sensors.find(img_topic) == pubs.pub_sensors.end();
	auto& pubImg = pubs.pub_sensors[img_topic];
	auto& pubCamInfo = pubs.pub_sensors[camInfo_topic];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pubImg = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_Image>(vehVarName(img_topic, veh), publisher_history_len_));
		pubCamInfo = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_CameraInfo>(vehVarName(camInfo_topic, veh), publisher_history_len_));
#else
		pubImg = mvsim_node::make_shared<PublisherWrapper<Msg_Image>>(
			n_, vehVarName(img_topic, veh), publisher_history_len_);
		pubCamInfo = mvsim_node::make_shared<PublisherWrapper<Msg_CameraInfo>>(
			n_, vehVarName(camInfo_topic, veh), publisher_history_len_);
#endif
	}
	lck.unlock();

	// Stamp with the observation's own simulation timestamp (see note in the
	// 2D LiDAR handler).
	const auto obsStamp = myObsStamp(obs.timestamp);

	// Send TF:
	mrpt::poses::CPose3D sensorPose = obs.getSensorPose();
	auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.header.frame_id = "base_link";
	tfStmp.child_frame_id = obs.sensorLabel;
	tfStmp.header.stamp = obsStamp;

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tfStmp);
	pubs.pub_tf->publish(tfMsg);

	// Send observation:
	Msg_Header msg_header;
	msg_header.stamp = obsStamp;
	msg_header.frame_id = obs.sensorLabel;

	{
		// Convert observation MRPT -> ROS
		Msg_Image msg_img;
		msg_img = mrpt2ros::toROS(obs.image, msg_header);
		pubImg->publish(mvsim_node::make_shared<Msg_Image>(msg_img));
	}
	// Send CameraInfo
	{
		Msg_CameraInfo camInfo = camInfoToRos(obs.cameraParams);
		camInfo.header = msg_header;
		pubCamInfo->publish(mvsim_node::make_shared<Msg_CameraInfo>(camInfo));
	}
}

void MVSimNode::internalOn(
	const mvsim::VehicleBase& veh, const mrpt::obs::CObservation3DRangeScan& obs)
{
	using namespace std::string_literals;

	// Look up sensor publish options:
	const auto* depthSensor = findDepthCameraSensor(veh, obs.sensorLabel);
	const bool wantDepthImage = depthSensor ? depthSensor->publishDepthImage() : true;
	const bool wantColoredPc = depthSensor ? depthSensor->publishColoredPointcloud() : false;

	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	const auto pubsPtr = findPubSubs(veh);
	if (!pubsPtr)
	{
		return;	 // removed at runtime
	}
	auto& pubs = *pubsPtr;

	const auto lbPoints = obs.sensorLabel + "_points"s;
	const auto lbImage = obs.sensorLabel + "_rgb/image_raw"s;
	const auto lbImageCamInfo = obs.sensorLabel + "_rgb/camera_info"s;
	const auto lbDepthImage = obs.sensorLabel + "_depth/image_raw"s;
	const auto lbDepthCamInfo = obs.sensorLabel + "_depth/camera_info"s;

	// Create the publishers the first time an observation arrives:
	const bool is_1st_pub = pubs.pub_sensors.find(lbPoints) == pubs.pub_sensors.end();

	auto& pubPts = pubs.pub_sensors[lbPoints];
	auto& pubImg = pubs.pub_sensors[lbImage];
	auto& pubImgCamInfo = pubs.pub_sensors[lbImageCamInfo];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pubImg = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_Image>(vehVarName(lbImage, veh), publisher_history_len_));
		pubPts = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_PointCloud2>(vehVarName(lbPoints, veh), publisher_history_len_));
		pubImgCamInfo = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_CameraInfo>(vehVarName(lbImageCamInfo, veh), publisher_history_len_));
#else
		pubImg = mvsim_node::make_shared<PublisherWrapper<Msg_Image>>(
			n_, vehVarName(lbImage, veh), publisher_history_len_);
		pubPts = mvsim_node::make_shared<PublisherWrapper<Msg_PointCloud2>>(
			n_, vehVarName(lbPoints, veh), publisher_history_len_);
		pubImgCamInfo = mvsim_node::make_shared<PublisherWrapper<Msg_CameraInfo>>(
			n_, vehVarName(lbImageCamInfo, veh), publisher_history_len_);
#endif
	}

	// depth image + camera_info publishers
	if (wantDepthImage && obs.hasRangeImage)
	{
		const bool is_1st_depth = pubs.pub_sensors.find(lbDepthImage) == pubs.pub_sensors.end();
		if (is_1st_depth)
		{
			auto& pubDepthImg = pubs.pub_sensors[lbDepthImage];
			auto& pubDepthCamInfo = pubs.pub_sensors[lbDepthCamInfo];
#if PACKAGE_ROS_VERSION == 1
			pubDepthImg = mvsim_node::make_shared<ros::Publisher>(
				n_.advertise<Msg_Image>(vehVarName(lbDepthImage, veh), publisher_history_len_));
			pubDepthCamInfo = mvsim_node::make_shared<ros::Publisher>(n_.advertise<Msg_CameraInfo>(
				vehVarName(lbDepthCamInfo, veh), publisher_history_len_));
#else
			pubDepthImg = mvsim_node::make_shared<PublisherWrapper<Msg_Image>>(
				n_, vehVarName(lbDepthImage, veh), publisher_history_len_);
			pubDepthCamInfo = mvsim_node::make_shared<PublisherWrapper<Msg_CameraInfo>>(
				n_, vehVarName(lbDepthCamInfo, veh), publisher_history_len_);
#endif
		}
	}

	lck.unlock();

	// Stamp with the observation's own simulation timestamp (see note in the
	// 2D LiDAR handler).
	const auto obsStamp = myObsStamp(obs.timestamp);

	// ----------------------------------------------------------------
	// RGB IMAGE
	// ----------------------------------------------------------------
	if (obs.hasIntensityImage)
	{
		// Send TF:
		mrpt::poses::CPose3D sensorPose = obs.sensorPose + obs.relativePoseIntensityWRTDepth;
		auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

		Msg_TransformStamped tfStmp;
		tfStmp.transform = tf2::toMsg(transform);
		tfStmp.header.frame_id = "base_link";
		tfStmp.child_frame_id = lbImage;
		tfStmp.header.stamp = obsStamp;

		Msg_TFMessage tfMsg;
		tfMsg.transforms.push_back(tfStmp);
		pubs.pub_tf->publish(tfMsg);

		Msg_Header msg_header;
		msg_header.stamp = obsStamp;
		msg_header.frame_id = lbImage;

		// Send observation:
		{
			Msg_Image msg_img;
			msg_img = mrpt2ros::toROS(obs.intensityImage, msg_header);
			pubImg->publish(mvsim_node::make_shared<Msg_Image>(msg_img));
		}

		// RGB CameraInfo:
		{
			Msg_CameraInfo camInfo = camInfoToRos(obs.cameraParamsIntensity);
			camInfo.header = msg_header;
			pubImgCamInfo->publish(mvsim_node::make_shared<Msg_CameraInfo>(camInfo));
		}
	}

	// ----------------------------------------------------------------
	// DEPTH IMAGE as 16UC1
	// ----------------------------------------------------------------
	if (wantDepthImage && obs.hasRangeImage)
	{
		// Send TF for depth frame: the depth image is in the camera optical
		// frame (+Z forward), while sensorPose is +X forward:
		{
			const mrpt::poses::CPose3D sensorPose =
				obs.sensorPose + mrpt::poses::CPose3D::FromYawPitchRoll(-M_PI_2, 0.0, -M_PI_2);
			auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

			Msg_TransformStamped tfStmp;
			tfStmp.transform = tf2::toMsg(transform);
			tfStmp.header.frame_id = "base_link";
			tfStmp.child_frame_id = obs.sensorLabel + "_depth";
			tfStmp.header.stamp = obsStamp;

			Msg_TFMessage tfMsg;
			tfMsg.transforms.push_back(tfStmp);
			pubs.pub_tf->publish(tfMsg);
		}

		Msg_Header msg_header;
		msg_header.stamp = obsStamp;
		msg_header.frame_id = obs.sensorLabel + "_depth";

		// Depth image from rangeImage (uint16_t matrix, in rangeUnits, 0 =
		// invalid), as 16UC1 in millimeters (REP 118) or 32FC1 in meters:
		{
			const bool asFloat = depthSensor && depthSensor->rosDepthImageEncoding() == "32FC1";
			const auto cols = static_cast<uint32_t>(obs.rangeImage.cols());
			const auto rows = static_cast<uint32_t>(obs.rangeImage.rows());

			Msg_Image depth_msg;
			depth_msg.header = msg_header;
			depth_msg.width = cols;
			depth_msg.height = rows;
			depth_msg.encoding = asFloat ? "32FC1" : "16UC1";
			depth_msg.is_bigendian = 0;
			depth_msg.step =
				cols * static_cast<uint32_t>(asFloat ? sizeof(float) : sizeof(uint16_t));
			depth_msg.data.resize(static_cast<size_t>(depth_msg.step) * rows);

			const bool isMillimeters = std::abs(obs.rangeUnits - 1e-3f) < 1e-9f;
			if (!asFloat && isMillimeters)
			{
				std::memcpy(depth_msg.data.data(), obs.rangeImage.data(), depth_msg.data.size());
			}
			else
			{
				for (uint32_t r = 0; r < rows; r++)
				{
					for (uint32_t c = 0; c < cols; c++)
					{
						const uint16_t raw = obs.rangeImage(r, c);
						const float meters = raw * obs.rangeUnits;
						const size_t idx = static_cast<size_t>(r) * cols + c;
						if (asFloat)
						{
							// Invalid pixels: NaN, as in common RGB-D drivers
							const float v =
								raw == 0 ? std::numeric_limits<float>::quiet_NaN() : meters;
							std::memcpy(&depth_msg.data[idx * sizeof(float)], &v, sizeof(float));
						}
						else
						{
							const auto mm = static_cast<uint16_t>(
								std::min(65535.0f, std::round(meters * 1000.0f)));
							std::memcpy(
								&depth_msg.data[idx * sizeof(uint16_t)], &mm, sizeof(uint16_t));
						}
					}
				}
			}

			auto lck2 = mrpt::lockHelper(pubsub_vehicles_mtx_);

			auto& pubDepthImg = pubs.pub_sensors[lbDepthImage];
			pubDepthImg->publish(mvsim_node::make_shared<Msg_Image>(depth_msg));
		}

		// Depth CameraInfo:
		{
			Msg_CameraInfo camInfo = camInfoToRos(obs.cameraParams);
			camInfo.header = msg_header;

			auto lck3 = mrpt::lockHelper(pubsub_vehicles_mtx_);

			auto& pubDepthCamInfo = pubs.pub_sensors[lbDepthCamInfo];
			pubDepthCamInfo->publish(mvsim_node::make_shared<Msg_CameraInfo>(camInfo));
		}
	}

	// ----------------------------------------------------------------
	// POINTS (XYZ or XYZRGB)
	// ----------------------------------------------------------------
	if (obs.hasRangeImage)
	{
		// Send TF:
		mrpt::poses::CPose3D sensorPose = obs.sensorPose;
		auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

		Msg_TransformStamped tfStmp;
		tfStmp.transform = tf2::toMsg(transform);
		tfStmp.header.frame_id = "base_link";
		tfStmp.child_frame_id = lbPoints;
		tfStmp.header.stamp = obsStamp;

		Msg_TFMessage tfMsg;
		tfMsg.transforms.push_back(tfStmp);
		pubs.pub_tf->publish(tfMsg);

		// Send observation:
		{
			Msg_PointCloud2 msg_pts;
			Msg_Header msg_header;
			msg_header.stamp = obsStamp;
			msg_header.frame_id = lbPoints;

			mrpt::obs::T3DPointsProjectionParams pp;
			pp.takeIntoAccountSensorPoseOnRobot = false;

			if (wantColoredPc && obs.hasIntensityImage)
			{
				// colored pointcloud (XYZRGB)
				mrpt::maps::CGenericPointsMap pts;
				pts.registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Rf);
				pts.registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gf);
				pts.registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bf);

				const_cast<mrpt::obs::CObservation3DRangeScan&>(obs).unprojectInto(pts, pp);
				mrpt2ros::toROS(pts, msg_header, msg_pts);
			}
			else
			{
				// Original: plain XYZ pointcloud
				mrpt::maps::CSimplePointsMap pts;
				const_cast<mrpt::obs::CObservation3DRangeScan&>(obs).unprojectInto(pts, pp);
				mrpt2ros::toROS(pts, msg_header, msg_pts);
			}
			pubPts->publish(mvsim_node::make_shared<Msg_PointCloud2>(msg_pts));
		}
	}
}

void MVSimNode::internalOn(
	const mvsim::VehicleBase& veh, const mrpt::obs::CObservationPointCloud& obs)
{
	using namespace std::string_literals;

	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	const auto pubsPtr = findPubSubs(veh);
	if (!pubsPtr)
	{
		return;	 // removed at runtime
	}
	auto& pubs = *pubsPtr;

	const auto lbPoints = obs.sensorLabel + "_points"s;

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub = pubs.pub_sensors.find(lbPoints) == pubs.pub_sensors.end();

	auto& pubPts = pubs.pub_sensors[lbPoints];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pubPts = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_PointCloud2>(vehVarName(lbPoints, veh), publisher_history_len_));
#else
		pubPts = mvsim_node::make_shared<PublisherWrapper<Msg_PointCloud2>>(
			n_, vehVarName(lbPoints, veh), publisher_history_len_);
#endif
	}
	lck.unlock();

	// Stamp with the observation's own simulation timestamp (see note in the
	// 2D LiDAR handler).
	const auto obsStamp = myObsStamp(obs.timestamp);

	// POINTS
	// --------

	// Send TF:
	mrpt::poses::CPose3D sensorPose = obs.sensorPose;
	auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.header.frame_id = "base_link";
	tfStmp.child_frame_id = lbPoints;
	tfStmp.header.stamp = obsStamp;

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tfStmp);
	pubs.pub_tf->publish(tfMsg);

	// Send observation:
	{
		// Convert observation MRPT -> ROS
		auto msg_pts = mvsim_node::make_shared<Msg_PointCloud2>();
		Msg_Header msg_header;
		msg_header.stamp = obsStamp;
		msg_header.frame_id = lbPoints;

#if MRPT_VERSION < 0x020f00	 // 2.15.0 support legacy classes
		if (auto* xyzirt = dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(obs.pointcloud.get());
			xyzirt)
		{
			mrpt2ros::toROS(*xyzirt, msg_header, *msg_pts);
		}
		else if (auto* xyzi = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(obs.pointcloud.get());
				 xyzi)
		{
			mrpt2ros::toROS(*xyzi, msg_header, *msg_pts);
		}
		else
#endif
			if (auto* sPts =
					dynamic_cast<const mrpt::maps::CSimplePointsMap*>(obs.pointcloud.get());
				sPts)
		{
			mrpt2ros::toROS(*sPts, msg_header, *msg_pts);
		}
		else if (auto* sGenPts =
					 dynamic_cast<const mrpt::maps::CGenericPointsMap*>(obs.pointcloud.get());
				 sGenPts)
		{
			mrpt2ros::toROS(*sGenPts, msg_header, *msg_pts);
		}
		else
		{
			THROW_EXCEPTION("Do not know how to handle this variant of CPointsMap");
		}

		pubPts->publish(msg_pts);
	}
}

#if PACKAGE_ROS_VERSION == 2
namespace
{
mrpt::img::TColor toColor(const std_msgs::msg::ColorRGBA& c)
{
	const auto f2u8 = [](float v)
	{ return static_cast<uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f); };
	return {f2u8(c.r), f2u8(c.g), f2u8(c.b), f2u8(c.a)};
}

/** Converts a marker into a runtime object. Returns false if the marker
 * type is not supported. */
bool markerToRuntimeObject(
	const visualization_msgs::msg::Marker& m, mvsim::RuntimeObjectDescription& d)
{
	using Shape = mvsim::RuntimeObjectDescription::Shape;
	using visualization_msgs::msg::Marker;

	auto pose = mrpt::ros2bridge::fromROS(m.pose);
	d.size = {m.scale.x, m.scale.y, m.scale.z};
	d.color = toColor(m.color);

	const auto scaled = [&m](const geometry_msgs::msg::Point& p)
	{ return mrpt::math::TPoint3D(p.x * m.scale.x, p.y * m.scale.y, p.z * m.scale.z); };

	switch (m.type)
	{
		case Marker::CUBE:
			// A zero-height cube is a flat decal:
			d.shape = m.scale.z == 0 ? Shape::Rectangle : Shape::Box;
			break;
		case Marker::SPHERE:
			d.shape = Shape::Sphere;
			break;
		case Marker::CYLINDER:
			// A zero-height cylinder is a flat disk decal:
			if (m.scale.z == 0)
			{
				d.shape = Shape::Disk;
			}
			else
			{
				// Markers are centered, mvsim cylinders start at their base:
				d.shape = Shape::Cylinder;
				pose = pose + mrpt::poses::CPose3D(0, 0, -0.5 * m.scale.z, 0, 0, 0);
			}
			break;
		case Marker::TRIANGLE_LIST:
			d.shape = Shape::Triangles;
			for (const auto& p : m.points)
			{
				d.points.push_back(scaled(p));
			}
			for (const auto& c : m.colors)
			{
				d.point_colors.push_back(toColor(c));
			}
			if (d.point_colors.size() != d.points.size())
			{
				d.point_colors.clear();
			}
			break;
		case Marker::LINE_LIST:
		case Marker::LINE_STRIP:
			d.shape = Shape::Lines;
			for (size_t i = 0; i < m.points.size(); i++)
			{
				if (m.type == Marker::LINE_STRIP && i >= 2)
				{
					d.points.push_back(d.points.back());
				}
				d.points.push_back(
					mrpt::math::TPoint3D(m.points[i].x, m.points[i].y, m.points[i].z));
			}
			d.size.x = 2.0;	 // line width (pixels)
			break;
		default:
			return false;
	};
	d.pose = pose.asTPose();
	return true;
}
}  // namespace

void MVSimNode::onRuntimeObjectMarkers(const Msg_MarkerArray& msg, bool visibleToSensors)
{
	using visualization_msgs::msg::Marker;

	if (!mvsim_world_)
	{
		return;
	}
	auto& ro = mvsim_world_->runtimeObjects();

	// Separate name spaces for each topic:
	const std::string prefix = visibleToSensors ? "ros/" : "ros_overlay/";

	std::vector<mvsim::RuntimeObjectDescription> toSpawn;
	for (const auto& m : msg.markers)
	{
		const std::string name = prefix + m.ns + "/" + std::to_string(m.id);

		if (m.action == Marker::DELETEALL)
		{
			// Apply the pending ones first, to keep the message order:
			ro.spawn(toSpawn);
			toSpawn.clear();
			ro.removeByPrefix(prefix);
			continue;
		}
		if (m.action == Marker::DELETE)
		{
			ro.spawn(toSpawn);
			toSpawn.clear();
			ro.remove({name});
			continue;
		}
		if (!m.header.frame_id.empty() && m.header.frame_id != world_frame_id_)
		{
			RCLCPP_WARN_THROTTLE(
				n_->get_logger(), *clock_, 5000,
				"Runtime object markers must be given in the '%s' frame (got '%s')",
				world_frame_id_.c_str(), m.header.frame_id.c_str());
		}

		mvsim::RuntimeObjectDescription d;
		d.name = name;
		d.visible_to_sensors = visibleToSensors;
		if (!markerToRuntimeObject(m, d))
		{
			RCLCPP_WARN_THROTTLE(
				n_->get_logger(), *clock_, 5000,
				"Unsupported marker type %d for runtime objects (supported: CUBE, SPHERE, "
				"CYLINDER, TRIANGLE_LIST, LINE_LIST, LINE_STRIP)",
				m.type);
			continue;
		}
		toSpawn.push_back(std::move(d));
	}

	try
	{
		ro.spawn(toSpawn);
	}
	catch (const std::exception& e)
	{
		RCLCPP_ERROR(n_->get_logger(), "Error spawning runtime objects: %s", e.what());
	}
}

void MVSimNode::publishObjectsGroundTruth()
{
	if (!pub_objects_ground_truth_ || !mvsim_world_->has_simul_timestamp())
	{
		return;
	}
	if (tim_publish_objects_gt_.Tac() < 1.0 / objects_ground_truth_rate_)
	{
		return;
	}
	tim_publish_objects_gt_.Tic();

	const auto snap = mvsim_world_->getGroundTruthSnapshot();
	const auto stamp =
		disable_sim_time_clock_
			? myNow()
			: mrpt2ros::toROS(mvsim_world_->simul_time_to_timestamp(snap.simul_time));

	Msg_TFMessage msg;
	msg.transforms.reserve(snap.objects.size());
	for (const auto& o : snap.objects)
	{
		const auto p = mrpt2ros::toROS_Pose(o.pose);
		Msg_TransformStamped tx;
		tx.header.frame_id = world_frame_id_;
		tx.header.stamp = stamp;
		tx.child_frame_id = o.name;
		tx.transform.translation.x = p.position.x;
		tx.transform.translation.y = p.position.y;
		tx.transform.translation.z = p.position.z;
		tx.transform.rotation = p.orientation;
		msg.transforms.push_back(std::move(tx));
	}
	pub_objects_ground_truth_->publish(msg);
}
#endif
