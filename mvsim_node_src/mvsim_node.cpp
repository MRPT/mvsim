/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>	 // kbhit()
#include <mrpt/version.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>
#include <mvsim/mvsim_node_core.h>

#include "rapidxml_utils.hpp"

#if MRPT_VERSION >= 0x020b04  // >=2.11.4?
#define HAVE_POINTS_XYZIRT
#endif

#if defined(HAVE_POINTS_XYZIRT)
#include <mrpt/maps/CPointsMapXYZIRT.h>
#endif

#if PACKAGE_ROS_VERSION == 1
// ===========================================
//                    ROS 1
// ===========================================
#include <mrpt/ros1bridge/image.h>
#include <mrpt/ros1bridge/imu.h>
#include <mrpt/ros1bridge/laser_scan.h>
#include <mrpt/ros1bridge/map.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/pose.h>
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
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/imu.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>

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
	localn_.param(
		"force_publish_vehicle_namespace", force_publish_vehicle_namespace_,
		force_publish_vehicle_namespace_);

	// JLBC: At present, mvsim does not use sim_time for neither ROS 1 nor
	// ROS 2.
	// n_.setParam("use_sim_time", false);
	if (true == localn_.param("use_sim_time", false))
	{
		THROW_EXCEPTION("At present, MVSIM can only work with use_sim_time=false");
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

	gui_refresh_period_ms_ =
		n_->declare_parameter<double>("gui_refresh_period", gui_refresh_period_ms_);

	headless_ = n_->declare_parameter<bool>("headless", headless_);

	period_ms_publish_tf_ =
		n_->declare_parameter<double>("period_ms_publish_tf", period_ms_publish_tf_);

	do_fake_localization_ =
		n_->declare_parameter<bool>("do_fake_localization", do_fake_localization_);

	publisher_history_len_ =
		n_->declare_parameter<int>("publisher_history_len", publisher_history_len_);

	force_publish_vehicle_namespace_ = n_->declare_parameter<bool>(
		"force_publish_vehicle_namespace", force_publish_vehicle_namespace_);

	// n_->declare_parameter("use_sim_time"); // already declared error?
	if (true == n_->get_parameter_or("use_sim_time", false))
	{
		THROW_EXCEPTION("At present, MVSIM can only work with use_sim_time=false");
	}
#endif

	// Launch GUI thread:
	thread_params_.obj = this;
	thGUI_ = std::thread(&MVSimNode::thread_update_GUI, std::ref(thread_params_));

	// Init ROS publishers:
#if PACKAGE_ROS_VERSION == 1
	// pub_clock_ =
	// mvsim_node::make_shared<ros::Publisher>(n_.advertise<rosgraph_msgs::Clock>("/clock",
	// 10));
#endif

#if PACKAGE_ROS_VERSION == 1
	// sim_time_.fromSec(0.0);
	base_last_cmd_.fromSec(0.0);
#else
	// sim_time_ = rclcpp::Time(0);
	base_last_cmd_ = rclcpp::Time(0);
#endif

	mvsim_world_->registerCallbackOnObservation(
		[this](const mvsim::Simulable& veh, const mrpt::obs::CObservation::Ptr& obs)
		{
			if (!obs) return;

			mrpt::system::CTimeLoggerEntry tle(profiler_, "lambda_onNewObservation");

			const mvsim::Simulable* vehPtr = &veh;
			const mrpt::obs::CObservation::Ptr obsCopy = obs;
			const auto fut = ros_publisher_workers_.enqueue(
				[this, vehPtr, obsCopy]()
				{
					try
					{
						onNewObservation(*vehPtr, obsCopy);
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
		});
}

void MVSimNode::launch_mvsim_server()
{
	ROS12_INFO("[MVSimNode] launch_mvsim_server()");

	ASSERT_(!mvsim_server_);

	// Start network server:
	mvsim_server_ = mvsim_node::make_shared<mvsim::Server>();

	mvsim_server_->start();
}

void MVSimNode::loadWorldModel(const std::string& world_xml_file)
{
	ROS12_INFO("[MVSimNode] Loading world file: %s", world_xml_file.c_str());

	ASSERT_FILE_EXISTS_(world_xml_file);

	// Load from XML:
	rapidxml::file<> fil_xml(world_xml_file.c_str());
	mvsim_world_->load_from_XML(fil_xml.data(), world_xml_file);

	ROS12_INFO("[MVSimNode] World file load done.");
	world_init_ok_ = true;

	// Notify the ROS system about the good news:
	notifyROSWorldIsUpdated();
}

/*------------------------------------------------------------------------------
 * ~MVSimNode()
 * Destructor.
 *----------------------------------------------------------------------------*/
MVSimNode::~MVSimNode() { terminateSimulation(); }

void MVSimNode::terminateSimulation()
{
	if (!mvsim_world_) return;

	mvsim_world_->simulator_must_close(true);

	thread_params_.closing = true;
	if (thGUI_.joinable()) thGUI_.join();

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

	if (!mvsim_world_) return;

	// Do simulation itself:
	// ========================================================================
	// Handle 1st iter:
	if (t_old_ < 0) t_old_ = realtime_tictac_.Tac();
	// Compute how much time has passed to simulate in real-time:
	const double t_new = realtime_tictac_.Tac();
	const double incr_time = realtime_factor_ * (t_new - t_old_);

	// Just in case the computer is *really fast*...
	if (incr_time < mvsim_world_->get_simul_timestep()) return;

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
		World::TGUIKeyEvent keyevent = gui_key_events_;

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
				txt2gui_tmp += "gt. vel: "s + it_veh->second->getVelocityLocal().asString();

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
		if (keyevent.keycode != 0) gui_key_events_ = World::TGUIKeyEvent();

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
				if (guiparams.keyevent.keycode != 0) obj->gui_key_events_ = guiparams.keyevent;

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
	const auto& vehs = mvsim_world_->getListOfVehicles();
	pubsub_vehicles_.clear();
	pubsub_vehicles_.resize(vehs.size());
	size_t idx = 0;
	for (auto it = vehs.begin(); it != vehs.end(); ++it, ++idx)
	{
		mvsim::VehicleBase* veh = dynamic_cast<mvsim::VehicleBase*>(it->second.get());
		if (!veh) continue;

		auto& pubsubs = pubsub_vehicles_[idx];

		initPubSubs(pubsubs, veh);
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
#endif

	// Publish maps and static stuff:
	mvsim_world_->runVisitorOnWorldElements([this](mvsim::WorldElementBase& obj)
											{ publishWorldElements(obj); });
}

ros_Time MVSimNode::myNow() const
{
#if PACKAGE_ROS_VERSION == 1
	return ros::Time::now();
#else
	return clock_->now();
#endif
}

double MVSimNode::myNowSec() const
{
#if PACKAGE_ROS_VERSION == 1
	return ros::Time::now().toSec();
#else
	return clock_->now().nanoseconds() * 1e-9;
#endif
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
		chassis_shape_msg.id = veh->getVehicleIndex();
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

			wheel_shape_msg.color.r = w.color.R / 255.0f;
			wheel_shape_msg.color.g = w.color.G / 255.0f;
			wheel_shape_msg.color.b = w.color.B / 255.0f;
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
			poly_msg.points[i].x = poly[i].x;
			poly_msg.points[i].y = poly[i].y;
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
	if (!mvsim_world_) return;

	const auto& vehs = mvsim_world_->getListOfVehicles();

	// skip if the node is already shutting down:
	if (!ok()) return;

		// Get current simulation time (for messages) and publish "/clock"
		// ----------------------------------------------------------------
#if PACKAGE_ROS_VERSION == 1
		// sim_time_.fromSec(mvsim_world_->get_simul_time());
		// clockMsg_.clock = sim_time_;
		// pub_clock_->publish(clockMsg_);
#else
		// sim_time_ = myNow();
		// MRPT_TODO("Publish /clock for ROS2 too?");
#endif

	// Publish all TFs for each vehicle:
	// ---------------------------------------------------------------------
	if (tim_publish_tf_.Tac() > period_ms_publish_tf_ * 1e-3)
	{
		tim_publish_tf_.Tic();

		size_t i = 0;
		ASSERT_EQUAL_(pubsub_vehicles_.size(), vehs.size());

		for (auto it = vehs.begin(); it != vehs.end(); ++it, ++i)
		{
			const auto& veh = it->second;
			auto& pubs = pubsub_vehicles_[i];

			// 1) Ground-truth pose and velocity
			// --------------------------------------------
			const mrpt::math::TPose3D& gh_veh_pose = veh->getPose();
			// [vx,vy,w] in global frame
			const auto& gh_veh_vel = veh->getTwist();

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
						tx.header.stamp = myNow();
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
				const mrpt::math::TPose3D odo_pose = gh_veh_pose;

				// TF(namespace <Ri>): /odom -> /base_link
				{
					Msg_TransformStamped tx;
					tx.header.frame_id = "odom";
					tx.child_frame_id = "base_link";
					tx.header.stamp = myNow();
					tx.transform = tf2::toMsg(mrpt2ros::toROS_tfTransform(odo_pose));

					Msg_TFMessage tfMsg;
					tfMsg.transforms.push_back(tx);
					pubs.pub_tf->publish(tfMsg);
				}

				// Apart from TF, publish to the "odom" topic as well
				{
					Msg_Odometry odoMsg;
					odoMsg.pose.pose = mrpt2ros::toROS_Pose(odo_pose);

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

void MVSimNode::onNewObservation(
	const mvsim::Simulable& sim, const mrpt::obs::CObservation::Ptr& obs)
{
	mrpt::system::CTimeLoggerEntry tle(profiler_, "onNewObservation");

	// skip if the node is already shutting down:
	if (!ok()) return;

	ASSERT_(obs);
	ASSERT_(!obs->sensorLabel.empty());

	const auto& vehPtr = dynamic_cast<const mvsim::VehicleBase*>(&sim);
	if (!vehPtr) return;  // for example, if obs from invisible aux block.
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
	if (mvsim_world_->getListOfVehicles().size() == 1 && !force_publish_vehicle_namespace_)
	{
		return sVarName;
	}
	else
	{
		return veh.getName() + std::string("/") + sVarName;
	}
}

void MVSimNode::internalOn(
	const mvsim::VehicleBase& veh, const mrpt::obs::CObservation2DRangeScan& obs)
{
	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	auto& pubs = pubsub_vehicles_[veh.getVehicleIndex()];

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

	// Send TF:
	mrpt::poses::CPose3D sensorPose = obs.sensorPose;
	auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.header.frame_id = "base_link";
	tfStmp.child_frame_id = obs.sensorLabel;
	tfStmp.header.stamp = myNow();

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tfStmp);
	pubs.pub_tf->publish(tfMsg);

	// Send observation:
	{
		// Convert observation MRPT -> ROS
		Msg_Pose msg_pose_laser;
		Msg_LaserScan msg_laser;
		// Force usage of simulation time:
		msg_laser.header.stamp = myNow();
		msg_laser.header.frame_id = obs.sensorLabel;
		mrpt2ros::toROS(obs, msg_laser, msg_pose_laser);
		pub->publish(mvsim_node::make_shared<Msg_LaserScan>(msg_laser));
	}
}

void MVSimNode::internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservationIMU& obs)
{
	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	auto& pubs = pubsub_vehicles_[veh.getVehicleIndex()];

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

	// Send TF:
	mrpt::poses::CPose3D sensorPose = obs.sensorPose;
	auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.header.frame_id = "base_link";
	tfStmp.child_frame_id = obs.sensorLabel;
	tfStmp.header.stamp = myNow();

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tfStmp);
	pubs.pub_tf->publish(tfMsg);

	// Send observation:
	{
		// Convert observation MRPT -> ROS
		Msg_Imu msg_imu;
		Msg_Header msg_header;
		// Force usage of simulation time:
		msg_header.stamp = myNow();
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
	auto& pubs = pubsub_vehicles_[veh.getVehicleIndex()];

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

	// Send TF:
	mrpt::poses::CPose3D sensorPose = obs.sensorPose;
	auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.header.frame_id = "base_link";
	tfStmp.child_frame_id = obs.sensorLabel;
	tfStmp.header.stamp = myNow();

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tfStmp);
	pubs.pub_tf->publish(tfMsg);

	// Send observation:
	{
		// Convert observation MRPT -> ROS
		auto msg = mvsim_node::make_shared<Msg_GPS>();
		msg->header.stamp = myNow();
		msg->header.frame_id = obs.sensorLabel;

		const auto& o = obs.getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();

		msg->latitude = o.fields.latitude_degrees;
		msg->longitude = o.fields.longitude_degrees;
		msg->altitude = o.fields.altitude_meters;

		if (auto& c = obs.covariance_enu; c.has_value())
		{
			msg->position_covariance_type =
				sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

			msg->position_covariance.fill(0.0);
			msg->position_covariance[0] = (*c)(0, 0);
			msg->position_covariance[4] = (*c)(1, 1);
			msg->position_covariance[8] = (*c)(2, 2);
		}

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
			for (size_t i = 0; i < dist.size(); i++) dist[i] = c.dist[i];
			break;

		case mrpt::img::DistortionModel::none:
			ci.distortion_model = "plumb_bob";
			dist.resize(5);
			for (size_t i = 0; i < dist.size(); i++) dist[i] = 0;
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
}  // namespace

void MVSimNode::internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservationImage& obs)
{
	using namespace std::string_literals;

	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	auto& pubs = pubsub_vehicles_[veh.getVehicleIndex()];

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
#else
		pubImg = mvsim_node::make_shared<PublisherWrapper<Msg_Image>>(
			n_, vehVarName(img_topic, veh), publisher_history_len_);
		pubCamInfo = mvsim_node::make_shared<PublisherWrapper<Msg_CameraInfo>>(
			n_, vehVarName(camInfo_topic, veh), publisher_history_len_);
#endif
	}
	lck.unlock();

	// Send TF:
	mrpt::poses::CPose3D sensorPose;
	obs.getSensorPose(sensorPose);
	auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.header.frame_id = "base_link";
	tfStmp.child_frame_id = obs.sensorLabel;
	tfStmp.header.stamp = myNow();

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tfStmp);
	pubs.pub_tf->publish(tfMsg);

	// Send observation:
	Msg_Header msg_header;
	msg_header.stamp = myNow();
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

	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	auto& pubs = pubsub_vehicles_[veh.getVehicleIndex()];

	const auto lbPoints = obs.sensorLabel + "_points"s;
	const auto lbImage = obs.sensorLabel + "_image"s;

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub = pubs.pub_sensors.find(lbPoints) == pubs.pub_sensors.end();

	auto& pubPts = pubs.pub_sensors[lbPoints];
	auto& pubImg = pubs.pub_sensors[lbImage];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pubImg = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_Image>(vehVarName(lbImage, veh), publisher_history_len_));
		pubPts = mvsim_node::make_shared<ros::Publisher>(
			n_.advertise<Msg_PointCloud2>(vehVarName(lbPoints, veh), publisher_history_len_));
#else
		pubImg = mvsim_node::make_shared<PublisherWrapper<Msg_Image>>(
			n_, vehVarName(lbImage, veh), publisher_history_len_);
		pubPts = mvsim_node::make_shared<PublisherWrapper<Msg_PointCloud2>>(
			n_, vehVarName(lbPoints, veh), publisher_history_len_);
#endif
	}
	lck.unlock();

	const auto now = myNow();

	// IMAGE
	// --------
	if (obs.hasIntensityImage)
	{
		// Send TF:
		mrpt::poses::CPose3D sensorPose = obs.sensorPose + obs.relativePoseIntensityWRTDepth;
		auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

		Msg_TransformStamped tfStmp;
		tfStmp.transform = tf2::toMsg(transform);
		tfStmp.header.frame_id = "base_link";
		tfStmp.child_frame_id = lbImage;
		tfStmp.header.stamp = now;

		Msg_TFMessage tfMsg;
		tfMsg.transforms.push_back(tfStmp);
		pubs.pub_tf->publish(tfMsg);

		// Send observation:
		{
			// Convert observation MRPT -> ROS
			Msg_Image msg_img;
			Msg_Header msg_header;
			msg_header.stamp = now;
			msg_header.frame_id = lbImage;
			msg_img = mrpt2ros::toROS(obs.intensityImage, msg_header);
			pubImg->publish(mvsim_node::make_shared<Msg_Image>(msg_img));
		}
	}

	// POINTS
	// --------
	if (obs.hasRangeImage)
	{
		// Send TF:
		mrpt::poses::CPose3D sensorPose = obs.sensorPose;
		auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

		Msg_TransformStamped tfStmp;
		tfStmp.transform = tf2::toMsg(transform);
		tfStmp.header.frame_id = "base_link";
		tfStmp.child_frame_id = lbPoints;
		tfStmp.header.stamp = now;

		Msg_TFMessage tfMsg;
		tfMsg.transforms.push_back(tfStmp);
		pubs.pub_tf->publish(tfMsg);

		// Send observation:
		{
			// Convert observation MRPT -> ROS
			Msg_PointCloud2 msg_pts;
			Msg_Header msg_header;
			msg_header.stamp = now;
			msg_header.frame_id = lbPoints;

			mrpt::obs::T3DPointsProjectionParams pp;
			pp.takeIntoAccountSensorPoseOnRobot = false;

			mrpt::maps::CSimplePointsMap pts;
			const_cast<mrpt::obs::CObservation3DRangeScan&>(obs).unprojectInto(pts, pp);
			mrpt2ros::toROS(pts, msg_header, msg_pts);
			pubPts->publish(mvsim_node::make_shared<Msg_PointCloud2>(msg_pts));
		}
	}
}

void MVSimNode::internalOn(
	const mvsim::VehicleBase& veh, const mrpt::obs::CObservationPointCloud& obs)
{
	using namespace std::string_literals;

	auto lck = mrpt::lockHelper(pubsub_vehicles_mtx_);
	auto& pubs = pubsub_vehicles_[veh.getVehicleIndex()];

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

	const auto now = myNow();

	// POINTS
	// --------

	// Send TF:
	mrpt::poses::CPose3D sensorPose = obs.sensorPose;
	auto transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.header.frame_id = "base_link";
	tfStmp.child_frame_id = lbPoints;
	tfStmp.header.stamp = now;

	Msg_TFMessage tfMsg;
	tfMsg.transforms.push_back(tfStmp);
	pubs.pub_tf->publish(tfMsg);

	// Send observation:
	{
		// Convert observation MRPT -> ROS
		Msg_PointCloud2 msg_pts;
		Msg_Header msg_header;
		msg_header.stamp = now;
		msg_header.frame_id = lbPoints;

#if defined(HAVE_POINTS_XYZIRT)
		if (auto* xyzirt = dynamic_cast<const mrpt::maps::CPointsMapXYZIRT*>(obs.pointcloud.get());
			xyzirt)
		{
			mrpt2ros::toROS(*xyzirt, msg_header, msg_pts);
		}
		else
#endif
			if (auto* xyzi = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(obs.pointcloud.get());
				xyzi)
		{
			mrpt2ros::toROS(*xyzi, msg_header, msg_pts);
		}
		else if (auto* sPts =
					 dynamic_cast<const mrpt::maps::CSimplePointsMap*>(obs.pointcloud.get());
				 sPts)
		{
			mrpt2ros::toROS(*sPts, msg_header, msg_pts);
		}
		else
		{
			THROW_EXCEPTION("Do not know how to handle this variant of CPointsMap");
		}

		pubPts->publish(mvsim_node::make_shared<Msg_PointCloud2>(msg_pts));
	}
}
