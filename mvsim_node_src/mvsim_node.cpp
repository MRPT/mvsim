/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>	 // kbhit()
#include <mvsim/WorldElements/OccupancyGridMap.h>

#if PACKAGE_ROS_VERSION == 1
// ===========================================
//                    ROS 1
// ===========================================
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrpt/ros1bridge/image.h>
#include <mrpt/ros1bridge/laser_scan.h>
#include <mrpt/ros1bridge/map.h>
#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/pose.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// usings:
using Msg_OccupancyGrid = nav_msgs::OccupancyGrid;
using Msg_MapMetaData = nav_msgs::MapMetaData;
using Msg_TransformStamped = geometry_msgs::TransformStamped;
#else
// ===========================================
//                    ROS 2
// ===========================================
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/map.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/pose.h>

#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// see: https://github.com/ros2/geometry2/pull/416
#if defined(MVSIM_HAS_TF2_GEOMETRY_MSGS_HPP)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

// usings:
using Msg_OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Msg_MapMetaData = nav_msgs::msg::MapMetaData;
using Msg_TransformStamped = geometry_msgs::msg::TransformStamped;
#endif

#include <iostream>
#include <rapidxml_utils.hpp>

#include "mvsim/mvsim_node_core.h"

#if PACKAGE_ROS_VERSION == 1
namespace mrpt2ros = mrpt::ros1bridge;
#else
namespace mrpt2ros = mrpt::ros2bridge;
#endif

#if PACKAGE_ROS_VERSION == 1
#define ROS12_INFO(...) ROS_INFO(__VA_ARGS__)
#define ROS12_ERROR(...) ROS_ERROR(__VA_ARGS__)
#else
#define ROS12_INFO(...) RCLCPP_INFO(n_->get_logger(), __VA_ARGS__)
#define ROS12_ERROR(...) RCLCPP_ERROR(n_->get_logger(), __VA_ARGS__)
#endif

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
#if PACKAGE_ROS_VERSION == 2
	clock_ = n_->get_clock();
	ts_.attachClock(clock_);

	// ROS2: needs to declare parameters:
	n_->declare_parameter<std::string>("world_file", "default.world.xml");
	n_->declare_parameter<double>("simul_rate", 100);
	n_->declare_parameter<double>("base_watchdog_timeout", 0.2);
	{
		double t;
		base_watchdog_timeout_ = std::chrono::milliseconds(
			1000 * n_->get_parameter_or("base_watchdog_timeout", t, 0.2));
	}

	realtime_factor_ =
		n_->declare_parameter<double>("realtime_factor", realtime_factor_);

	gui_refresh_period_ms_ = n_->declare_parameter<double>(
		"gui_refresh_period", gui_refresh_period_ms_);

	headless_ = n_->declare_parameter<bool>("headless", headless_);

	period_ms_publish_tf_ = n_->declare_parameter<double>(
		"period_ms_publish_tf", period_ms_publish_tf_);

	do_fake_localization_ = n_->declare_parameter<bool>(
		"do_fake_localization", do_fake_localization_);

	// n_->declare_parameter("use_sim_time"); // already declared error?
#endif

	// Launch GUI thread:
	thread_params_.obj = this;
	thGUI_ =
		std::thread(&MVSimNode::thread_update_GUI, std::ref(thread_params_));

	// Init ROS publishers:
#if PACKAGE_ROS_VERSION == 1
	// pub_clock_ = n_.advertise<rosgraph_msgs::Clock>("/clock", 10);

	pub_map_ros_ = n_.advertise<nav_msgs::OccupancyGrid>(
		"simul_map", 1 /*queue len*/, true /*latch*/);
	pub_map_metadata_ = n_.advertise<nav_msgs::MapMetaData>(
		"simul_map_metadata", 1 /*queue len*/, true /*latch*/);
#else
	rclcpp::QoS qosLatched(rclcpp::KeepLast(10));
	qosLatched.durability(
		rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

	pub_map_ros_ = n_->create_publisher<nav_msgs::msg::OccupancyGrid>(
		"simul_map", qosLatched);
	pub_map_metadata_ = n_->create_publisher<nav_msgs::msg::MapMetaData>(
		"simul_map_metadata", qosLatched);
#endif

#if PACKAGE_ROS_VERSION == 1
	// sim_time_.fromSec(0.0);
	base_last_cmd_.fromSec(0.0);
#else
	// sim_time_ = rclcpp::Time(0);
	base_last_cmd_ = rclcpp::Time(0);
#endif

	// Node parameters:
#if PACKAGE_ROS_VERSION == 1
	double t;
	if (!localn_.getParam("base_watchdog_timeout", t)) t = 0.2;
	base_watchdog_timeout_.fromSec(t);
	localn_.param("realtime_factor", realtime_factor_, 1.0);
	localn_.param(
		"gui_refresh_period", gui_refresh_period_ms_, gui_refresh_period_ms_);
	localn_.param("headless", headless_, headless_);
	localn_.param(
		"period_ms_publish_tf", period_ms_publish_tf_, period_ms_publish_tf_);
	localn_.param(
		"do_fake_localization", do_fake_localization_, do_fake_localization_);

	// In case the user didn't set it:
	n_.setParam("/use_sim_time", true);
#endif

	mvsim_world_.registerCallbackOnObservation(
		[this](
			const mvsim::Simulable& veh,
			const mrpt::obs::CObservation::Ptr& obs) {
			if (!obs) return;

			mrpt::system::CTimeLoggerEntry tle(
				profiler_, "lambda_onNewObservation");

			const mvsim::Simulable* vehPtr = &veh;
			const mrpt::obs::CObservation::Ptr obsCopy = obs;
			auto fut = ros_publisher_workers_.enqueue([this, vehPtr,
													   obsCopy]() {
				try
				{
					onNewObservation(*vehPtr, obsCopy);
				}
				catch (const std::exception& e)
				{
					ROS12_ERROR(
						"[MVSimNode] Error processing observation with label "
						"'%s':\n%s",
						obsCopy ? obsCopy->sensorLabel.c_str() : "(nullptr)",
						e.what());
				}
			});
		});
}

void MVSimNode::launch_mvsim_server()
{
	ROS12_INFO("[MVSimNode] launch_mvsim_server()");

	ASSERT_(!mvsim_server_);

	// Start network server:
	mvsim_server_ = std::make_shared<mvsim::Server>();

#if 0
	 if (argPort.isSet()) server->listenningPort(argPort.getValue());
	mvsim_server_->setMinLoggingLevel(
		mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
			argVerbosity.getValue()));
#endif

	mvsim_server_->start();
}

void MVSimNode::loadWorldModel(const std::string& world_xml_file)
{
	ROS12_INFO("[MVSimNode] Loading world file: %s", world_xml_file.c_str());

	ASSERT_FILE_EXISTS_(world_xml_file);

	// Load from XML:
	rapidxml::file<> fil_xml(world_xml_file.c_str());
	mvsim_world_.load_from_XML(fil_xml.data(), world_xml_file);

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
{
	thread_params_.closing = true;
	if (thGUI_.joinable()) thGUI_.join();

	mvsim_world_.free_opengl_resources();
}

#if PACKAGE_ROS_VERSION == 1
/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/
void MVSimNode::configCallback(
	mvsim::mvsimNodeConfig& config, [[maybe_unused]] uint32_t level)
{
	// Set class variables to new values. They should match what is input at the
	// dynamic reconfigure GUI.
	//  message = config.message.c_str();
	ROS_INFO("MVSimNode::configCallback() called.");

	if (mvsim_world_.is_GUI_open() && !config.show_gui)
		mvsim_world_.close_GUI();
}
#endif

// Process pending msgs, run real-time simulation, etc.
void MVSimNode::spin()
{
	using namespace mvsim;
	using namespace std::string_literals;
	using mrpt::DEG2RAD;
	using mrpt::RAD2DEG;

	// Do simulation itself:
	// ========================================================================
	// Handle 1st iter:
	if (t_old_ < 0) t_old_ = realtime_tictac_.Tac();
	// Compute how much time has passed to simulate in real-time:
	double t_new = realtime_tictac_.Tac();
	double incr_time = realtime_factor_ * (t_new - t_old_);

	if (incr_time < mvsim_world_.get_simul_timestep())	// Just in case the
														// computer is *really
														// fast*...
		return;

	// Simulate:
	mvsim_world_.run_simulation(incr_time);

	// t_old_simul = world.get_simul_time();
	t_old_ = t_new;

	const auto& vehs = mvsim_world_.getListOfVehicles();

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
				"Selected vehicle: %u/%u\n",
				static_cast<unsigned>(teleop_idx_veh_ + 1),
				static_cast<unsigned>(vehs.size()));
			if (vehs.size() > teleop_idx_veh_)
			{
				// Get iterator to selected vehicle:
				auto it_veh = vehs.begin();
				std::advance(it_veh, teleop_idx_veh_);

				// Get speed: ground truth
				txt2gui_tmp += "gt. vel: "s +
							   it_veh->second->getVelocityLocal().asString();

				// Get speed: ground truth
				txt2gui_tmp +=
					"\nodo vel: "s +
					it_veh->second->getVelocityLocalOdoEstimate().asString();

				// Generic teleoperation interface for any controller that
				// supports it:
				{
					ControllerBaseInterface* controller =
						it_veh->second->getControllerInterface();
					ControllerBaseInterface::TeleopInput teleop_in;
					ControllerBaseInterface::TeleopOutput teleop_out;
					teleop_in.keycode = keyevent.keycode;
					controller->teleop_interface(teleop_in, teleop_out);
					txt2gui_tmp += teleop_out.append_gui_lines;
				}
			}
		}

		msg2gui_ = txt2gui_tmp;	 // send txt msgs to show in the GUI

		// Clear the keystroke buffer
		if (keyevent.keycode != 0) gui_key_events_ = World::TGUIKeyEvent();

	}  // end refresh teleop stuff
}

/*------------------------------------------------------------------------------
 * thread_update_GUI()
 *----------------------------------------------------------------------------*/
void MVSimNode::thread_update_GUI(TThreadParams& thread_params)
{
	try
	{
		using namespace mvsim;

		MVSimNode* obj = thread_params.obj;

		while (!thread_params.closing)
		{
			if (obj->world_init_ok_ && !obj->headless_)
			{
				World::TUpdateGUIParams guiparams;
				guiparams.msg_lines = obj->msg2gui_;

				obj->mvsim_world_.update_GUI(&guiparams);

				// Send key-strokes to the main thread:
				if (guiparams.keyevent.keycode != 0)
					obj->gui_key_events_ = guiparams.keyevent;

				std::this_thread::sleep_for(
					std::chrono::milliseconds(obj->gui_refresh_period_ms_));
			}
			else if (obj->world_init_ok_ && obj->headless_)
			{
				obj->mvsim_world_.internalGraphicsLoopTasksForSimulation();

				std::this_thread::sleep_for(
					std::chrono::microseconds(static_cast<size_t>(
						obj->mvsim_world_.get_simul_timestep() * 1000000)));
			}
			else
			{
				std::this_thread::sleep_for(
					std::chrono::milliseconds(obj->gui_refresh_period_ms_));
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
	if (mvsim::OccupancyGridMap* grid =
			dynamic_cast<mvsim::OccupancyGridMap*>(&obj);
		grid)
	{
		static Msg_OccupancyGrid ros_map;
		static mvsim::OccupancyGridMap* cachedGrid = nullptr;

		if (cachedGrid != grid)
		{
			cachedGrid = grid;
			mrpt2ros::toROS(grid->getOccGrid(), ros_map);
		}

#if PACKAGE_ROS_VERSION == 1
		static size_t loop_count = 0;
		ros_map.header.stamp = ros::Time::now();
		ros_map.header.seq = loop_count++;
#else
		ros_map.header.stamp = clock_->now();
		ros_map.header.frame_id = "map";
#endif

#if PACKAGE_ROS_VERSION == 1
		auto& pubMap = pub_map_ros_;
		auto& pubMapMeta = pub_map_metadata_;
#else
		auto& pubMap = *pub_map_ros_;
		auto& pubMapMeta = *pub_map_metadata_;
#endif
		pubMap.publish(ros_map);
		pubMapMeta.publish(ros_map.info);

	}  // end gridmap

}  // end visit(World Elements)

// ROS: Publish grid map for visualization purposes:
void MVSimNode::notifyROSWorldIsUpdated()
{
#if PACKAGE_ROS_VERSION == 2
	// In ROS1 latching works so we only need to do this once, here.
	// In ROS2,latching doesn't work, we must re-publish on a regular basis...
	static mrpt::system::CTicTac lastMapPublished;
	if (lastMapPublished.Tac() > 2.0)
	{
		mvsim_world_.runVisitorOnWorldElements(
			[this](mvsim::WorldElementBase& obj) {
				publishWorldElements(obj);
			});
		lastMapPublished.Tic();
	}
#endif

	mvsim_world_.runVisitorOnVehicles(
		[this](mvsim::VehicleBase& v) { publishVehicles(v); });

	// Create subscribers & publishers for each vehicle's stuff:
	// ----------------------------------------------------
	auto& vehs = mvsim_world_.getListOfVehicles();
	pubsub_vehicles_.clear();
	pubsub_vehicles_.resize(vehs.size());
	size_t idx = 0;
	for (auto it = vehs.begin(); it != vehs.end(); ++it, ++idx)
	{
		mvsim::VehicleBase* veh =
			dynamic_cast<mvsim::VehicleBase*>(it->second.get());
		if (!veh) continue;

		initPubSubs(pubsub_vehicles_[idx], veh);
	}

	// Publish the static transform /world -> /map
	sendStaticTF("world", "map", tfIdentity_, myNow());
}

#if PACKAGE_ROS_VERSION == 1
ros::Time MVSimNode::myNow() const { return ros::Time::now(); }
#else
rclcpp::Time MVSimNode::myNow() const { return n_->get_clock()->now(); }
#endif

void MVSimNode::sendStaticTF(
	const std::string& frame_id, const std::string& child_frame_id,
	const tf2::Transform& txf,
#if PACKAGE_ROS_VERSION == 1
	const ros::Time& stamp
#else
	const rclcpp::Time& stamp
#endif
)
{
	Msg_TransformStamped tx;
	tx.header.frame_id = frame_id;
	tx.child_frame_id = child_frame_id;
	tx.header.stamp = stamp;
	tx.transform = tf2::toMsg(txf);
	static_tf_br_.sendTransform(tx);
}

/** Initialize all pub/subs required for each vehicle, for the specific vehicle
 * \a veh */
void MVSimNode::initPubSubs(TPubSubPerVehicle& pubsubs, mvsim::VehicleBase* veh)
{
	// sub: <VEH>/cmd_vel
#if PACKAGE_ROS_VERSION == 1
	pubsubs.sub_cmd_vel = n_.subscribe<geometry_msgs::Twist>(
		vehVarName("cmd_vel", *veh), 10,
		boost::bind(&MVSimNode::onROSMsgCmdVel, this, _1, veh));
#else
	using std::placeholders::_1;

	pubsubs.sub_cmd_vel = n_->create_subscription<geometry_msgs::msg::Twist>(
		vehVarName("cmd_vel", *veh), 10,
		[this, veh](const geometry_msgs::msg::Twist::SharedPtr msg) {
			return this->onROSMsgCmdVel(msg, veh);
		});
#endif

#if PACKAGE_ROS_VERSION == 1
	// pub: <VEH>/odom
	pubsubs.pub_odom =
		n_.advertise<nav_msgs::Odometry>(vehVarName("odom", *veh), 10);

	// pub: <VEH>/base_pose_ground_truth
	pubsubs.pub_ground_truth = n_.advertise<nav_msgs::Odometry>(
		vehVarName("base_pose_ground_truth", *veh), 10);
#else
	// pub: <VEH>/odom
	pubsubs.pub_odom = n_->create_publisher<nav_msgs::msg::Odometry>(
		vehVarName("odom", *veh), 10);
	// pub: <VEH>/base_pose_ground_truth
	pubsubs.pub_ground_truth = n_->create_publisher<nav_msgs::msg::Odometry>(
		vehVarName("base_pose_ground_truth", *veh), 10);
#endif

	// pub: <VEH>/chassis_markers
	{
#if PACKAGE_ROS_VERSION == 1
		pubsubs.pub_chassis_markers =
			n_.advertise<visualization_msgs::MarkerArray>(
				vehVarName("chassis_markers", *veh), 5, true /*latch*/);
#else
		rclcpp::QoS qosLatched(rclcpp::KeepLast(5));
		qosLatched.durability(rmw_qos_durability_policy_t::
								  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

		pubsubs.pub_chassis_markers =
			n_->create_publisher<visualization_msgs::msg::MarkerArray>(
				vehVarName("chassis_markers", *veh), qosLatched);
#endif
		const mrpt::math::TPolygon2D& poly = veh->getChassisShape();

		// Create one "ROS marker" for each wheel + 1 for the chassis:
		auto& msg_shapes = pubsubs.chassis_shape_msg;
		msg_shapes.markers.resize(1 + veh->getNumWheels());

		// [0] Chassis shape:
		auto& chassis_shape_msg = msg_shapes.markers[0];

		chassis_shape_msg.pose =
			mrpt2ros::toROS_Pose(mrpt::poses::CPose3D::Identity());

#if PACKAGE_ROS_VERSION == 1
		chassis_shape_msg.action = visualization_msgs::Marker::MODIFY;
		chassis_shape_msg.type = visualization_msgs::Marker::LINE_STRIP;
#else
		chassis_shape_msg.action = visualization_msgs::msg::Marker::MODIFY;
		chassis_shape_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
#endif

		chassis_shape_msg.header.frame_id = vehVarName("base_link", *veh);
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
			const mvsim::Wheel& w = veh->getWheelInfo(i);

			const double lx = w.diameter * 0.5, ly = w.width * 0.5;

			// Init values. Copy the contents from the chassis msg
			wheel_shape_msg = msg_shapes.markers[0];

			chassis_shape_msg.ns = mrpt::format(
				"mvsim.chassis_shape.wheel%u", static_cast<unsigned int>(i));
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

			wheel_shape_msg.color.r = w.color.R / 255.0;
			wheel_shape_msg.color.g = w.color.G / 255.0;
			wheel_shape_msg.color.b = w.color.B / 255.0;

			// Set local pose of the wheel wrt the vehicle:
			wheel_shape_msg.pose = mrpt2ros::toROS_Pose(w.pose());
		}  // end for each wheel

		// Publish Initial pose
#if PACKAGE_ROS_VERSION == 1
		pubsubs.pub_chassis_markers.publish(msg_shapes);
#else
		pubsubs.pub_chassis_markers->publish(msg_shapes);
#endif
	}

	// pub: <VEH>/chassis_polygon
	{
#if PACKAGE_ROS_VERSION == 1
		pubsubs.pub_chassis_shape = n_.advertise<geometry_msgs::Polygon>(
			vehVarName("chassis_polygon", *veh), 1, true /*latch*/);

		geometry_msgs::Polygon poly_msg;
#else
		rclcpp::QoS qosLatched(rclcpp::KeepLast(1));
		qosLatched.durability(rmw_qos_durability_policy_t::
								  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

		pubsubs.pub_chassis_shape =
			n_->create_publisher<geometry_msgs::msg::Polygon>(
				vehVarName("chassis_polygon", *veh), qosLatched);

		geometry_msgs::msg::Polygon poly_msg;
#endif
		// Do the first (and unique) publish:
		const mrpt::math::TPolygon2D& poly = veh->getChassisShape();
		poly_msg.points.resize(poly.size());
		for (size_t i = 0; i < poly.size(); i++)
		{
			poly_msg.points[i].x = poly[i].x;
			poly_msg.points[i].y = poly[i].y;
			poly_msg.points[i].z = 0;
		}
#if PACKAGE_ROS_VERSION == 1
		pubsubs.pub_chassis_shape.publish(poly_msg);
#else
		pubsubs.pub_chassis_shape->publish(poly_msg);
#endif
	}

	if (do_fake_localization_)
	{
#if PACKAGE_ROS_VERSION == 1
		// pub: <VEH>/amcl_pose
		pubsubs.pub_amcl_pose =
			n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
				vehVarName("amcl_pose", *veh), 1);
		// pub: <VEH>/particlecloud
		pubsubs.pub_particlecloud = n_.advertise<geometry_msgs::PoseArray>(
			vehVarName("particlecloud", *veh), 1);
#else
		// pub: <VEH>/amcl_pose
		pubsubs.pub_amcl_pose =
			n_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
				vehVarName("amcl_pose", *veh), 1);
		// pub: <VEH>/particlecloud
		pubsubs.pub_particlecloud =
			n_->create_publisher<geometry_msgs::msg::PoseArray>(
				vehVarName("particlecloud", *veh), 1);
#endif
	}

	// STATIC Identity transform <VEH>/base_link -> <VEH>/base_footprint
	sendStaticTF(
		vehVarName("base_link", *veh), vehVarName("base_footprint", *veh),
		tfIdentity_, myNow());
}

void MVSimNode::onROSMsgCmdVel(
#if PACKAGE_ROS_VERSION == 1
	const geometry_msgs::Twist::ConstPtr& cmd,
#else
	const geometry_msgs::msg::Twist::SharedPtr cmd,
#endif
	mvsim::VehicleBase* veh)
{
	mvsim::ControllerBaseInterface* controller = veh->getControllerInterface();

	const bool ctrlAcceptTwist = controller->setTwistCommand(
		{cmd->linear.x, cmd->linear.y, cmd->angular.z});

	if (!ctrlAcceptTwist)
	{
#if PACKAGE_ROS_VERSION == 1
		ROS_WARN_THROTTLE(
			1.0,
			"*Warning* Vehicle's controller ['%s'] refuses Twist commands!",
			veh->getName().c_str());
#else
		RCLCPP_WARN_THROTTLE(
			n_->get_logger(), *n_->get_clock(), 1.0,
			"*Warning* Vehicle's controller ['%s'] refuses Twist commands!",
			veh->getName().c_str());
#endif
	}
}

/** Publish everything to be published at each simulation iteration */
void MVSimNode::spinNotifyROS()
{
	using namespace mvsim;
	const auto& vehs = mvsim_world_.getListOfVehicles();

	// Get current simulation time (for messages) and publish "/clock"
	// ----------------------------------------------------------------
#if PACKAGE_ROS_VERSION == 1
	// sim_time_.fromSec(mvsim_world_.get_simul_time());
	// clockMsg_.clock = sim_time_;
	// pub_clock_.publish(clockMsg_);
#else
	// sim_time_ = n_->get_clock()->now();
	// MRPT_TODO("Publish /clock for ROS2 too?");
#endif

#if PACKAGE_ROS_VERSION == 2
	// In ROS2,latching doesn't work, we must re-publish on a regular basis...
	mvsim_world_.runVisitorOnWorldElements(
		[this](mvsim::WorldElementBase& obj) { publishWorldElements(obj); });
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
			const VehicleBase::Ptr& veh = it->second;

			const std::string sOdomName = vehVarName("odom", *veh);
			const std::string sBaseLinkFrame = vehVarName("base_link", *veh);

			// 1) Ground-truth pose and velocity
			// --------------------------------------------
			const mrpt::math::TPose3D& gh_veh_pose = veh->getPose();
			// [vx,vy,w] in global frame
			const auto& gh_veh_vel = veh->getTwist();

			{
#if PACKAGE_ROS_VERSION == 1
				nav_msgs::Odometry gtOdoMsg;
#else
				nav_msgs::msg::Odometry gtOdoMsg;
#endif

				gtOdoMsg.pose.pose = mrpt2ros::toROS_Pose(gh_veh_pose);

				gtOdoMsg.twist.twist.linear.x = gh_veh_vel.vx;
				gtOdoMsg.twist.twist.linear.y = gh_veh_vel.vy;
				gtOdoMsg.twist.twist.linear.z = 0;
				gtOdoMsg.twist.twist.angular.z = gh_veh_vel.omega;

				gtOdoMsg.header.stamp = myNow();
				gtOdoMsg.header.frame_id = sOdomName;
				gtOdoMsg.child_frame_id = sBaseLinkFrame;

#if PACKAGE_ROS_VERSION == 1
				pubsub_vehicles_[i].pub_ground_truth.publish(gtOdoMsg);
#else
				pubsub_vehicles_[i].pub_ground_truth->publish(gtOdoMsg);
#endif
				if (do_fake_localization_)
				{
#if PACKAGE_ROS_VERSION == 1
					geometry_msgs::PoseWithCovarianceStamped currentPos;
					geometry_msgs::PoseArray particleCloud;
#else
					geometry_msgs::msg::PoseWithCovarianceStamped currentPos;
					geometry_msgs::msg::PoseArray particleCloud;
#endif

					// topic: <Ri>/particlecloud
					{
						particleCloud.header.stamp = myNow();
						particleCloud.header.frame_id = "map";
						particleCloud.poses.resize(1);
						particleCloud.poses[0] = gtOdoMsg.pose.pose;
#if PACKAGE_ROS_VERSION == 1
						pubsub_vehicles_[i].pub_particlecloud.publish(
							particleCloud);
#else
						pubsub_vehicles_[i].pub_particlecloud->publish(
							particleCloud);
#endif
					}

					// topic: <Ri>/amcl_pose
					{
						currentPos.header = gtOdoMsg.header;
						currentPos.pose.pose = gtOdoMsg.pose.pose;
#if PACKAGE_ROS_VERSION == 1
						pubsub_vehicles_[i].pub_amcl_pose.publish(currentPos);
#else
						pubsub_vehicles_[i].pub_amcl_pose->publish(currentPos);
#endif
					}

					// TF: /map -> <Ri>/odom
					{
						MRPT_TODO(
							"Save initial pose for each vehicle, set odometry "
							"from that pose");

						Msg_TransformStamped tx;
						tx.header.frame_id = "map";
						tx.child_frame_id = sOdomName;
						tx.header.stamp = myNow();
						tx.transform =
							tf2::toMsg(tf2::Transform::getIdentity());

						tf_br_.sendTransform(tx);
					}
				}
			}

			// 2) Chassis markers (for rviz visualization)
			// --------------------------------------------
			// pub: <VEH>/chassis_markers
			{
				// visualization_msgs::MarkerArray
				auto& msg_shapes = pubsub_vehicles_[i].chassis_shape_msg;
				ASSERT_EQUAL_(
					msg_shapes.markers.size(), (1 + veh->getNumWheels()));

				// [0] Chassis shape: static no need to update.
				// [1:N] Wheel shapes: may move
				for (size_t j = 0; j < veh->getNumWheels(); j++)
				{
					// visualization_msgs::Marker
					auto& wheel_shape_msg = msg_shapes.markers[1 + j];
					const mvsim::Wheel& w = veh->getWheelInfo(j);

					// Set local pose of the wheel wrt the vehicle:
					wheel_shape_msg.pose = mrpt2ros::toROS_Pose(w.pose());

				}  // end for each wheel

				// Publish Initial pose
#if PACKAGE_ROS_VERSION == 1
				pubsub_vehicles_[i].pub_chassis_markers.publish(msg_shapes);
#else
				pubsub_vehicles_[i].pub_chassis_markers->publish(msg_shapes);
#endif
			}

			// 3) odometry transform
			// --------------------------------------------
			{
				const mrpt::math::TPose3D odo_pose = gh_veh_pose;

				{
					Msg_TransformStamped tx;
					tx.header.frame_id = sOdomName;
					tx.child_frame_id = sBaseLinkFrame;
					tx.header.stamp = myNow();
					tx.transform =
						tf2::toMsg(mrpt2ros::toROS_tfTransform(odo_pose));
					tf_br_.sendTransform(tx);
				}

				// Apart from TF, publish to the "odom" topic as well
				{
#if PACKAGE_ROS_VERSION == 1
					nav_msgs::Odometry odoMsg;
#else
					nav_msgs::msg::Odometry odoMsg;
#endif
					odoMsg.pose.pose = mrpt2ros::toROS_Pose(odo_pose);

					// first, we'll populate the header for the odometry msg
					odoMsg.header.stamp = myNow();
					odoMsg.header.frame_id = sOdomName;
					odoMsg.child_frame_id = sBaseLinkFrame;

					// publish:
#if PACKAGE_ROS_VERSION == 1
					pubsub_vehicles_[i].pub_odom.publish(odoMsg);
#else
					pubsub_vehicles_[i].pub_odom->publish(odoMsg);
#endif
				}
			}

		}  // end for each vehicle

	}  // end publish tf

}  // end spinNotifyROS()

void MVSimNode::onNewObservation(
	const mvsim::Simulable& sim, const mrpt::obs::CObservation::Ptr& obs)
{
	mrpt::system::CTimeLoggerEntry tle(profiler_, "onNewObservation");

	using mrpt::obs::CObservation2DRangeScan;

	ASSERT_(obs);
	ASSERT_(!obs->sensorLabel.empty());

	const auto& vehPtr = dynamic_cast<const mvsim::VehicleBase*>(&sim);
	if (!vehPtr) return;  // for example, if obs from invisible aux block.
	const auto& veh = *vehPtr;

	// -----------------------------
	// Observation: 2d laser scans
	// -----------------------------
	if (const auto* o2DLidar =
			dynamic_cast<const CObservation2DRangeScan*>(obs.get());
		o2DLidar)
	{
		internalOn(veh, *o2DLidar);
	}
	else if (const auto* oImage =
				 dynamic_cast<const mrpt::obs::CObservationImage*>(obs.get());
			 oImage)
	{
		internalOn(veh, *oImage);
	}
	else if (const auto* oRGBD =
				 dynamic_cast<const mrpt::obs::CObservation3DRangeScan*>(
					 obs.get());
			 oRGBD)
	{
		internalOn(veh, *oRGBD);
	}
	else if (const auto* oPC =
				 dynamic_cast<const mrpt::obs::CObservationPointCloud*>(
					 obs.get());
			 oPC)
	{
		internalOn(veh, *oPC);
	}
	else
	{
		// Don't know how to emit this observation to ROS!
#if PACKAGE_ROS_VERSION == 1
		ROS_WARN_STREAM_THROTTLE(
			1.0, "Do not know how to publish this observation to ROS: '"
					 << obs->sensorLabel
					 << "', class: " << obs->GetRuntimeClass()->className);
#else
		RCLCPP_WARN_STREAM_THROTTLE(
			n_->get_logger(), *n_->get_clock(), 1.0,
			"Do not know how to publish this observation to ROS: '"
				<< obs->sensorLabel
				<< "', class: " << obs->GetRuntimeClass()->className);
#endif
	}

}  // end of onNewObservation()

/** Creates the string "/<VEH_NAME>/<VAR_NAME>" if there're more than one
 * vehicle in the World, or "/<VAR_NAME>" otherwise. */
std::string MVSimNode::vehVarName(
	const std::string& sVarName, const mvsim::VehicleBase& veh) const
{
	using namespace std::string_literals;

	if (mvsim_world_.getListOfVehicles().size() == 1)
	{
		return sVarName;
	}
	else
	{
		return veh.getName() + std::string("/") + sVarName;
	}
}

void MVSimNode::internalOn(
	const mvsim::VehicleBase& veh,
	const mrpt::obs::CObservation2DRangeScan& obs)
{
	TPubSubPerVehicle& pubs = pubsub_vehicles_[veh.getVehicleIndex()];

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub =
		pubs.pub_sensors.find(obs.sensorLabel) == pubs.pub_sensors.end();
	auto& pub = pubs.pub_sensors[obs.sensorLabel];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pub = n_.advertise<sensor_msgs::LaserScan>(
			vehVarName(obs.sensorLabel, veh), 10);
#else
		pub = n_->create_publisher<sensor_msgs::msg::LaserScan>(
			vehVarName(obs.sensorLabel, veh), 10);
#endif
	}

#if PACKAGE_ROS_VERSION == 2
	rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubLidar =
		std::dynamic_pointer_cast<
			rclcpp::Publisher<sensor_msgs::msg::LaserScan>>(pub);
	ASSERT_(pubLidar);
#endif

	const std::string sSensorFrameId = vehVarName(obs.sensorLabel, veh);

	// Send TF:
	mrpt::poses::CPose3D sensorPose;
	obs.getSensorPose(sensorPose);

	tf2::Transform transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.child_frame_id = sSensorFrameId;
	tfStmp.header.frame_id = vehVarName("base_link", veh);
	tfStmp.header.stamp = myNow();
	tf_br_.sendTransform(tfStmp);

	// Send observation:
	{
		// Convert observation MRPT -> ROS
#if PACKAGE_ROS_VERSION == 1
		geometry_msgs::Pose msg_pose_laser;
		sensor_msgs::LaserScan msg_laser;
#else
		geometry_msgs::msg::Pose msg_pose_laser;
		sensor_msgs::msg::LaserScan msg_laser;
#endif
		mrpt2ros::toROS(obs, msg_laser, msg_pose_laser);

		// Force usage of simulation time:
		msg_laser.header.stamp = myNow();
		msg_laser.header.frame_id = sSensorFrameId;

#if PACKAGE_ROS_VERSION == 1
		pub.publish(msg_laser);
#else
		pubLidar->publish(msg_laser);
#endif
	}
}

void MVSimNode::internalOn(
	const mvsim::VehicleBase& veh, const mrpt::obs::CObservationImage& obs)
{
	TPubSubPerVehicle& pubs = pubsub_vehicles_[veh.getVehicleIndex()];

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub =
		pubs.pub_sensors.find(obs.sensorLabel) == pubs.pub_sensors.end();
	auto& pub = pubs.pub_sensors[obs.sensorLabel];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pub = n_.advertise<sensor_msgs::Image>(
			vehVarName(obs.sensorLabel, veh), 10);
#else
		pub = n_->create_publisher<sensor_msgs::msg::Image>(
			vehVarName(obs.sensorLabel, veh), 10);
#endif
	}

#if PACKAGE_ROS_VERSION == 2
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImg =
		std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Image>>(
			pub);
	ASSERT_(pubImg);
#endif

	const std::string sSensorFrameId = vehVarName(obs.sensorLabel, veh);

	// Send TF:
	mrpt::poses::CPose3D sensorPose;
	obs.getSensorPose(sensorPose);

	tf2::Transform transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.child_frame_id = sSensorFrameId;
	tfStmp.header.frame_id = vehVarName("base_link", veh);
	tfStmp.header.stamp = myNow();
	tf_br_.sendTransform(tfStmp);

	// Send observation:
	{
		// Convert observation MRPT -> ROS
#if PACKAGE_ROS_VERSION == 1
		sensor_msgs::Image msg_img;
		std_msgs::Header msg_header;
#else
		sensor_msgs::msg::Image msg_img;
		std_msgs::msg::Header msg_header;
#endif
		msg_header.stamp = myNow();
		msg_header.frame_id = sSensorFrameId;

		msg_img = mrpt2ros::toROS(obs.image, msg_header);

#if PACKAGE_ROS_VERSION == 1
		pub.publish(msg_img);
#else
		pubImg->publish(msg_img);
#endif
	}
}

void MVSimNode::internalOn(
	const mvsim::VehicleBase& veh,
	const mrpt::obs::CObservation3DRangeScan& obs)
{
	using namespace std::string_literals;

	TPubSubPerVehicle& pubs = pubsub_vehicles_[veh.getVehicleIndex()];

	const auto lbPoints = obs.sensorLabel + "_points"s;
	const auto lbImage = obs.sensorLabel + "_image"s;

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub =
		pubs.pub_sensors.find(lbPoints) == pubs.pub_sensors.end();

	auto& pubPts = pubs.pub_sensors[lbPoints];
	auto& pubImg = pubs.pub_sensors[lbImage];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pubPts = n_.advertise<sensor_msgs::PointCloud2>(
			vehVarName(lbPoints, veh), 10);
		pubImg = n_.advertise<sensor_msgs::Image>(vehVarName(lbImage, veh), 10);
#else
		pubImg = n_->create_publisher<sensor_msgs::msg::Image>(
			vehVarName(lbImage, veh), 10);
		pubPts = n_->create_publisher<sensor_msgs::msg::PointCloud2>(
			vehVarName(lbPoints, veh), 10);
#endif
	}

#if PACKAGE_ROS_VERSION == 2
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPoints =
		std::dynamic_pointer_cast<
			rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(pubPts);
	ASSERT_(pubPoints);

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pubImage =
		std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Image>>(
			pubImg);
	ASSERT_(pubImage);
#endif

	const std::string sSensorFrameId_image = vehVarName(lbImage, veh);
	const std::string sSensorFrameId_points = vehVarName(lbPoints, veh);

	const auto now = myNow();

	// IMAGE
	// --------
	if (obs.hasIntensityImage)
	{
		// Send TF:
		mrpt::poses::CPose3D sensorPose =
			obs.sensorPose + obs.relativePoseIntensityWRTDepth;

		tf2::Transform transform = mrpt2ros::toROS_tfTransform(sensorPose);

		Msg_TransformStamped tfStmp;
		tfStmp.transform = tf2::toMsg(transform);
		tfStmp.child_frame_id = sSensorFrameId_image;
		tfStmp.header.frame_id = vehVarName("base_link", veh);
		tfStmp.header.stamp = now;
		tf_br_.sendTransform(tfStmp);

		// Send observation:
		{
			// Convert observation MRPT -> ROS
#if PACKAGE_ROS_VERSION == 1
			sensor_msgs::Image msg_img;
			std_msgs::Header msg_header;
#else
			sensor_msgs::msg::Image msg_img;
			std_msgs::msg::Header msg_header;
#endif
			msg_header.stamp = now;
			msg_header.frame_id = sSensorFrameId_image;

			msg_img = mrpt2ros::toROS(obs.intensityImage, msg_header);

#if PACKAGE_ROS_VERSION == 1
			pubImg.publish(msg_img);
#else
			pubImage->publish(msg_img);
#endif
		}
	}

	// POINTS
	// --------
	if (obs.hasRangeImage)
	{
		// Send TF:
		mrpt::poses::CPose3D sensorPose = obs.sensorPose;

		tf2::Transform transform = mrpt2ros::toROS_tfTransform(sensorPose);

		Msg_TransformStamped tfStmp;
		tfStmp.transform = tf2::toMsg(transform);
		tfStmp.child_frame_id = sSensorFrameId_points;
		tfStmp.header.frame_id = vehVarName("base_link", veh);
		tfStmp.header.stamp = now;
		tf_br_.sendTransform(tfStmp);

		// Send observation:
		{
			// Convert observation MRPT -> ROS
#if PACKAGE_ROS_VERSION == 1
			sensor_msgs::PointCloud2 msg_pts;
			std_msgs::Header msg_header;
#else
			sensor_msgs::msg::PointCloud2 msg_pts;
			std_msgs::msg::Header msg_header;
#endif
			msg_header.stamp = now;
			msg_header.frame_id = sSensorFrameId_points;

			mrpt::obs::T3DPointsProjectionParams pp;
			pp.takeIntoAccountSensorPoseOnRobot = false;

			mrpt::maps::CSimplePointsMap pts;
			const_cast<mrpt::obs::CObservation3DRangeScan&>(obs).unprojectInto(
				pts, pp);

			mrpt2ros::toROS(pts, msg_header, msg_pts);

#if PACKAGE_ROS_VERSION == 1
			pubPts.publish(msg_pts);
#else
			pubPoints->publish(msg_pts);
#endif
		}
	}
}

void MVSimNode::internalOn(
	const mvsim::VehicleBase& veh, const mrpt::obs::CObservationPointCloud& obs)
{
	using namespace std::string_literals;

	TPubSubPerVehicle& pubs = pubsub_vehicles_[veh.getVehicleIndex()];

	const auto lbPoints = obs.sensorLabel + "_points"s;

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub =
		pubs.pub_sensors.find(lbPoints) == pubs.pub_sensors.end();

	auto& pubPts = pubs.pub_sensors[lbPoints];

	if (is_1st_pub)
	{
#if PACKAGE_ROS_VERSION == 1
		pubPts = n_.advertise<sensor_msgs::PointCloud2>(
			vehVarName(lbPoints, veh), 10);
#else
		pubPts = n_->create_publisher<sensor_msgs::msg::PointCloud2>(
			vehVarName(lbPoints, veh), 10);
#endif
	}

#if PACKAGE_ROS_VERSION == 2
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPoints =
		std::dynamic_pointer_cast<
			rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(pubPts);
	ASSERT_(pubPoints);
#endif

	const std::string sSensorFrameId_points = vehVarName(lbPoints, veh);

	const auto now = myNow();

	// POINTS
	// --------

	// Send TF:
	mrpt::poses::CPose3D sensorPose = obs.sensorPose;

	tf2::Transform transform = mrpt2ros::toROS_tfTransform(sensorPose);

	Msg_TransformStamped tfStmp;
	tfStmp.transform = tf2::toMsg(transform);
	tfStmp.child_frame_id = sSensorFrameId_points;
	tfStmp.header.frame_id = vehVarName("base_link", veh);
	tfStmp.header.stamp = now;
	tf_br_.sendTransform(tfStmp);

	// Send observation:
	{
		// Convert observation MRPT -> ROS
#if PACKAGE_ROS_VERSION == 1
		sensor_msgs::PointCloud2 msg_pts;
		std_msgs::Header msg_header;
#else
		sensor_msgs::msg::PointCloud2 msg_pts;
		std_msgs::msg::Header msg_header;
#endif
		msg_header.stamp = now;
		msg_header.frame_id = sSensorFrameId_points;

		mrpt::obs::T3DPointsProjectionParams pp;
		pp.takeIntoAccountSensorPoseOnRobot = false;

		if (auto* sPts = dynamic_cast<const mrpt::maps::CSimplePointsMap*>(
				obs.pointcloud.get());
			sPts)
		{
			mrpt2ros::toROS(*sPts, msg_header, msg_pts);
		}
		else if (auto* xyzi = dynamic_cast<const mrpt::maps::CPointsMapXYZI*>(
					 obs.pointcloud.get());
				 xyzi)
		{
			mrpt2ros::toROS(*xyzi, msg_header, msg_pts);
		}
		else
		{
			THROW_EXCEPTION(
				"Do not know how to handle this variant of CPointsMap");
		}

#if PACKAGE_ROS_VERSION == 1
		pubPts.publish(msg_pts);
#else
		pubPoints->publish(msg_pts);
#endif
	}
}