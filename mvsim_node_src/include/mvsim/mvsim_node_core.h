/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mvsim/Comms/Server.h>
#include <mvsim/World.h>
#include <tf2/LinearMath/Transform.h>

#include <atomic>
#include <thread>

#if PACKAGE_ROS_VERSION == 1
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <mvsim/mvsimNodeConfig.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <tf2_msgs/TFMessage.h>
#include <visualization_msgs/MarkerArray.h>

// usings:
using ros_Time = ros::Time;
using ros_Duration = ros::Duration;

using Msg_Polygon = geometry_msgs::Polygon;
using Msg_PoseArray = geometry_msgs::PoseArray;
using Msg_PoseWithCovarianceStamped = geometry_msgs::PoseWithCovarianceStamped;
using Msg_Twist = geometry_msgs::Twist;
using Msg_Twist_CSPtr = geometry_msgs::Twist::ConstPtr;
using Msg_OccupancyGrid = nav_msgs::OccupancyGrid;
using Msg_Odometry = nav_msgs::Odometry;
using Msg_MapMetaData = nav_msgs::MapMetaData;
using Msg_Bool = std_msgs::Bool;
using Msg_TFMessage = tf2_msgs::TFMessage;
using Msg_MarkerArray = visualization_msgs::MarkerArray;
using Msg_CameraInfo = sensor_msgs::CameraInfo;
#else
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wrapper/publisher_wrapper.h"

// usings:
using ros_Time = rclcpp::Time;
using ros_Duration = rclcpp::Duration;

using Msg_Polygon = geometry_msgs::msg::Polygon;
using Msg_PoseArray = geometry_msgs::msg::PoseArray;
using Msg_PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using Msg_Twist = geometry_msgs::msg::Twist;
using Msg_Twist_CSPtr = geometry_msgs::msg::Twist::ConstSharedPtr;
using Msg_OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Msg_Odometry = nav_msgs::msg::Odometry;
using Msg_MapMetaData = nav_msgs::msg::MapMetaData;
using Msg_Bool = std_msgs::msg::Bool;
using Msg_TFMessage = tf2_msgs::msg::TFMessage;
using Msg_MarkerArray = visualization_msgs::msg::MarkerArray;
using Msg_CameraInfo = sensor_msgs::msg::CameraInfo;
#endif

namespace mvsim_node
{
#if PACKAGE_ROS_VERSION == 1
template <typename T, typename... Args>
boost::shared_ptr<T> make_shared(Args&&... args)
{
	return boost::make_shared<T>(std::forward<Args>(args)...);
}

template <typename T>
using shared_ptr = boost::shared_ptr<T>;
#else
template <typename T, typename... Args>
std::shared_ptr<T> make_shared(Args&&... args)
{
	return std::make_shared<T>(std::forward<Args>(args)...);
}

template <typename T>
using shared_ptr = std::shared_ptr<T>;
#endif
}  // namespace mvsim_node

/** A class to wrap libmvsim as a ROS node
 */
class MVSimNode
{
   public:
	/** Constructor. */
#if PACKAGE_ROS_VERSION == 1
	MVSimNode(ros::NodeHandle& n);
#else
	MVSimNode(rclcpp::Node::SharedPtr& n);
#endif

	/** Destructor. */
	~MVSimNode();

	void terminateSimulation();

	void launch_mvsim_server();

	void loadWorldModel(const std::string& world_xml_file);

	void spin();  //!< Process pending msgs, run real-time simulation, etc.

#if PACKAGE_ROS_VERSION == 1
	/** Callback function for dynamic reconfigure server */
	void configCallback(mvsim::mvsimNodeConfig& config, uint32_t level);
#endif

	void onNewObservation(const mvsim::Simulable& veh, const mrpt::obs::CObservation::Ptr& obs);

	/// The mvsim library simulated world (includes everything: vehicles,
	/// obstacles, etc.)
	mvsim_node::shared_ptr<mvsim::World> mvsim_world_ = mvsim_node::make_shared<mvsim::World>();

	mrpt::WorkerThreadsPool ros_publisher_workers_{
		4 /*threads*/, mrpt::WorkerThreadsPool::POLICY_FIFO};

	/// (Defaul=1.0) >1: speed-up, <1: slow-down
	double realtime_factor_ = 1.0;
	int gui_refresh_period_ms_ = 50;
	bool headless_ = false;

	/// Default=true. Behaves as navigation/fake_localization for each
	/// vehicle without the need to launch additional nodes.
	bool do_fake_localization_ = true;

	int publisher_history_len_ = 50;

   protected:
	mvsim_node::shared_ptr<mvsim::Server> mvsim_server_;

#if PACKAGE_ROS_VERSION == 1
	ros::NodeHandle& n_;
	ros::NodeHandle localn_{"~"};
#else
	rclcpp::Node::SharedPtr n_;
#endif

	// === ROS Publishers ====
#if PACKAGE_ROS_VERSION == 1
	// mvsim_node::shared_ptr<ros::Publisher> pub_clock_;
#else
	rclcpp::TimeSource ts_{n_};
	rclcpp::Clock::SharedPtr clock_;
#endif

	struct WorldPubs
	{
#if PACKAGE_ROS_VERSION == 1
		/// used for simul_map publication
		mvsim_node::shared_ptr<ros::Publisher> pub_map_ros;	 //!< Publisher of "simul_map" topic
		mvsim_node::shared_ptr<ros::Publisher>
			pub_map_metadata;  //!< Publisher of "simul_map_metadata" topic
#else
		/// used for simul_map publication
		rclcpp::Publisher<Msg_OccupancyGrid>::SharedPtr pub_map_ros;
		rclcpp::Publisher<Msg_MapMetaData>::SharedPtr pub_map_metadata;
#endif
	};

	WorldPubs worldPubs_;

	struct TPubSubPerVehicle
	{
#if PACKAGE_ROS_VERSION == 1
		mvsim_node::shared_ptr<ros::Subscriber>
			sub_cmd_vel;  //!< Subscribers vehicle's "cmd_vel" topic

		mvsim_node::shared_ptr<ros::Publisher> pub_odom;  //!< Publisher of "odom" topic
		mvsim_node::shared_ptr<ros::Publisher>
			pub_ground_truth;  //!< "base_pose_ground_truth" topic

		/// "fake_localization" pubs:
		mvsim_node::shared_ptr<ros::Publisher> pub_amcl_pose;  //!< Publisher of "amcl_pose" topic
		mvsim_node::shared_ptr<ros::Publisher>
			pub_particlecloud;	//!< Publisher of "particlecloud" topic

		/// Map <sensor_label> => publisher
		std::map<std::string, mvsim_node::shared_ptr<ros::Publisher>> pub_sensors;

		mvsim_node::shared_ptr<ros::Publisher> pub_chassis_markers;	 //!< "<VEH>/chassis_markers"
		mvsim_node::shared_ptr<ros::Publisher> pub_chassis_shape;  //!< "<VEH>/chassis_shape"
		mvsim_node::shared_ptr<ros::Publisher> pub_collision;  //!< "<VEH>/collision"

		mvsim_node::shared_ptr<ros::Publisher> pub_tf;	//!< "<VEH>/tf"
		mvsim_node::shared_ptr<ros::Publisher> pub_tf_static;  //!< "<VEH>/tf_static"

		Msg_MarkerArray chassis_shape_msg;
#else
		/// Subscribers vehicle's "cmd_vel" topic
		rclcpp::Subscription<Msg_Twist>::SharedPtr sub_cmd_vel;

		/// Publisher of "odom" topic
		rclcpp::Publisher<Msg_Odometry>::SharedPtr pub_odom;
		/// "base_pose_ground_truth" topic
		rclcpp::Publisher<Msg_Odometry>::SharedPtr pub_ground_truth;

		/// "fake_localization" pubs:
		rclcpp::Publisher<Msg_PoseWithCovarianceStamped>::SharedPtr pub_amcl_pose;
		rclcpp::Publisher<Msg_PoseArray>::SharedPtr pub_particlecloud;

		/// Map <sensor_label> => publisher
		std::map<std::string, mvsim_node::shared_ptr<PublisherWrapperBase>> pub_sensors;

		/// "<VEH>/chassis_markers"
		rclcpp::Publisher<Msg_MarkerArray>::SharedPtr pub_chassis_markers;
		/// "<VEH>/chassis_shape"
		rclcpp::Publisher<Msg_Polygon>::SharedPtr pub_chassis_shape;
		/// "<VEH>/collision"
		rclcpp::Publisher<Msg_Bool>::SharedPtr pub_collision;

		/// "<VEH>/tf", "<VEH>/tf_static"
		rclcpp::Publisher<Msg_TFMessage>::SharedPtr pub_tf;
		rclcpp::Publisher<Msg_TFMessage>::SharedPtr pub_tf_static;

		Msg_MarkerArray chassis_shape_msg;
#endif
	};

	/// Pubs/Subs for each vehicle. Initialized by initPubSubs(), called
	/// from notifyROSWorldIsUpdated()
	std::vector<TPubSubPerVehicle> pubsub_vehicles_;
	std::mutex pubsub_vehicles_mtx_;

	/** Initialize all pub/subs required for each vehicle, for the specific
	 * vehicle \a veh */
	void initPubSubs(TPubSubPerVehicle& out_pubsubs, mvsim::VehicleBase* veh);

	// === End ROS Publishers ====

	// === ROS Hooks ====
	void onROSMsgCmdVel(Msg_Twist_CSPtr cmd, mvsim::VehicleBase* veh);
	// === End ROS Hooks====

#if PACKAGE_ROS_VERSION == 1
	// rosgraph_msgs::Clock clockMsg_;
#endif
	// ros_Time sim_time_;	 //!< Current simulation time
	ros_Time base_last_cmd_;  //!< received a vel_cmd (for watchdog)
	ros_Duration base_watchdog_timeout_ = ros_Duration(1, 0);

	/// Unit transform (const, once)
	const tf2::Transform tfIdentity_ = tf2::Transform::getIdentity();

	struct TThreadParams
	{
		TThreadParams() = default;

		MVSimNode* obj = nullptr;
		std::atomic_bool closing{false};
	};
	TThreadParams thread_params_;
	mrpt::system::CTicTac realtime_tictac_;

	double t_old_ = -1;

	/// will be true after a success call to loadWorldModel()
	bool world_init_ok_ = false;

	/// Minimum period between publication of TF transforms & /*/odom topics
	/// (In ms)
	double period_ms_publish_tf_ = 20;

	mrpt::system::CTicTac tim_publish_tf_;

	/// If true, vehicle namespaces will be used even if there is only one vehicle:
	bool force_publish_vehicle_namespace_ = false;

	/// Minimum period between update of live info & read of teleop key
	/// strokes in GUI (In ms)
	double period_ms_teleop_refresh_ = 100;
	mrpt::system::CTicTac tim_teleop_refresh_;

	std::map<mvsim::VehicleBase*, double> lastCmdVelTimestamp_;

	/// for teleoperation from the GUI (selects the focused" vehicle)
	size_t teleop_idx_veh_ = 0;
	mvsim::World::TGUIKeyEvent gui_key_events_;
	std::string msg2gui_;

	std::thread thGUI_;
	static void thread_update_GUI(TThreadParams& thread_params);

	/** Publish relevant stuff whenever a new world model is loaded (grid
	 * maps, etc.) */
	void notifyROSWorldIsUpdated();

	/** Publish everything to be published at each simulation iteration */
	void spinNotifyROS();

	/** Creates the string "/<VEH_NAME>/<VAR_NAME>" if there're more than
	 * one vehicle in the World, or "/<VAR_NAME>" otherwise. */
	std::string vehVarName(const std::string& sVarName, const mvsim::VehicleBase& veh) const;

	ros_Time myNow() const;
	double myNowSec() const;

	mrpt::system::CTimeLogger profiler_{true /*enabled*/, "mvsim_node"};

	void publishWorldElements(mvsim::WorldElementBase& obj);
	void publishVehicles(mvsim::VehicleBase& veh);

	void internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservation2DRangeScan& obs);
	void internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservationImage& obs);
	void internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservation3DRangeScan& obs);
	void internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservationPointCloud& obs);
	void internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservationIMU& obs);
	void internalOn(const mvsim::VehicleBase& veh, const mrpt::obs::CObservationGPS& obs);

};	// end class
