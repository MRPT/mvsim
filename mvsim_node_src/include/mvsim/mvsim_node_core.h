/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#if PACKAGE_ROS_VERSION == 1
#include <dynamic_reconfigure/server.h>
#include <mvsim/mvsimNodeConfig.h>
#endif

#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>
#include <mvsim/Comms/Server.h>
#include <mvsim/World.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#if PACKAGE_ROS_VERSION == 1
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>
#include <visualization_msgs/MarkerArray.h>
#else
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#endif

#include <atomic>
#include <thread>

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

	void launch_mvsim_server();

	void loadWorldModel(const std::string& world_xml_file);

	void spin();  //!< Process pending msgs, run real-time simulation, etc.

#if PACKAGE_ROS_VERSION == 1
	/** Callback function for dynamic reconfigure server */
	void configCallback(mvsim::mvsimNodeConfig& config, uint32_t level);
#endif

	void onNewObservation(
		const mvsim::Simulable& veh, const mrpt::obs::CObservation::Ptr& obs);

	/// The mvsim library simulated world (includes everything: vehicles,
	/// obstacles, etc.)
	mvsim::World mvsim_world_;

	mrpt::WorkerThreadsPool ros_publisher_workers_{
		3 /*threads*/, mrpt::WorkerThreadsPool::POLICY_DROP_OLD};

	/// (Defaul=1.0) >1: speed-up, <1: slow-down
	double realtime_factor_ = 1.0;
	int gui_refresh_period_ms_ = 50;
	bool show_gui_ = true;	//!< Default= true

	/// Default=true. Behaves as navigation/fake_localization for each
	/// vehicle without the need to launch additional nodes.
	bool do_fake_localization_ = true;

	//!< (Default=0.1) Time tolerance for published TFs
	double m_transform_tolerance = 0.1;

   protected:
	std::shared_ptr<mvsim::Server> mvsim_server_;

#if PACKAGE_ROS_VERSION == 1
	ros::NodeHandle& n_;
	ros::NodeHandle localn_{"~"};
#else
	rclcpp::Node::SharedPtr n_;
#endif

	// === ROS Publishers ====
	/// used for simul_map publication
#if PACKAGE_ROS_VERSION == 1
	ros::Publisher pub_map_ros_, pub_map_metadata_;
	// ros::Publisher pub_clock_;
#else
	rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_map_ros_;
	rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr pub_map_metadata_;
	rclcpp::TimeSource ts_{n_};
	rclcpp::Clock::SharedPtr clock_;
#endif

#if PACKAGE_ROS_VERSION == 1
	tf2_ros::TransformBroadcaster tf_br_;  //!< Use to send data to TF
	tf2_ros::StaticTransformBroadcaster static_tf_br_;
#else
	tf2_ros::TransformBroadcaster tf_br_{n_};  //!< Use to send data to TF
	tf2_ros::StaticTransformBroadcaster static_tf_br_{n_};
#endif

	struct TPubSubPerVehicle
	{
#if PACKAGE_ROS_VERSION == 1
		ros::Subscriber sub_cmd_vel;  //!< Subscribers vehicle's "cmd_vel" topic
		ros::Publisher pub_odom;  //!< Publisher of "odom" topic
		ros::Publisher pub_ground_truth;  //!< "base_pose_ground_truth" topic

		/// "fake_localization" pubs:
		ros::Publisher pub_amcl_pose, pub_particlecloud;

		/// Map <sensor_label> => publisher
		std::map<std::string, ros::Publisher> pub_sensors;

		ros::Publisher pub_chassis_markers;	 //!< "<VEH>/chassis_markers"
		ros::Publisher pub_chassis_shape;  //!< "<VEH>/chassis_shape"

		visualization_msgs::MarkerArray chassis_shape_msg;
#else
		/// Subscribers vehicle's "cmd_vel" topic
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
		/// Publisher of "odom" topic
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
		/// "base_pose_ground_truth" topic
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_ground_truth;

		/// "fake_localization" pubs:
		rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::
			SharedPtr pub_amcl_pose;
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
			pub_particlecloud;

		/// Map <sensor_label> => publisher
		std::map<std::string, rclcpp::PublisherBase::SharedPtr> pub_sensors;

		/// "<VEH>/chassis_markers"
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
			pub_chassis_markers;
		/// "<VEH>/chassis_shape"
		rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr
			pub_chassis_shape;

		visualization_msgs::msg::MarkerArray chassis_shape_msg;
#endif
	};

	/// Pubs/Subs for each vehicle. Initialized by initPubSubs(), called
	/// from notifyROSWorldIsUpdated()
	std::vector<TPubSubPerVehicle> m_pubsub_vehicles;

	/** Initialize all pub/subs required for each vehicle, for the specific
	 * vehicle \a veh */
	void initPubSubs(TPubSubPerVehicle& out_pubsubs, mvsim::VehicleBase* veh);

	// === End ROS Publishers ====

	// === ROS Hooks ====
	void onROSMsgCmdVel(
#if PACKAGE_ROS_VERSION == 1
		const geometry_msgs::Twist::ConstPtr& cmd,
#else
		const geometry_msgs::msg::Twist::SharedPtr cmd,
#endif
		mvsim::VehicleBase* veh);
	// === End ROS Hooks====

#if PACKAGE_ROS_VERSION == 1
	// rosgraph_msgs::Clock clockMsg_;
	// ros::Time sim_time_;  //!< Current simulation time
	ros::Time base_last_cmd_;  //!< received a vel_cmd (for watchdog)
	ros::Duration base_watchdog_timeout_;
#else
	// rclcpp::Time sim_time_;	 //!< Current simulation time
	rclcpp::Time base_last_cmd_;  //!< received a vel_cmd (for watchdog)
	rclcpp::Duration base_watchdog_timeout_ = std::chrono::seconds(1);
#endif

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

	/// Minimum period between update of live info & read of teleop key
	/// strokes in GUI (In ms)
	double period_ms_teleop_refresh_ = 100;
	mrpt::system::CTicTac tim_teleop_refresh_;

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
	std::string vehVarName(
		const std::string& sVarName, const mvsim::VehicleBase& veh) const;

	void sendStaticTF(
		const std::string& frame_id, const std::string& child_frame_id,
		const tf2::Transform& tx,
#if PACKAGE_ROS_VERSION == 1
		const ros::Time& stamp
#else
		const rclcpp::Time& stamp
#endif
	);

#if PACKAGE_ROS_VERSION == 1
	ros::Time myNow() const;
#else
	rclcpp::Time myNow() const;
#endif

	mrpt::system::CTimeLogger profiler_{true /*enabled*/, "mvsim_node"};

	void publishWorldElements(mvsim::WorldElementBase& obj);
	void publishVehicles(mvsim::VehicleBase& veh);

	void internalOn(
		const mvsim::VehicleBase& veh,
		const mrpt::obs::CObservation2DRangeScan& obs);
	void internalOn(
		const mvsim::VehicleBase& veh, const mrpt::obs::CObservationImage& obs);
	void internalOn(
		const mvsim::VehicleBase& veh,
		const mrpt::obs::CObservation3DRangeScan& obs);

};	// end class
