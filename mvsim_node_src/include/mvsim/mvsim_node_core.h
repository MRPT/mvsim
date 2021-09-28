/**
 */

#ifndef SR_MVSIM_NODE_CORE_H
#define SR_MVSIM_NODE_CORE_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <thread>

// Auto-generated from cfg/ directory.
#include <mrpt/utils/CTicTac.h>
#include <mvsim/mvsim.h>  // the mvsim library
#include <mvsim/mvsimNodeConfig.h>

#if MRPT_VERSION >= 0x130
#include <mrpt/obs/CObservation.h>
using mrpt::obs::CObservation;
#else
#include <mrpt/slam/CObservation.h>
using mrpt::slam::CObservation;
#endif

/** A class to wrap libmvsim as a ROS node
 */
class MVSimNode
{
   public:
	/** Constructor. */
	MVSimNode(ros::NodeHandle& n);
	/** Destructor. */
	~MVSimNode();

	void loadWorldModel(const std::string& world_xml_file);

	void spin();  //!< Process pending msgs, run real-time simulation, etc.

	/** Callback function for dynamic reconfigure server */
	void configCallback(mvsim::mvsimNodeConfig& config, uint32_t level);

	void onNewObservation(
		const mvsim::VehicleBase& veh, const mrpt::obs::CObservation::Ptr& obs);

	/// The mvsim library simulated world (includes everything: vehicles,
	/// obstacles, etc.)
	mvsim::World mvsim_world_;

	double realtime_factor_;  //!< (Defaul=1.0) >1: speed-up, <1: slow-down
	int gui_refresh_period_ms_;	 //!< Default:25
	bool m_show_gui;  //!< Default= true
	bool m_do_fake_localization;  //!< Default=true. Behaves as
								  //! navigation/fake_localization for each
								  //! vehicle without the need to launch
								  //! additional nodes.
	double m_transform_tolerance;  //!< (Default=0.1) Time tolerance for
								   //! published TFs

   protected:
	ros::NodeHandle& m_n;
	ros::NodeHandle m_localn;

	// === ROS Publishers ====
	ros::Publisher m_pub_map_ros,
		m_pub_map_metadata;	 //!< used for simul_map publication
	ros::Publisher m_pub_clock;
	tf::TransformBroadcaster m_tf_br;  //!< Use to send data to TF
	tf2_ros::StaticTransformBroadcaster
		m_static_tf_br;	 //!< For sending STATIC tf

	struct TPubSubPerVehicle
	{
		ros::Subscriber
			sub_cmd_vel;  //!< Subscribers for each vehicle's "cmd_vel" topic
		ros::Publisher pub_odom;  //!< Publisher of "odom" topic
		ros::Publisher
			pub_ground_truth;  //!< Publisher of "base_pose_ground_truth" topic
		ros::Publisher pub_amcl_pose,
			pub_particlecloud;	//!< Publishers for "fake_localization" topics
		std::map<std::string, ros::Publisher>
			pub_sensors;  //!< Map <sensor_label> => publisher
		ros::Publisher pub_chassis_markers;	 //!< "<VEH>/chassis_markers"
		ros::Publisher pub_chassis_shape;  //!< "<VEH>/chassis_shape"
		visualization_msgs::MarkerArray chassis_shape_msg;
	};

	std::vector<TPubSubPerVehicle>
		m_pubsub_vehicles;	//!< Pubs/Subs for each vehicle. Initialized by
							//! initPubSubs(), called from
							//! notifyROSWorldIsUpdated()

	/** Initialize all pub/subs required for each vehicle, for the specific
	 * vehicle \a veh */
	void initPubSubs(TPubSubPerVehicle& out_pubsubs, mvsim::VehicleBase* veh);

	// === End ROS Publishers ====

	// === ROS Hooks ====
	void onROSMsgCmdVel(
		const geometry_msgs::Twist::ConstPtr& cmd, mvsim::VehicleBase* veh);
	// === End ROS Hooks====

	rosgraph_msgs::Clock m_clockMsg;
	ros::Time m_sim_time;  //!< Current simulation time
	ros::Time
		m_base_last_cmd;  //!< Last time we received a vel_cmd (for watchdog)
	ros::Duration m_base_watchdog_timeout;
	const tf::Transform
		m_tfIdentity;  //!< Unit transform (const, created only once)

	struct TThreadParams
	{
		MVSimNode* obj;
		volatile bool closing;
		TThreadParams() : obj(NULL), closing(false) {}
	};
	TThreadParams thread_params_;
	mrpt::system::CTicTac realtime_tictac_;

	double t_old_;	// = realtime_tictac_.Tac();
	bool world_init_ok_;  //!< will be true after a success call to
						  //! loadWorldModel()

	double m_period_ms_publish_tf;	//!< Minimum period between publication of
									//! TF transforms & /*/odom topics (In ms)
	mrpt::system::CTicTac m_tim_publish_tf;

	double m_period_ms_teleop_refresh;	//!< Minimum period between update of
										//! live info & read of teleop key
										//! strokes in GUI (In ms)
	mrpt::system::CTicTac m_tim_teleop_refresh;

	size_t m_teleop_idx_veh;  //!< for teleoperation from the GUI (selects the
							  //!"focused" vehicle)
	mvsim::World::TGUIKeyEvent m_gui_key_events;
	std::string m_msg2gui;

	std::thread thGUI_;
	static void thread_update_GUI(TThreadParams& thread_params);

	/** Publish relevant stuff whenever a new world model is loaded (grid maps,
	 * etc.) */
	void notifyROSWorldIsUpdated();

	/** Publish everything to be published at each simulation iteration */
	void spinNotifyROS();

	/** Creates the string "/<VEH_NAME>/<VAR_NAME>" if there're more than one
	 * vehicle in the World, or "/<VAR_NAME>" otherwise. */
	std::string vehVarName(
		const std::string& sVarName, const mvsim::VehicleBase* veh) const;

	void sendStaticTF(
		const std::string& frame_id, const std::string& child_frame_id,
		const tf::Transform& tx, const ros::Time& stamp);

	struct MVSimVisitor_notifyROSWorldIsUpdated
		: public mvsim::World::vehicle_visitor_t,
		  public mvsim::World::world_element_visitor_t
	{
		void visit(mvsim::VehicleBase* obj);
		void visit(mvsim::WorldElementBase* obj);

		MVSimVisitor_notifyROSWorldIsUpdated(MVSimNode& parent)
			: m_parent(parent)
		{
		}
		MVSimNode& m_parent;
	};

};	// end class

#endif	// SR_MVSIM_NODE_CORE_H
