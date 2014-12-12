/**
 */

#ifndef SR_MVSIM_NODE_CORE_H
#define SR_MVSIM_NODE_CORE_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// Custom message includes. Auto-generated from msg/ directory.
//#include "node_example/NodeExampleData.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>

#include <tf/transform_broadcaster.h>

// Auto-generated from cfg/ directory.
#include <mvsim/mvsimNodeConfig.h>

#include <mvsim/mvsim.h>  // the mvsim library
#include <mrpt/system/threads.h> // sleep(), thread handles
#include <mrpt/utils/CTicTac.h>

/** A class to wrap libmvsim as a ROS node
  */
class MVSimNode
{
public:
	/** Constructor. */
	MVSimNode();
	/** Destructor. */
	~MVSimNode();

	void loadWorldModel(const std::string &world_xml_file);

	void spin(); //!< Process pending msgs, run real-time simulation, etc.

	/** Callback function for dynamic reconfigure server */
	void configCallback(mvsim::mvsimNodeConfig &config, uint32_t level);

	/** Publish the message */
	void publishMessage(ros::Publisher *pub_message);

	/** Callback function for subscriber */
	//  void messageCallback(const mvsim_node::NodeExampleData::ConstPtr &msg);

	mvsim::World  mvsim_world_; //!< The mvsim library simulated world (includes everything: vehicles, obstacles, etc.)

	double realtime_factor_; //!< (Defaul=1.0) >1: speed-up, <1: slow-down
	int    gui_refresh_period_ms_; //!< Default:25

protected:

	struct TThreadParams
	{
		MVSimNode *obj;
		volatile bool closing;
		TThreadParams(): obj(NULL), closing(false) {}
	};
	TThreadParams thread_params_;
	mrpt::utils::CTicTac realtime_tictac_;

	double t_old_; // = realtime_tictac_.Tac();
	bool   world_init_ok_; //!< will be true after a success call to loadWorldModel()

	double m_period_ms_publish_tf;    //!< Minimum period between publication of TF transforms & /*/odom topics (In ms)
	mrpt::utils::CTicTac   m_tim_publish_tf;

	double m_period_ms_teleop_refresh; //!< Minimum period between update of live info & read of teleop key strokes in GUI (In ms)
	mrpt::utils::CTicTac   m_tim_teleop_refresh;

	size_t m_teleop_idx_veh;  //!< for teleoperation from the GUI (selects the "focused" vehicle)
	mvsim::World::TGUIKeyEvent m_gui_key_events;
	std::string m_msg2gui;

	mrpt::system::TThreadHandle thGUI_;
	static void thread_update_GUI(TThreadParams &thread_params);

	tf::TransformBroadcaster tf_br_; //!< Use to send data to TF
	ros::Publisher m_odo_publisher;


	/** Publish the ground truth pose of a robot to tf as: map -> <ROBOT>/base_link */
	void broadcastTF_GTPose(
		const mrpt::math::TPose3D &pose,
		const std::string &robotName = std::string("r1"));

	/** Publish "odometry" for a robot to tf as: odom -> <ROBOT>/base_link */
	void broadcastTF_Odom(
		const mrpt::math::TPose3D &pose,
		const std::string &robotName = std::string("r1"));

	/** Publish pose to tf: parentFrame -> <ROBOT>/base_link */
	void broadcastTF(
		const mrpt::math::TPose3D &pose,
		const std::string &parentFrame,
		const std::string &childFrame);

}; // end class

#endif // SR_MVSIM_NODE_CORE_H
