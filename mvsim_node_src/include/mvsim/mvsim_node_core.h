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

	mrpt::system::TThreadHandle thGUI_;
	static void thread_update_GUI(TThreadParams &thread_params);

}; // end class

#endif // SR_MVSIM_NODE_CORE_H
