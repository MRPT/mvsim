/**
  */

#include "mvsim/mvsim_node_core.h"
#include <rapidxml_utils.hpp>
#include <iostream>

#include <mrpt/system/os.h> // kbhit()

#include <mrpt_bridge/laser_scan.h>

std::string msg2gui;


/*------------------------------------------------------------------------------
 * MVSimNode()
 * Constructor.
 *----------------------------------------------------------------------------*/
MVSimNode::MVSimNode() :
	realtime_factor_ (1.0),
	gui_refresh_period_ms_ (25),
	t_old_           (-1),
	world_init_ok_   (false)
{
	// Launch GUI thread:
	thread_params_.obj = this;
	thGUI_ = mrpt::system::createThreadRef( &MVSimNode::thread_update_GUI, thread_params_);
}

void MVSimNode::loadWorldModel(const std::string &world_xml_file)
{
	ROS_INFO("[MVSimNode] Loading world file: %s",world_xml_file.c_str());

	// Load from XML:
	rapidxml::file<> fil_xml(world_xml_file.c_str());
	mvsim_world_.load_from_XML( fil_xml.data(), world_xml_file );

	ROS_INFO("[MVSimNode] World file load done.");
	world_init_ok_	= true;
}


/*------------------------------------------------------------------------------
 * ~MVSimNode()
 * Destructor.
 *----------------------------------------------------------------------------*/

MVSimNode::~MVSimNode()
{
	thread_params_.closing = true;
	mrpt::system::joinThread( thGUI_ );
}

/*------------------------------------------------------------------------------
 * publishMessage()
 * Publish the message.
 *----------------------------------------------------------------------------*/

void MVSimNode::publishMessage(ros::Publisher *pub_message)
{
/*  mvsim::NodeExampleData msg;
  msg.message = message;
  msg.a = a;
  msg.b = b;

  pub_message->publish(msg);*/
}

/*------------------------------------------------------------------------------
 * messageCallback()
 * Callback function for subscriber.
 *----------------------------------------------------------------------------*/

/*void MVSimNode::messageCallback(const node_example::NodeExampleData::ConstPtr &msg)
{
  message = msg->message;
  a = msg->a;
  b = msg->b;

  // Note that these are only set to INFO so they will print to a terminal for example purposes.
  // Typically, they should be DEBUG.
  ROS_INFO("message is %s", message.c_str());
  ROS_INFO("sum of a + b = %d", a + b);
}*/

/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/

void MVSimNode::configCallback(mvsim::mvsimNodeConfig &config, uint32_t level)
{
	// Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	//  message = config.message.c_str();
	std::cout << "Debug: " << config.debug << std::endl;
}

// Process pending msgs, run real-time simulation, etc.
void MVSimNode::spin()
{
	// Handle 1st iter:
	if (t_old_<0) t_old_ = realtime_tictac_.Tac();

	// Simulation ============================================================
	// Compute how much time has passed to simulate in real-time:
	double t_new = realtime_tictac_.Tac();
	double incr_time = realtime_factor_ * (t_new-t_old_);

	if (incr_time >= mvsim_world_.get_simul_timestep())  // Just in case the computer is *really fast*...
	{
		// Simulate:
		mvsim_world_.run_simulation(incr_time);

		//t_old_simul = world.get_simul_time();
		t_old_ = t_new;
	}

}


/*------------------------------------------------------------------------------
 * thread_update_GUI()
 *----------------------------------------------------------------------------*/
void MVSimNode::thread_update_GUI(TThreadParams &thread_params)
{
	MVSimNode *obj = thread_params.obj;

	while (!thread_params.closing)
	{
		if (obj->world_init_ok_)
		{
			mvsim::World::TUpdateGUIParams guiparams;
			guiparams.msg_lines = msg2gui;

			obj->mvsim_world_.update_GUI(&guiparams);

			// Send key-strokes to the main thread:
			//if(guiparams.keyevent.keycode!=0) gui_key_events = guiparams.keyevent;
		}
		mrpt::system::sleep(obj->gui_refresh_period_ms_);
	}
}

