/**
  */

#include "mvsim/mvsim_node_core.h"
#include <rapidxml_utils.hpp>
#include <iostream>

#include <mrpt/system/os.h> // kbhit()

#include <mrpt_bridge/laser_scan.h>

/*------------------------------------------------------------------------------
 * MVSimNode()
 * Constructor.
 *----------------------------------------------------------------------------*/
MVSimNode::MVSimNode() :
	realtime_factor_ (1.0),
	gui_refresh_period_ms_ (25),
	t_old_           (-1),
	world_init_ok_   (false),
	m_teleop_idx_veh (0)
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
	using namespace mvsim;

	// Do simulation itself:
	// ========================================================================
	// Handle 1st iter:
	if (t_old_<0) t_old_ = realtime_tictac_.Tac();
	// Compute how much time has passed to simulate in real-time:
	double t_new = realtime_tictac_.Tac();
	double incr_time = realtime_factor_ * (t_new-t_old_);

	if (incr_time < mvsim_world_.get_simul_timestep())  // Just in case the computer is *really fast*...
		return;

	// Simulate:
	mvsim_world_.run_simulation(incr_time);

	//t_old_simul = world.get_simul_time();
	t_old_ = t_new;


	// House-keeping to be done at each iteration
	// ========================================================================
	const World::TListVehicles &vehs = mvsim_world_.getListOfVehicles();
	for (World::TListVehicles::const_iterator it = vehs.begin(); it!=vehs.end();++it)
	{
		const std::string &sVehName = it->first;
		const VehicleBase * veh = it->second;

		// get and broadcast the ground-truth vehicle pose:
		const mrpt::math::TPose3D & gh_veh_pose = veh->getPose();

		this->broadcastTF_GTPose(gh_veh_pose,sVehName);
	}

	// GUI msgs, teleop, etc.
	// ========================================================================
	std::string txt2gui_tmp;
	World::TGUIKeyEvent keyevent = m_gui_key_events;

	// Global keys:
	switch (keyevent.keycode)
	{
	//case 27: do_exit=true; break;
	case '1': case '2': case '3': case '4': case '5': case '6':
		m_teleop_idx_veh = keyevent.keycode-'1';
		break;
	};

	{ // Test: Differential drive: Control raw forces
		txt2gui_tmp+=mrpt::format("Selected vehicle: %u/%u\n", static_cast<unsigned>(m_teleop_idx_veh+1),static_cast<unsigned>(vehs.size()) );
		if (vehs.size()>m_teleop_idx_veh)
		{
			// Get iterator to selected vehicle:
			World::TListVehicles::const_iterator it_veh = vehs.begin();
			std::advance(it_veh, m_teleop_idx_veh);

			// Get speed: ground truth
			{
				const vec3 &vel = it_veh->second->getVelocityLocal();
				txt2gui_tmp+=mrpt::format("gt. vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg/s\n", vel.vals[0], vel.vals[1], mrpt::utils::RAD2DEG(vel.vals[2]) );
			}
			// Get speed: ground truth
			{
				const vec3 &vel = it_veh->second->getVelocityLocalOdoEstimate();
				txt2gui_tmp+=mrpt::format("odo vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg/s\n", vel.vals[0], vel.vals[1], mrpt::utils::RAD2DEG(vel.vals[2]) );
			}

			// Generic teleoperation interface for any controller that supports it:
			{
				ControllerBaseInterface *controller = it_veh->second->getControllerInterface();
				ControllerBaseInterface::TeleopInput teleop_in;
				ControllerBaseInterface::TeleopOutput teleop_out;
				teleop_in.keycode = keyevent.keycode;
				controller->teleop_interface(teleop_in,teleop_out);
				txt2gui_tmp+=teleop_out.append_gui_lines;
			}

		}
	}

	// Clear the keystroke buffer
	if (keyevent.keycode!=0)
		m_gui_key_events = World::TGUIKeyEvent();

	m_msg2gui = txt2gui_tmp;  // send txt msgs to show in the GUI

}


/*------------------------------------------------------------------------------
 * thread_update_GUI()
 *----------------------------------------------------------------------------*/
void MVSimNode::thread_update_GUI(TThreadParams &thread_params)
{
	using namespace mvsim;

	MVSimNode *obj = thread_params.obj;

	while (!thread_params.closing)
	{
		if (obj->world_init_ok_)
		{
			World::TUpdateGUIParams guiparams;
			guiparams.msg_lines = obj->m_msg2gui;

			obj->mvsim_world_.update_GUI(&guiparams);

			// Send key-strokes to the main thread:
			if(guiparams.keyevent.keycode!=0) obj->m_gui_key_events = guiparams.keyevent;
		}
		mrpt::system::sleep(obj->gui_refresh_period_ms_);
	}
}


/** Publish the ground truth pose of a robot to tf as: map -> <ROBOT>/base_link */
void MVSimNode::broadcastTF_GTPose(const mrpt::math::TPose3D &pose, const std::string &robotName)
{
	broadcastTF(pose,"/map","/"+robotName+"/base_link");
}

/** Publish "odometry" for a robot to tf as: odom -> <ROBOT>/base_link */
void MVSimNode::broadcastTF_Odom(const mrpt::math::TPose3D &pose,const std::string &robotName)
{
	broadcastTF(pose,"/odom/"+robotName, "/"+robotName+"/base_link" );
}

/** Publish pose to tf: parentFrame -> <ROBOT>/base_link */
void MVSimNode::broadcastTF(
	const mrpt::math::TPose3D &pose,
	const std::string &parentFrame,
	const std::string &childFrame)
{
	tf::Matrix3x3 rot;
	rot.setEulerYPR(pose.yaw,pose.pitch,pose.roll);
	const tf::Transform tr(rot, tf::Vector3(pose.x,pose.y,pose.z) );

	tf_br_.sendTransform(tf::StampedTransform(tr, ros::Time::now(), parentFrame, childFrame));
}
