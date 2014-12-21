/**
  */

#include "mvsim/mvsim_node_core.h"
#include <rapidxml_utils.hpp>
#include <iostream>

#include <mrpt/system/os.h> // kbhit()

#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/pose.h>

#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>

#include <nav_msgs/Odometry.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>

/*------------------------------------------------------------------------------
 * MVSimNode()
 * Constructor.
 *----------------------------------------------------------------------------*/
MVSimNode::MVSimNode(ros::NodeHandle &n) :
	mvsim_world_     (*this),
	realtime_factor_ (1.0),
	gui_refresh_period_ms_ (75),
	m_show_gui       (true),
	m_do_fake_localization (true),
	m_transform_tolerance (0.1),
	m_n              (n),
	m_localn         ("~"),
	m_tf_br          (),
	m_tfIdentity     (tf::createIdentityQuaternion(), tf::Point(0, 0, 0)),
	t_old_           (-1),
	world_init_ok_   (false),
	m_period_ms_publish_tf (20),
	m_period_ms_teleop_refresh(100),
	m_teleop_idx_veh (0)
{
	// Launch GUI thread:
	thread_params_.obj = this;
	thGUI_ = mrpt::system::createThreadRef( &MVSimNode::thread_update_GUI, thread_params_);

	// Init ROS publishers:
	m_pub_clock = m_n.advertise<rosgraph_msgs::Clock>("/clock",10);

	m_pub_map_ros  = m_n.advertise<nav_msgs::OccupancyGrid>( "simul_map", 1  /*queue len*/, true /*latch*/);
	m_pub_map_metadata = m_n.advertise<nav_msgs::MapMetaData>("simul_map_metadata", 1/*queue len*/, true /*latch*/);

	m_sim_time.fromSec(0.0);
	m_base_last_cmd.fromSec(0.0);

	// Node parameters:
	double t;
	if(!m_localn.getParam("base_watchdog_timeout", t))
	  t = 0.2;
	m_base_watchdog_timeout.fromSec(t);

	m_localn.param("realtime_factor", realtime_factor_, 1.0);
	m_localn.param("gui_refresh_period", gui_refresh_period_ms_, gui_refresh_period_ms_);
	m_localn.param("show_gui", m_show_gui, m_show_gui);
	m_localn.param("period_ms_publish_tf",m_period_ms_publish_tf,m_period_ms_publish_tf);
	m_localn.param("do_fake_localization", m_do_fake_localization, m_do_fake_localization);

	// In case the user didn't set it:
	m_n.setParam("/use_sim_time", true);

}


void MVSimNode::loadWorldModel(const std::string &world_xml_file)
{
	ROS_INFO("[MVSimNode] Loading world file: %s",world_xml_file.c_str());

	// Load from XML:
	rapidxml::file<> fil_xml(world_xml_file.c_str());
	mvsim_world_.load_from_XML( fil_xml.data(), world_xml_file );

	ROS_INFO("[MVSimNode] World file load done.");
	world_init_ok_	= true;

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
	mrpt::system::joinThread( thGUI_ );
}

/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/
void MVSimNode::configCallback(mvsim::mvsimNodeConfig &config, uint32_t level)
{
	// Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	//  message = config.message.c_str();
	ROS_INFO("MVSimNode::configCallback() called.");

	mvsim_world_.set_simul_timestep( config.simul_timestep );

	if (mvsim_world_.is_GUI_open() && !config.show_gui)
		mvsim_world_.close_GUI();

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

	const World::TListVehicles &vehs = mvsim_world_.getListOfVehicles();

	// Publish new state to ROS
	// ========================================================================
	this->spinNotifyROS();

	// GUI msgs, teleop, etc.
	// ========================================================================
	if (m_tim_teleop_refresh.Tac()>m_period_ms_teleop_refresh*1e-3)
	{
		m_tim_teleop_refresh.Tic();

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

		m_msg2gui = txt2gui_tmp;  // send txt msgs to show in the GUI

		// Clear the keystroke buffer
		if (keyevent.keycode!=0)
			m_gui_key_events = World::TGUIKeyEvent();

	} // end refresh teleop stuff



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
		if (obj->world_init_ok_ && obj->m_show_gui)
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

// Visitor: Vehicles
// ----------------------------------------
void MVSimNode::MVSimVisitor_notifyROSWorldIsUpdated::visit(mvsim::VehicleBase *obj)
{

} // end visit(Vehicles)

// Visitor: World elements
// ----------------------------------------
void MVSimNode::MVSimVisitor_notifyROSWorldIsUpdated::visit(mvsim::WorldElementBase *obj)
{
	// GridMaps --------------
	if ( dynamic_cast<mvsim::OccupancyGridMap*>(obj) )
	{
		mvsim::OccupancyGridMap* grid = dynamic_cast<mvsim::OccupancyGridMap*>(obj);

		nav_msgs::OccupancyGrid ros_map;
		mrpt_bridge::convert(grid->getOccGrid(), ros_map);

		static size_t loop_count = 0;
		ros_map.header.stamp = ros::Time::now();
		ros_map.header.seq = loop_count++;

		m_parent.m_pub_map_ros.publish(ros_map );
		m_parent.m_pub_map_metadata.publish( ros_map.info );

	} // end gridmap

} // end visit(World Elements)

// ROS: Publish grid map for visualization purposes:
void MVSimNode::notifyROSWorldIsUpdated()
{
	MVSimVisitor_notifyROSWorldIsUpdated myvisitor(*this);

	mvsim_world_.runVisitorOnWorldElements(myvisitor);
	mvsim_world_.runVisitorOnVehicles(myvisitor);

	// Create subscribers & publishers for each vehicle's stuff:
	// ----------------------------------------------------
	mvsim::World::TListVehicles &vehs = mvsim_world_.getListOfVehicles();
	m_pubsub_vehicles.clear();
	m_pubsub_vehicles.resize(vehs.size());
	size_t idx=0;
	for (mvsim::World::TListVehicles::iterator it = vehs.begin(); it!=vehs.end();++it,++idx)
	{
		mvsim::VehicleBase * veh = it->second;
		initPubSubs(m_pubsub_vehicles[idx],veh);
	}
}

/** Initialize all pub/subs required for each vehicle, for the specific vehicle \a veh */
void MVSimNode::initPubSubs(TPubSubPerVehicle &pubsubs, mvsim::VehicleBase* veh)
{
	// sub: /cmd_vel
	pubsubs.sub_cmd_vel = m_n.subscribe<geometry_msgs::Twist>(
		vehVarName("cmd_vel",veh), 10,
		boost::bind(&MVSimNode::onROSMsgCmdVel,this,_1,veh)
		);

	// pub: /odom
	pubsubs.pub_odom = m_n.advertise<nav_msgs::Odometry>(vehVarName("odom", veh), 10);

	// pub: /base_pose_ground_truth
	pubsubs.pub_ground_truth = m_n.advertise<nav_msgs::Odometry>(vehVarName("base_pose_ground_truth", veh), 10);

	if (m_do_fake_localization)
	{
		// pub: /amcl_pose
		pubsubs.pub_amcl_pose = m_n.advertise<geometry_msgs::PoseWithCovarianceStamped>(vehVarName("amcl_pose", veh), 1);
		// pub: /particlecloud
		pubsubs.pub_particlecloud = m_n.advertise<geometry_msgs::PoseArray>(vehVarName("particlecloud", veh), 1);
	}
}


void MVSimNode::onROSMsgCmdVel(const geometry_msgs::Twist::ConstPtr &cmd, mvsim::VehicleBase * veh )
{
	mvsim::ControllerBaseInterface *controller = veh->getControllerInterface();

	const bool ctrlAcceptTwist = controller->setTwistCommand(cmd->linear.x, cmd->angular.z);

	if (!ctrlAcceptTwist) {
		ROS_DEBUG_THROTTLE(5.0, "*Warning* Vehicle's controller ['%s'] refuses Twist commands!", veh->getName().c_str() );
	}
}


/** Publish everything to be published at each simulation iteration */
void MVSimNode::spinNotifyROS()
{
	using namespace mvsim;
	const World::TListVehicles &vehs = mvsim_world_.getListOfVehicles();

	// Get current simulation time (for messages) and publish "/clock"
	// ----------------------------------------------------------------
	m_sim_time.fromSec(mvsim_world_.get_simul_time());
	m_clockMsg.clock = m_sim_time;
	m_pub_clock.publish(m_clockMsg);

	// Publish all TFs for each vehicle:
	// ---------------------------------------------------------------------
	if (m_tim_publish_tf.Tac()>m_period_ms_publish_tf*1e-3)
	{
		m_tim_publish_tf.Tic();

		size_t i=0;
		ROS_ASSERT(m_pubsub_vehicles.size()==vehs.size());

		for (World::TListVehicles::const_iterator it = vehs.begin(); it!=vehs.end();++it, ++i)
		{
			const VehicleBase * veh = it->second;

			const std::string sOdomName      = vehVarName("odom",veh);
			const std::string sBaseLinkFrame = vehVarName("base_link",veh);

			// 1) Ground-truth pose and velocity
			// --------------------------------------------
			const mrpt::math::TPose3D & gh_veh_pose = veh->getPose();
			const mvsim::vec3         & gh_veh_vel  = veh->getVelocity();  // [vx,vy,w] in global frame

			{
				nav_msgs::Odometry gtOdoMsg;

				gtOdoMsg.pose.pose.position.x = gh_veh_pose.x;
				gtOdoMsg.pose.pose.position.y = gh_veh_pose.y;
				gtOdoMsg.pose.pose.position.z = gh_veh_pose.z;

				tf::Quaternion quat;
				quat.setEuler( gh_veh_pose.roll, gh_veh_pose.pitch, gh_veh_pose.yaw );

				gtOdoMsg.pose.pose.orientation.x = quat.x();
				gtOdoMsg.pose.pose.orientation.y = quat.y();
				gtOdoMsg.pose.pose.orientation.z = quat.z();
				gtOdoMsg.pose.pose.orientation.w = quat.w();
				gtOdoMsg.twist.twist.linear.x    = gh_veh_vel.vals[0];
				gtOdoMsg.twist.twist.linear.y    = gh_veh_vel.vals[1];
				gtOdoMsg.twist.twist.linear.z    = 0;
				gtOdoMsg.twist.twist.angular.z   = gh_veh_vel.vals[2];

				gtOdoMsg.header.stamp = m_sim_time;
				gtOdoMsg.header.frame_id = sOdomName;
				gtOdoMsg.child_frame_id = sBaseLinkFrame;

				m_pubsub_vehicles[i].pub_ground_truth.publish(gtOdoMsg);

				if (m_do_fake_localization)
				{
					geometry_msgs::PoseWithCovarianceStamped  currentPos;
					geometry_msgs::PoseArray  particleCloud;

					// topic: <Ri>/particlecloud
					particleCloud.header.stamp = m_sim_time;
					particleCloud.header.frame_id = "/map";
					particleCloud.poses.resize(1);
					particleCloud.poses[0] = gtOdoMsg.pose.pose;
					m_pubsub_vehicles[i].pub_particlecloud.publish(particleCloud);

					// topic: <Ri>/amcl_pose
					currentPos.header = gtOdoMsg.header;
					currentPos.pose.pose = gtOdoMsg.pose.pose;
					m_pubsub_vehicles[i].pub_amcl_pose.publish(currentPos);

					// TF: /map -> <Ri>/odom
					{
						MRPT_TODO("Save initial pose for each vehicle, set odometry from that pose");
						const tf::Transform tr(tf::createIdentityQuaternion() , tf::Vector3(0,0,0) );
						m_tf_br.sendTransform(tf::StampedTransform( tr, m_sim_time, "/map", sOdomName) );
					}

				}
			}


			// 2) Sensor placement on vehicles:
			// --------------------------------------------
			// ...

			// 3) odometry transform
			// --------------------------------------------
			{
				const mrpt::math::TPose3D odo_pose = gh_veh_pose;

				{
					tf::Matrix3x3 rot;
					rot.setEulerYPR(odo_pose.yaw,odo_pose.pitch,odo_pose.roll);
					const tf::Transform tr(rot, tf::Vector3(odo_pose.x,odo_pose.y,odo_pose.z) );

					m_tf_br.sendTransform(tf::StampedTransform( tr, m_sim_time, sOdomName, sBaseLinkFrame) );
				}

				// Apart from TF, publish to the "odom" topic as well
				{
					nav_msgs::Odometry odoMsg;

					odoMsg.pose.pose.position.x = odo_pose.x;
					odoMsg.pose.pose.position.y = odo_pose.y;
					odoMsg.pose.pose.position.z = odo_pose.z;

					tf::Quaternion quat;
					quat.setEuler( odo_pose.roll, odo_pose.pitch, odo_pose.yaw );

					odoMsg.pose.pose.orientation.x = quat.x();
					odoMsg.pose.pose.orientation.y = quat.y();
					odoMsg.pose.pose.orientation.z = quat.z();
					odoMsg.pose.pose.orientation.w = quat.w();

					// first, we'll populate the header for the odometry msg
					odoMsg.header.stamp = m_sim_time;
					odoMsg.header.frame_id = sOdomName;
					odoMsg.child_frame_id  = sBaseLinkFrame;

					// publish:
					m_pubsub_vehicles[i].pub_odom.publish(odoMsg);
				}
			}


			// 4) Identity transform between base_footprint and base_link
			// ------------------------------------------------------------
			m_tf_br.sendTransform(tf::StampedTransform(m_tfIdentity,m_sim_time,vehVarName("base_link", veh),vehVarName("base_footprint", veh)));

		} // end for each vehicle

		// Publish the static transform /world -> /map
		m_tf_br.sendTransform(tf::StampedTransform(m_tfIdentity,m_sim_time,"/world","/map" ));

	} // end publish tf

} // end spinNotifyROS()

void MVSimNode::MyWorld::onNewObservation(const mvsim::VehicleBase &veh, const mrpt::slam::CObservation* obs)
{
	ROS_ASSERT(obs);
	ROS_ASSERT(!obs->sensorLabel.empty());

	TPubSubPerVehicle & pubs = m_parent.m_pubsub_vehicles[veh.getVehicleIndex()];

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub = pubs.pub_sensors.find(obs->sensorLabel)==pubs.pub_sensors.end();
	ros::Publisher & pub = pubs.pub_sensors[obs->sensorLabel];

	// Observation: 2d laser scans
	// -----------------------------
	if ( dynamic_cast<const mrpt::slam::CObservation2DRangeScan*>(obs) )
	{
		if (is_1st_pub)
			pub = m_parent.m_n.advertise<sensor_msgs::LaserScan>( m_parent.vehVarName(obs->sensorLabel,&veh), 10);

		// Convert observation MRPT -> ROS
		const mrpt::slam::CObservation2DRangeScan* o = dynamic_cast<const mrpt::slam::CObservation2DRangeScan*>(obs);
		geometry_msgs::Pose msg_pose_laser;
		sensor_msgs::LaserScan msg_laser;
		mrpt_bridge::convert(*o, msg_laser, msg_pose_laser);

		// Force usage of simulation time:
		msg_laser.header.stamp = m_parent.m_sim_time;
		msg_laser.header.frame_id = m_parent.vehVarName(obs->sensorLabel,&veh);

		mrpt::poses::CPose3D pose_laser;
		tf::Transform transform;
		o->getSensorPose(pose_laser);
		mrpt_bridge::convert(pose_laser, transform);

		m_parent.m_tf_br.sendTransform(tf::StampedTransform(
			transform,
			msg_laser.header.stamp,
			m_parent.vehVarName("base_link",&veh), // parent frame
			msg_laser.header.frame_id
			));

		pub.publish(msg_laser);
	}
	else {
		// Don't know how to emit this observation to ROS!
	}


} // end of onNewObservation()


/** Creates the string "/<VEH_NAME>/<VAR_NAME>" if there're more than one vehicle in the World, or "/<VAR_NAME>" otherwise. */
std::string MVSimNode::vehVarName(const std::string &sVarName, const mvsim::VehicleBase * veh) const
{
	if (mvsim_world_.getListOfVehicles().size()==1)
	{
		return std::string("/")+sVarName;
	}
	else
	{
		return std::string("/")+ veh->getName() +std::string("/")+sVarName;
	}
}




