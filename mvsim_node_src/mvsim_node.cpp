/**
 */

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>	 // kbhit()
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/pose.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <rapidxml_utils.hpp>

#include "mvsim/mvsim_node_core.h"

/*------------------------------------------------------------------------------
 * MVSimNode()
 * Constructor.
 *----------------------------------------------------------------------------*/
MVSimNode::MVSimNode(ros::NodeHandle& n)
	: n_(n),
	  tf_br_(),
	  tfIdentity_(tf::createIdentityQuaternion(), tf::Point(0, 0, 0))
{
	// Launch GUI thread:
	thread_params_.obj = this;
	thGUI_ =
		std::thread(&MVSimNode::thread_update_GUI, std::ref(thread_params_));

	// Init ROS publishers:
	pub_clock_ = n_.advertise<rosgraph_msgs::Clock>("/clock", 10);

	pub_map_ros_ = n_.advertise<nav_msgs::OccupancyGrid>(
		"simul_map", 1 /*queue len*/, true /*latch*/);
	pub_map_metadata_ = n_.advertise<nav_msgs::MapMetaData>(
		"simul_map_metadata", 1 /*queue len*/, true /*latch*/);

	sim_time_.fromSec(0.0);
	base_last_cmd_.fromSec(0.0);

	// Node parameters:
	double t;
	if (!localn_.getParam("base_watchdog_timeout", t)) t = 0.2;
	base_watchdog_timeout_.fromSec(t);

	localn_.param("realtime_factor", realtime_factor_, 1.0);
	localn_.param(
		"gui_refresh_period", gui_refresh_period_ms_, gui_refresh_period_ms_);
	localn_.param("show_gui", show_gui_, show_gui_);
	localn_.param(
		"period_ms_publish_tf", period_ms_publish_tf_, period_ms_publish_tf_);
	localn_.param(
		"do_fake_localization", do_fake_localization_, do_fake_localization_);

	// In case the user didn't set it:
	n_.setParam("/use_sim_time", true);

	mvsim_world_.registerCallbackOnObservation(
		[this](
			const mvsim::Simulable& veh,
			const mrpt::obs::CObservation::Ptr& obs) {
			onNewObservation(veh, obs);
		});
}

void MVSimNode::loadWorldModel(const std::string& world_xml_file)
{
	ROS_INFO("[MVSimNode] Loading world file: %s", world_xml_file.c_str());
	ROS_ASSERT_MSG(
		mrpt::system::fileExists(world_xml_file),
		"[MVSimNode::loadWorldModel] File does not exist!: '%s'",
		world_xml_file.c_str());

	// Load from XML:
	rapidxml::file<> fil_xml(world_xml_file.c_str());
	mvsim_world_.load_from_XML(fil_xml.data(), world_xml_file);

	ROS_INFO("[MVSimNode] World file load done.");
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
	thGUI_.join();
}

/*------------------------------------------------------------------------------
 * configCallback()
 * Callback function for dynamic reconfigure server.
 *----------------------------------------------------------------------------*/
void MVSimNode::configCallback(mvsim::mvsimNodeConfig& config, uint32_t level)
{
	// Set class variables to new values. They should match what is input at the
	// dynamic reconfigure GUI.
	//  message = config.message.c_str();
	ROS_INFO("MVSimNode::configCallback() called.");

	if (mvsim_world_.is_GUI_open() && !config.show_gui)
		mvsim_world_.close_GUI();
}

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
					"odo vel: "s +
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
	using namespace mvsim;

	MVSimNode* obj = thread_params.obj;

	while (!thread_params.closing)
	{
		if (obj->world_init_ok_ && obj->show_gui_)
		{
			World::TUpdateGUIParams guiparams;
			guiparams.msg_lines = obj->msg2gui_;

			obj->mvsim_world_.update_GUI(&guiparams);

			// Send key-strokes to the main thread:
			if (guiparams.keyevent.keycode != 0)
				obj->gui_key_events_ = guiparams.keyevent;
		}
		std::this_thread::sleep_for(
			std::chrono::milliseconds(obj->gui_refresh_period_ms_));
	}
}

// Visitor: Vehicles
// ----------------------------------------
void MVSimNode::MVSimVisitor_notifyROSWorldIsUpdated::visit(
	mvsim::VehicleBase* obj)
{
}  // end visit(Vehicles)

// Visitor: World elements
// ----------------------------------------
void MVSimNode::MVSimVisitor_notifyROSWorldIsUpdated::visit(
	mvsim::WorldElementBase* obj)
{
	// GridMaps --------------
	if (dynamic_cast<mvsim::OccupancyGridMap*>(obj))
	{
		mvsim::OccupancyGridMap* grid =
			dynamic_cast<mvsim::OccupancyGridMap*>(obj);

		nav_msgs::OccupancyGrid ros_map;
		mrpt_bridge::convert(grid->getOccGrid(), ros_map);

		static size_t loop_count = 0;
		ros_map.header.stamp = ros::Time::now();
		ros_map.header.seq = loop_count++;

		parent_.pub_map_ros_.publish(ros_map);
		parent_.pub_map_metadata_.publish(ros_map.info);

	}  // end gridmap

}  // end visit(World Elements)

// ROS: Publish grid map for visualization purposes:
void MVSimNode::notifyROSWorldIsUpdated()
{
	MVSimVisitor_notifyROSWorldIsUpdated myvisitor(*this);

	mvsim_world_.runVisitorOnWorldElements(myvisitor);
	mvsim_world_.runVisitorOnVehicles(myvisitor);

	// Create subscribers & publishers for each vehicle's stuff:
	// ----------------------------------------------------
	auto& vehs = mvsim_world_.getListOfVehicles();
	m_pubsub_vehicles.clear();
	m_pubsub_vehicles.resize(vehs.size());
	size_t idx = 0;
	for (auto it = vehs.begin(); it != vehs.end(); ++it, ++idx)
	{
		mvsim::VehicleBase* veh =
			dynamic_cast<mvsim::VehicleBase*>(it->second.get());
		if (!veh) continue;

		initPubSubs(m_pubsub_vehicles[idx], veh);
	}

	// Publish the static transform /world -> /map
	sendStaticTF("/world", "/map", tfIdentity_, sim_time_);
}

void MVSimNode::sendStaticTF(
	const std::string& frame_id, const std::string& child_frame_id,
	const tf::Transform& txf, const ros::Time& stamp)
{
	geometry_msgs::TransformStamped tx;
	tx.header.frame_id = frame_id;
	tx.child_frame_id = child_frame_id;
	tx.header.stamp = stamp;
	tf::transformTFToMsg(txf, tx.transform);
	static_tf_br_.sendTransform(tx);
}

/** Initialize all pub/subs required for each vehicle, for the specific vehicle
 * \a veh */
void MVSimNode::initPubSubs(TPubSubPerVehicle& pubsubs, mvsim::VehicleBase* veh)
{
	// sub: <VEH>/cmd_vel
	pubsubs.sub_cmd_vel = n_.subscribe<geometry_msgs::Twist>(
		vehVarName("cmd_vel", *veh), 10,
		boost::bind(&MVSimNode::onROSMsgCmdVel, this, _1, veh));

	// pub: <VEH>/odom
	pubsubs.pub_odom =
		n_.advertise<nav_msgs::Odometry>(vehVarName("odom", *veh), 10);

	// pub: <VEH>/base_pose_ground_truth
	pubsubs.pub_ground_truth = n_.advertise<nav_msgs::Odometry>(
		vehVarName("base_pose_ground_truth", *veh), 10);

	// pub: <VEH>/chassis_markers
	{
		pubsubs.pub_chassis_markers =
			n_.advertise<visualization_msgs::MarkerArray>(
				vehVarName("chassis_markers", *veh), 5, true /*latch*/);
		const mrpt::math::TPolygon2D& poly = veh->getChassisShape();

		// Create one "ROS marker" for each wheel + 1 for the chassis:
		visualization_msgs::MarkerArray& msg_shapes = pubsubs.chassis_shape_msg;
		msg_shapes.markers.resize(1 + veh->getNumWheels());

		// [0] Chassis shape:
		visualization_msgs::Marker& chassis_shape_msg = msg_shapes.markers[0];

		chassis_shape_msg.action = visualization_msgs::Marker::MODIFY;
		chassis_shape_msg.type = visualization_msgs::Marker::LINE_STRIP;
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
			visualization_msgs::Marker& wheel_shape_msg =
				msg_shapes.markers[1 + i];
			const mvsim::Wheel& w = veh->getWheelInfo(i);

			const double lx = w.diameter * 0.5, ly = w.width * 0.5;
			wheel_shape_msg = chassis_shape_msg;  // Init values
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
			tf::Matrix3x3 rot;
			rot.setEulerYPR(w.yaw, 0, 0);
			tf::poseTFToMsg(
				tf::Transform(rot, tf::Vector3(w.x, w.y, 0)),
				wheel_shape_msg.pose);
		}  // end for each wheel

		// Publish Initial pose
		pubsubs.pub_chassis_markers.publish(msg_shapes);
	}

	// pub: <VEH>/chassis_polygon
	{
		pubsubs.pub_chassis_shape = n_.advertise<geometry_msgs::Polygon>(
			vehVarName("chassis_polygon", *veh), 1, true /*latch*/);

		// Do the first (and unique) publish:
		geometry_msgs::Polygon poly_msg;
		const mrpt::math::TPolygon2D& poly = veh->getChassisShape();
		poly_msg.points.resize(poly.size());
		for (size_t i = 0; i < poly.size(); i++)
		{
			poly_msg.points[i].x = poly[i].x;
			poly_msg.points[i].y = poly[i].y;
			poly_msg.points[i].z = 0;
		}
		pubsubs.pub_chassis_shape.publish(poly_msg);
	}

	if (do_fake_localization_)
	{
		// pub: <VEH>/amcl_pose
		pubsubs.pub_amcl_pose =
			n_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
				vehVarName("amcl_pose", *veh), 1);
		// pub: <VEH>/particlecloud
		pubsubs.pub_particlecloud = n_.advertise<geometry_msgs::PoseArray>(
			vehVarName("particlecloud", *veh), 1);
	}

	// STATIC Identity transform <VEH>/base_link -> <VEH>/base_footprint
	sendStaticTF(
		vehVarName("base_link", *veh), vehVarName("base_footprint", *veh),
		tfIdentity_, sim_time_);
}

void MVSimNode::onROSMsgCmdVel(
	const geometry_msgs::Twist::ConstPtr& cmd, mvsim::VehicleBase* veh)
{
	mvsim::ControllerBaseInterface* controller = veh->getControllerInterface();

	const bool ctrlAcceptTwist =
		controller->setTwistCommand(cmd->linear.x, cmd->angular.z);

	if (!ctrlAcceptTwist)
	{
		ROS_DEBUG_THROTTLE(
			5.0,
			"*Warning* Vehicle's controller ['%s'] refuses Twist commands!",
			veh->getName().c_str());
	}
}

/** Publish everything to be published at each simulation iteration */
void MVSimNode::spinNotifyROS()
{
	using namespace mvsim;
	const auto& vehs = mvsim_world_.getListOfVehicles();

	// Get current simulation time (for messages) and publish "/clock"
	// ----------------------------------------------------------------
	sim_time_.fromSec(mvsim_world_.get_simul_time());
	clockMsg_.clock = sim_time_;
	pub_clock_.publish(clockMsg_);

	// Publish all TFs for each vehicle:
	// ---------------------------------------------------------------------
	if (tim_publish_tf_.Tac() > period_ms_publish_tf_ * 1e-3)
	{
		tim_publish_tf_.Tic();

		size_t i = 0;
		ROS_ASSERT(m_pubsub_vehicles.size() == vehs.size());

		for (auto it = vehs.begin(); it != vehs.end(); ++it, ++i)
		{
			const VehicleBase::Ptr& veh = it->second;

			const std::string sOdomName = vehVarName("odom", *veh);
			const std::string sBaseLinkFrame = vehVarName("base_link", *veh);

			// 1) Ground-truth pose and velocity
			// --------------------------------------------
			const mrpt::math::TPose3D& gh_veh_pose = veh->getPose();
			// [vx,vy,w] in global frame
			const auto& gh_veh_vel = veh->getVelocity();

			{
				nav_msgs::Odometry gtOdoMsg;

				gtOdoMsg.pose.pose.position.x = gh_veh_pose.x;
				gtOdoMsg.pose.pose.position.y = gh_veh_pose.y;
				gtOdoMsg.pose.pose.position.z = gh_veh_pose.z;

				tf::Quaternion quat;
				quat.setEuler(
					gh_veh_pose.roll, gh_veh_pose.pitch, gh_veh_pose.yaw);

				gtOdoMsg.pose.pose.orientation.x = quat.x();
				gtOdoMsg.pose.pose.orientation.y = quat.y();
				gtOdoMsg.pose.pose.orientation.z = quat.z();
				gtOdoMsg.pose.pose.orientation.w = quat.w();
				gtOdoMsg.twist.twist.linear.x = gh_veh_vel.vx;
				gtOdoMsg.twist.twist.linear.y = gh_veh_vel.vy;
				gtOdoMsg.twist.twist.linear.z = 0;
				gtOdoMsg.twist.twist.angular.z = gh_veh_vel.omega;

				gtOdoMsg.header.stamp = sim_time_;
				gtOdoMsg.header.frame_id = sOdomName;
				gtOdoMsg.child_frame_id = sBaseLinkFrame;

				m_pubsub_vehicles[i].pub_ground_truth.publish(gtOdoMsg);

				if (do_fake_localization_)
				{
					geometry_msgs::PoseWithCovarianceStamped currentPos;
					geometry_msgs::PoseArray particleCloud;

					// topic: <Ri>/particlecloud
					if (m_pubsub_vehicles[i]
							.pub_particlecloud.getNumSubscribers() > 0)
					{
						particleCloud.header.stamp = sim_time_;
						particleCloud.header.frame_id = "/map";
						particleCloud.poses.resize(1);
						particleCloud.poses[0] = gtOdoMsg.pose.pose;
						m_pubsub_vehicles[i].pub_particlecloud.publish(
							particleCloud);
					}

					// topic: <Ri>/amcl_pose
					if (m_pubsub_vehicles[i].pub_amcl_pose.getNumSubscribers() >
						0)
					{
						currentPos.header = gtOdoMsg.header;
						currentPos.pose.pose = gtOdoMsg.pose.pose;
						m_pubsub_vehicles[i].pub_amcl_pose.publish(currentPos);
					}

					// TF: /map -> <Ri>/odom
					{
						MRPT_TODO(
							"Save initial pose for each vehicle, set odometry "
							"from that pose");
						const tf::Transform tr(
							tf::createIdentityQuaternion(),
							tf::Vector3(0, 0, 0));
						tf_br_.sendTransform(tf::StampedTransform(
							tr, sim_time_, "/map", sOdomName));
					}
				}
			}

			// 2) Chassis markers (for rviz visualization)
			// --------------------------------------------
			// pub: <VEH>/chassis_markers
			if (m_pubsub_vehicles[i].pub_chassis_markers.getNumSubscribers() >
				0)
			{
				visualization_msgs::MarkerArray& msg_shapes =
					m_pubsub_vehicles[i].chassis_shape_msg;
				ROS_ASSERT(
					msg_shapes.markers.size() == (1 + veh->getNumWheels()));

				// [0] Chassis shape: static no need to update.
				// [1:N] Wheel shapes: may move
				for (size_t j = 0; j < veh->getNumWheels(); j++)
				{
					visualization_msgs::Marker& wheel_shape_msg =
						msg_shapes.markers[1 + j];
					const mvsim::Wheel& w = veh->getWheelInfo(j);

					// Set local pose of the wheel wrt the vehicle:
					tf::Matrix3x3 rot;
					rot.setEulerYPR(w.yaw, 0, 0);
					tf::poseTFToMsg(
						tf::Transform(rot, tf::Vector3(w.x, w.y, 0)),
						wheel_shape_msg.pose);
				}  // end for each wheel

				// Publish Initial pose
				m_pubsub_vehicles[i].pub_chassis_markers.publish(msg_shapes);
			}

			// 3) odometry transform
			// --------------------------------------------
			{
				const mrpt::math::TPose3D odo_pose = gh_veh_pose;

				{
					tf::Matrix3x3 rot;
					rot.setEulerYPR(
						odo_pose.yaw, odo_pose.pitch, odo_pose.roll);
					const tf::Transform tr(
						rot, tf::Vector3(odo_pose.x, odo_pose.y, odo_pose.z));

					tf_br_.sendTransform(tf::StampedTransform(
						tr, sim_time_, sOdomName, sBaseLinkFrame));
				}

				// Apart from TF, publish to the "odom" topic as well
				if (m_pubsub_vehicles[i].pub_odom.getNumSubscribers() > 0)
				{
					nav_msgs::Odometry odoMsg;

					odoMsg.pose.pose.position.x = odo_pose.x;
					odoMsg.pose.pose.position.y = odo_pose.y;
					odoMsg.pose.pose.position.z = odo_pose.z;

					tf::Quaternion quat;
					quat.setEuler(odo_pose.roll, odo_pose.pitch, odo_pose.yaw);

					odoMsg.pose.pose.orientation.x = quat.x();
					odoMsg.pose.pose.orientation.y = quat.y();
					odoMsg.pose.pose.orientation.z = quat.z();
					odoMsg.pose.pose.orientation.w = quat.w();

					// first, we'll populate the header for the odometry msg
					odoMsg.header.stamp = sim_time_;
					odoMsg.header.frame_id = sOdomName;
					odoMsg.child_frame_id = sBaseLinkFrame;

					// publish:
					m_pubsub_vehicles[i].pub_odom.publish(odoMsg);
				}
			}

		}  // end for each vehicle

	}  // end publish tf

}  // end spinNotifyROS()

void MVSimNode::onNewObservation(
	const mvsim::Simulable& sim, const mrpt::obs::CObservation::Ptr& obs)
{
	using mrpt::obs::CObservation2DRangeScan;

	ROS_ASSERT(obs);
	ROS_ASSERT(!obs->sensorLabel.empty());

	const auto& veh = dynamic_cast<const mvsim::VehicleBase&>(sim);

	TPubSubPerVehicle& pubs = m_pubsub_vehicles[veh.getVehicleIndex()];

	// Create the publisher the first time an observation arrives:
	const bool is_1st_pub =
		pubs.pub_sensors.find(obs->sensorLabel) == pubs.pub_sensors.end();
	ros::Publisher& pub = pubs.pub_sensors[obs->sensorLabel];

	// Observation: 2d laser scans
	// -----------------------------
	if (const CObservation2DRangeScan* o =
			dynamic_cast<const CObservation2DRangeScan*>(obs.get());
		o)
	{
		if (is_1st_pub)
			pub = n_.advertise<sensor_msgs::LaserScan>(
				vehVarName(obs->sensorLabel, veh), 10);

		const std::string sSensorFrameId = vehVarName(obs->sensorLabel, veh);

		// Send TF:
		mrpt::poses::CPose3D pose_laser;
		tf::Transform transform;
		o->getSensorPose(pose_laser);
		mrpt_bridge::convert(pose_laser, transform);

		tf_br_.sendTransform(tf::StampedTransform(
			transform, sim_time_, vehVarName("base_link", veh),	 // parent frame
			sSensorFrameId));

		// Send observation:
		if (is_1st_pub || pub.getNumSubscribers() > 0)
		{
			// Convert observation MRPT -> ROS
			geometry_msgs::Pose msg_pose_laser;
			sensor_msgs::LaserScan msg_laser;
			mrpt_bridge::convert(*o, msg_laser, msg_pose_laser);

			// Force usage of simulation time:
			msg_laser.header.stamp = sim_time_;
			msg_laser.header.frame_id = sSensorFrameId;

			pub.publish(msg_laser);
		}
	}
	else
	{
		// Don't know how to emit this observation to ROS!
	}

}  // end of onNewObservation()

/** Creates the string "/<VEH_NAME>/<VAR_NAME>" if there're more than one
 * vehicle in the World, or "/<VAR_NAME>" otherwise. */
std::string MVSimNode::vehVarName(
	const std::string& sVarName, const mvsim::VehicleBase& veh) const
{
	if (mvsim_world_.getListOfVehicles().size() == 1)
	{
		return std::string("/") + sVarName;
	}
	else
	{
		return std::string("/") + veh.getName() + std::string("/") + sVarName;
	}
}
