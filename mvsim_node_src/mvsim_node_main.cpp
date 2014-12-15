/**
  */

#include "mvsim/mvsim_node_core.h"

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
	// Set up ROS.
	ros::init(argc, argv, "mvsim");
	ros::NodeHandle n;


	// Create a "Node" object.
	MVSimNode node;

	// Declare variables that can be modified by launch file or command line.
	int rate;
	std::string world_file;

	// Initialize node parameters from launch file or command line.
	// Use a private node handle so that multiple instances of the node can be run simultaneously
	// while using different parameters.
	ros::NodeHandle private_node_handle_("~");
	private_node_handle_.param("simul_rate", rate, 100);
	private_node_handle_.param("world_file", world_file, std::string(""));
	private_node_handle_.param("realtime_factor", node.realtime_factor_, 1.0);
	private_node_handle_.param("gui_refresh_period", node.gui_refresh_period_ms_, node.gui_refresh_period_ms_);
	private_node_handle_.param("show_gui", node.m_show_gui, node.m_show_gui);

	// Init world model:
	if (!world_file.empty())
		node.loadWorldModel(world_file);


	// Set up a dynamic reconfigure server.
	// Do this before parameter server, else some of the parameter server
	// values can be overwritten.
	dynamic_reconfigure::Server<mvsim::mvsimNodeConfig> dr_srv;
	dynamic_reconfigure::Server<mvsim::mvsimNodeConfig>::CallbackType cb;
	cb = boost::bind(&MVSimNode::configCallback, &node, _1, _2);
	dr_srv.setCallback(cb);


	// Create a publisher and name the topic.
	//ros::Publisher pub_message = n.advertise<node_example::NodeExampleData>("example", 10);
	// Name the topic, message queue, callback function with class name, and object containing callback function.
	//ros::Subscriber sub_message = n.subscribe("example", 1000, &NodeExample::messageCallback, node_example);

	// Tell ROS how fast to run this node.
	ros::Rate r(rate);

	// Main loop.
	while (n.ok())
	{
		node.spin();
		ros::spinOnce();
		r.sleep();
	}

	return 0;

} // end main()


