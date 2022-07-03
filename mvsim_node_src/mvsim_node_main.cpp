/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/round.h>

#include "mvsim/mvsim_node_core.h"

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node->
 *----------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
	// Set up ROS.
#if PACKAGE_ROS_VERSION == 1
	ros::init(argc, argv, "mvsim");
	ros::NodeHandle n;
#else
	rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("mvsim");
#endif

	try
	{
		// Create a "Node" object.
		std::shared_ptr<MVSimNode> node = std::make_shared<MVSimNode>(n);

		// Declare variables that can be modified by launch file or command
		// line.
		std::string world_file;

		double rate = 100.0;

#if PACKAGE_ROS_VERSION == 1

		// Initialize node parameters from launch file or command line.
		// Use a private node handle so that multiple instances of the node can
		// be run simultaneously while using different parameters.
		ros::NodeHandle private_node_handle_("~");
		private_node_handle_.param("simul_rate", rate, 100.0);
		private_node_handle_.param("world_file", world_file, std::string(""));
#else
		n->get_parameter("world_file", world_file);
		n->get_parameter("simul_rate", rate);
		ASSERT_(rate > 0);
		const auto periodMs = mrpt::round(1e6 / static_cast<double>(rate));
#endif

		// Launch mvsim:
		node->launch_mvsim_server();

		// Init world model:
		if (!world_file.empty()) node->loadWorldModel(world_file);

		// Attach world as a mvsim communications node:
		node->mvsim_world_.connectToServer();

#if PACKAGE_ROS_VERSION == 1
		// Set up a dynamic reconfigure server.
		// Do this before parameter server, else some of the parameter server
		// values can be overwritten.
		dynamic_reconfigure::Server<mvsim::mvsimNodeConfig> dr_srv;
		dynamic_reconfigure::Server<mvsim::mvsimNodeConfig>::CallbackType cb;
		cb = boost::bind(&MVSimNode::configCallback, node.get(), _1, _2);
		dr_srv.setCallback(cb);
#endif

		// Tell ROS how fast to run this node->
#if PACKAGE_ROS_VERSION == 1
		ros::Rate r(rate);

		// Main loop.
		while (n.ok())
		{
			node->spin();
			ros::spinOnce();
			r.sleep();
		}
#else
		auto ros_clock = rclcpp::Clock::make_shared();
		auto timer_ = rclcpp::create_timer(
			n, ros_clock, std::chrono::microseconds(periodMs),
			[&]() { node->spin(); });

		rclcpp::on_shutdown([&]() {
			std::cout << "[rclcpp::on_shutdown] Destroying MVSIM node..."
					  << std::endl;
			node.reset();
		});

		rclcpp::spin(n);
		rclcpp::shutdown();
#endif

		return 0;
	}
	catch (const std::exception& e)
	{
#if PACKAGE_ROS_VERSION == 1
		std::cerr << e.what() << std::endl;
#else
		RCLCPP_ERROR_STREAM(
			n->get_logger(), "Exception in main node body:\n"
								 << e.what());
#endif
		return 1;
	}

}  // end main()
