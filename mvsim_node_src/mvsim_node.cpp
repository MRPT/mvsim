/**
  */

#include "mvsim/mvsim_node_core.h"

/*------------------------------------------------------------------------------
 * MVSimNode()
 * Constructor.
 *----------------------------------------------------------------------------*/

MVSimNode::MVSimNode()
{
}

/*------------------------------------------------------------------------------
 * ~MVSimNode()
 * Destructor.
 *----------------------------------------------------------------------------*/

MVSimNode::~MVSimNode()
{
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

