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

class MVSimNode
{
public:
  //! Constructor.
  MVSimNode();

  //! Destructor.
  ~MVSimNode();

  //! Callback function for dynamic reconfigure server.
  void configCallback(mvsim::mvsimNodeConfig &config, uint32_t level);

  //! Publish the message.
  void publishMessage(ros::Publisher *pub_message);

  //! Callback function for subscriber.
//  void messageCallback(const mvsim_node::NodeExampleData::ConstPtr &msg);

  //! The actual message.
  std::string message;

  //! The first integer to use in addition.
  int a;

  //! The second integer to use in addition.
  int b;
};

#endif // SR_MVSIM_NODE_CORE_H
