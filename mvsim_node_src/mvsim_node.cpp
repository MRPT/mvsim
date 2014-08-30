#include <ros/ros.h>

//#include <std_msgs/String.h>

// Conversions ROS msgs <-> MRPT structs
//#include <mrpt_bridge/mrpt_bridge.h>

// MRPT stuff:
#include <mrpt/version.h>
#if MRPT_VERSION < 0x100
#	error "This program requires MRPT >= 1.0.0"
#endif


using namespace std;


/** The main */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "mvsim_node");


	return 0;
}


