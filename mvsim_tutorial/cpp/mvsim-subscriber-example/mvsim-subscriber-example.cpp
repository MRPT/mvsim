
#include <mvsim/Comms/Client.h>
#include <mvsim/mvsim-msgs/Pose.pb.h>

#include <chrono>
#include <iostream>
#include <thread>

void myCallback(const mvsim_msgs::Pose& p)
{
	std::cout << "topic callback: " << p.DebugString() << std::endl;
}

int main(int argc, char** argv)
{
	try
	{
		mvsim::Client client;
		client.setVerbosityLevel(mrpt::system::LVL_DEBUG);

		client.connect();

		client.subscribeTopic<mvsim_msgs::Pose>("/r1/pose", myCallback);

		std::this_thread::sleep_for(std::chrono::seconds(5));

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e);
		return 1;
	}
}
