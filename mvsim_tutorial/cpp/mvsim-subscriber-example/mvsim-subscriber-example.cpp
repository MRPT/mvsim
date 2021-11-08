
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/mvsim-msgs/GenericObservation.pb.h>
#include <mvsim/mvsim-msgs/TimeStampedPose.pb.h>

#include <chrono>
#include <iostream>
#include <thread>

void myPoseCallback(const mvsim_msgs::TimeStampedPose& p)
{
	std::cout << "topic callback: " << p.DebugString() << std::endl;
}

void mySensorCallback(const mvsim_msgs::GenericObservation& o)
{
	const std::vector<uint8_t> data(
		o.mrptserializedobservation().begin(),
		o.mrptserializedobservation().end());

	mrpt::serialization::CSerializable::Ptr obj;
	mrpt::serialization::OctetVectorToObject(data, obj);

	mrpt::obs::CObservation::Ptr obs =
		std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj);
	ASSERT_(obs);

	std::cout << "sensor callback: " << obs->asString() << "\n";
}

int main(int argc, char** argv)
{
	try
	{
		mvsim::Client client("mvsim-subscriber-example");
		client.setVerbosityLevel(mrpt::system::LVL_DEBUG);

		client.connect();

		client.subscribeTopic<mvsim_msgs::TimeStampedPose>(
			"/r1/pose", myPoseCallback);

		client.subscribeTopic<mvsim_msgs::GenericObservation>(
			"/r1/laser1", mySensorCallback);

		std::this_thread::sleep_for(std::chrono::seconds(5));

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e);
		return 1;
	}
}
