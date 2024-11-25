
#include <mrpt/core/exceptions.h>
#include <mvsim/Comms/Client.h>

#include <iostream>

// mvsim_msgs:
#include <mvsim/mvsim-msgs/GenericAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvSetPoseTwist.pb.h>

int main(int argc, char** argv)
{
	try
	{
		mvsim::Client client;
		client.connect();

		for (int i = 0; i < 20; i++)
		{
			// Request:
			mvsim_msgs::SrvSetPose req;

			req.set_objectid("r1");
			req.mutable_pose()->set_x(i * 0.50);
			req.mutable_pose()->set_y(0.0);
			req.mutable_pose()->set_z(0.0);
			req.mutable_pose()->set_yaw(0.0);
			req.mutable_pose()->set_pitch(0.0);
			req.mutable_pose()->set_roll(0.0);

			mvsim_msgs::GenericAnswer ans;
			client.callService("set_pose", req, ans);

			std::cout << "called service. Input:\n"
					  << req.DebugString() << "Response:\n"
					  << ans.DebugString() << std::endl;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e);
		return 1;
	}
}
