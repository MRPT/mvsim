/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Offline (no World / no Box2D) unit tests for the "exactly reproducible
// trajectories" feature: mvsim::PoseTrajectoryFollower.

#include <mvsim/PoseTrajectoryFollower.h>

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "test_utils.h"

int g_failures = 0;

using mvsim::PoseTrajectoryFollower;
using Waypoint = PoseTrajectoryFollower::Waypoint;

namespace
{

void test_straight_line_no_loop()
{
	std::cout << "[TEST] straight_line_no_loop\n";

	PoseTrajectoryFollower f;
	f.setLoop(false);
	f.setLookAheadDistance(0.5);
	f.setWaypoints({Waypoint(0.0, 0.0, 0.0), Waypoint(10.0, 10.0, 0.0)});

	EXPECT_NEAR(f.startTime(), 0.0, 1e-9);
	EXPECT_NEAR(f.endTime(), 10.0, 1e-9);

	EXPECT_FALSE(f.finished(-1.0));
	EXPECT_FALSE(f.finished(5.0));
	EXPECT_FALSE(f.finished(9.999));
	EXPECT_TRUE(f.finished(10.0));
	EXPECT_TRUE(f.finished(50.0));

	// Reference position at the midpoint in time should be the midpoint in space:
	const auto refMid = f.referencePose(5.0);
	EXPECT_NEAR(refMid.x, 5.0, 1e-6);
	EXPECT_NEAR(refMid.y, 0.0, 1e-6);
	EXPECT_NEAR(refMid.phi, 0.0, 1e-6);

	// A vehicle exactly on the path, correctly oriented: pure feedforward,
	// no lateral correction needed (lookahead point is straight ahead).
	const mrpt::math::TPose2D onPath(5.0, 0.0, 0.0);
	const auto twist = f.computeTwist(5.0, onPath);
	EXPECT_NEAR(twist.vx, 1.0 /* 10 m / 10 s */, 1e-6);
	EXPECT_NEAR(twist.omega, 0.0, 1e-6);

	// Once finished, the controller must command a full stop:
	const auto twistDone = f.computeTwist(20.0, mrpt::math::TPose2D(10.0, 0.0, 0.0));
	EXPECT_NEAR(twistDone.vx, 0.0, 1e-9);
	EXPECT_NEAR(twistDone.omega, 0.0, 1e-9);
}

void test_lateral_offset_correction()
{
	std::cout << "[TEST] lateral_offset_correction\n";

	PoseTrajectoryFollower f;
	f.setLoop(false);
	f.setLookAheadDistance(1.0);
	f.setWaypoints({Waypoint(0.0, 0.0, 0.0), Waypoint(10.0, 10.0, 0.0)});

	// Vehicle displaced +0.2m to the left of the path (path runs along +X,
	// vehicle at y=+0.2, heading matching the path): must steer right
	// (negative omega) to converge back onto the path.
	const mrpt::math::TPose2D leftOfPath(5.0, 0.2, 0.0);
	const auto twistLeft = f.computeTwist(5.0, leftOfPath);
	EXPECT_LT(twistLeft.omega, 0.0);

	// Symmetric case: displaced to the right must steer left (positive omega).
	const mrpt::math::TPose2D rightOfPath(5.0, -0.2, 0.0);
	const auto twistRight = f.computeTwist(5.0, rightOfPath);
	EXPECT_GT(twistRight.omega, 0.0);
}

void test_loop_wraps_time()
{
	std::cout << "[TEST] loop_wraps_time\n";

	PoseTrajectoryFollower f;
	f.setLoop(true);
	f.setLookAheadDistance(0.3);
	// A 4m square loop, 1 m/s nominal speed (4 s/side -> period 16s):
	f.setWaypoints({
		Waypoint(0.0, 0.0, 0.0),
		Waypoint(4.0, 4.0, 0.0),
		Waypoint(8.0, 4.0, 4.0),
		Waypoint(12.0, 0.0, 4.0),
		Waypoint(16.0, 0.0, 0.0),
	});

	EXPECT_FALSE(f.finished(1000.0));  // never finishes while looping

	const mrpt::math::TPose2D somePose(2.0, 0.0, 0.0);
	const auto twistA = f.computeTwist(2.0, somePose);
	const auto twistB = f.computeTwist(2.0 + 3 * 16.0, somePose);  // 3 full periods later

	EXPECT_NEAR(twistA.vx, twistB.vx, 1e-9);
	EXPECT_NEAR(twistA.omega, twistB.omega, 1e-9);

	const auto poseA = f.referencePose(2.0);
	const auto poseB = f.referencePose(2.0 - 2 * 16.0);	 // 2 full periods earlier
	EXPECT_NEAR(poseA.x, poseB.x, 1e-9);
	EXPECT_NEAR(poseA.y, poseB.y, 1e-9);
}

void test_paused_segment_holds_position()
{
	std::cout << "[TEST] paused_segment_holds_position\n";

	PoseTrajectoryFollower f;
	f.setLoop(false);
	// Drive to (5,0) by t=5, then *wait* at (5,0) until t=10, then continue.
	f.setWaypoints({
		Waypoint(0.0, 0.0, 0.0),
		Waypoint(5.0, 5.0, 0.0),
		Waypoint(10.0, 5.0, 0.0),
		Waypoint(15.0, 10.0, 0.0),
	});

	const mrpt::math::TPose2D atPause(5.0, 0.0, 0.0);
	const auto twist = f.computeTwist(7.5, atPause);
	EXPECT_NEAR(twist.vx, 0.0, 1e-9);
	EXPECT_NEAR(twist.omega, 0.0, 1e-9);
}

void test_reference_pose_heading_at_end_of_path()
{
	std::cout << "[TEST] reference_pose_heading_at_end_of_path\n";

	PoseTrajectoryFollower f;
	f.setLoop(false);
	// A diagonal straight line: heading should be atan2(10,10) = 45 deg
	// everywhere along it, including exactly at (and past) the last waypoint.
	f.setWaypoints({Waypoint(0.0, 0.0, 0.0), Waypoint(10.0, 10.0, 10.0)});

	const double expectedYaw = std::atan2(1.0, 1.0);  // 45 deg

	const auto poseAtEnd = f.referencePose(10.0);
	EXPECT_NEAR(poseAtEnd.phi, expectedYaw, 1e-6);

	// Past the end, wrapTime() clamps to the same terminal point, so the
	// heading must still reflect the incoming direction, not fall back to 0.
	const auto poseAfterEnd = f.referencePose(50.0);
	EXPECT_NEAR(poseAfterEnd.phi, expectedYaw, 1e-6);
}

void test_waypoint_validation()
{
	std::cout << "[TEST] waypoint_validation\n";

	// Fewer than 2 waypoints must throw.
	{
		PoseTrajectoryFollower f;
		bool threw = false;
		try
		{
			f.setWaypoints({Waypoint(0.0, 0.0, 0.0)});
		}
		catch (const std::exception&)
		{
			threw = true;
		}
		EXPECT_TRUE(threw);
	}

	// Repeated timestamps must throw.
	{
		PoseTrajectoryFollower f;
		bool threw = false;
		try
		{
			f.setWaypoints({Waypoint(0.0, 0.0, 0.0), Waypoint(0.0, 1.0, 1.0)});
		}
		catch (const std::exception&)
		{
			threw = true;
		}
		EXPECT_TRUE(threw);
	}

	// Out-of-order waypoints are accepted and internally sorted by time.
	{
		PoseTrajectoryFollower f;
		f.setWaypoints({Waypoint(10.0, 10.0, 0.0), Waypoint(0.0, 0.0, 0.0)});
		EXPECT_NEAR(f.startTime(), 0.0, 1e-9);
		EXPECT_NEAR(f.endTime(), 10.0, 1e-9);
	}
}

}  // namespace

int main()
{
	test_straight_line_no_loop();
	test_lateral_offset_correction();
	test_loop_wraps_time();
	test_paused_segment_holds_position();
	test_reference_pose_heading_at_end_of_path();
	test_waypoint_validation();

	if (g_failures == 0)
	{
		std::cout << "ALL TESTS PASSED\n";
	}
	else
	{
		std::cerr << g_failures << " TEST(S) FAILED\n";
	}
	return g_failures == 0 ? 0 : 1;
}
