/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// End-to-end (full World + Box2D) tests for the "trajectory" controller
// class, i.e. the "exactly reproducible trajectories" feature: a vehicle
// driven by a closed-form, time-parameterized (t,x,y) polyline read from
// the <controller class="trajectory"> XML node.

#include <mrpt/math/TPose3D.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/World.h>

#include <iostream>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{

// A 4x4m square, driven once per side every 4 seconds (1 m/s nominal speed).
const char* kSquareWaypoints = R"(
	<waypoint t="0.0"  x="0.0" y="0.0" />
	<waypoint t="4.0"  x="4.0" y="0.0" />
	<waypoint t="8.0"  x="4.0" y="4.0" />
	<waypoint t="12.0" x="0.0" y="4.0" />
	<waypoint t="16.0" x="0.0" y="0.0" />
)";

mvsim::VehicleBase::Ptr make_vehicle(
	mvsim::World& world, const std::string& dynClass, bool loop, const char* waypointsXml,
	double lookAheadDistance = 0.3)
{
	std::string wheels_config;
	if (dynClass == "differential")
	{
		wheels_config = R"(
        <l_wheel pos="0  0.5" mass="1.0" width="0.03" diameter="0.20" />
        <r_wheel pos="0 -0.5" mass="1.0" width="0.03" diameter="0.20" />
		)";
	}
	else if (dynClass == "ackermann")
	{
		wheels_config = R"(
        <rl_wheel pos="0  1" mass="6.0" width="0.30" diameter="0.62" />
        <rr_wheel pos="0 -1" mass="6.0" width="0.30" diameter="0.62" />
        <fl_wheel mass="6.0" width="0.30" diameter="0.62" />
        <fr_wheel mass="6.0" width="0.30" diameter="0.62" />
        <f_wheels_x>1.3</f_wheels_x>
        <f_wheels_d>2.0</f_wheels_d>
        <max_steer_ang_deg>60.0</max_steer_ang_deg>
		)";
	}

	const std::string xml =
		"<vehicle name=\"test_veh\">"
		"  <dynamics class=\"" +
		dynClass +
		"\">"
		"    <chassis linear_damping=\"0\" angular_damping=\"0\" />" +
		wheels_config + "    <controller class=\"trajectory\" loop=\"" +
		(loop ? std::string("true") : std::string("false")) + "\">" + "      <lookahead_distance>" +
		std::to_string(lookAheadDistance) + "</lookahead_distance>" + waypointsXml +
		"    </controller>"
		"  </dynamics>"
		"<init_pose>0 0 0</init_pose>"
		"</vehicle>";

	auto veh = mvsim::VehicleBase::factory(&world, xml);
	world.insert_vehicle(veh);
	return veh;
}

void run_square_test(const std::string& dynClass)
{
	std::cout << "[TEST] square trajectory (no loop) | " << dynClass << "\n";

	const double simulStep = 0.001;

	mvsim::World world;
	world.headless(true);
	world.set_gravity(9.81);
	world.set_simul_timestep(simulStep);
	world.internal_initialize();

	auto veh = make_vehicle(world, dynClass, /*loop*/ false, kSquareWaypoints);

	double simulatedSoFar = 0;
	const auto run_until = [&](double t)
	{
		const int steps = static_cast<int>((t - simulatedSoFar) / simulStep);
		for (int i = 0; i < steps; ++i)
		{
			world.run_simulation(simulStep);
		}
		simulatedSoFar = t;
	};

	// Corners should be reached close to their scheduled time:
	run_until(4.0);
	auto pose = veh->getPose();
	std::cout << "  @t=4  pose=" << pose << "\n";
	EXPECT_NEAR(pose.x, 4.0, 0.3);
	EXPECT_NEAR(pose.y, 0.0, 0.3);

	run_until(8.0);
	pose = veh->getPose();
	std::cout << "  @t=8  pose=" << pose << "\n";
	EXPECT_NEAR(pose.x, 4.0, 0.3);
	EXPECT_NEAR(pose.y, 4.0, 0.3);

	run_until(12.0);
	pose = veh->getPose();
	std::cout << "  @t=12 pose=" << pose << "\n";
	EXPECT_NEAR(pose.x, 0.0, 0.3);
	EXPECT_NEAR(pose.y, 4.0, 0.3);

	run_until(16.0);
	pose = veh->getPose();
	std::cout << "  @t=16 pose=" << pose << "\n";
	EXPECT_NEAR(pose.x, 0.0, 0.3);
	EXPECT_NEAR(pose.y, 0.0, 0.3);

	// Non-looping trajectory: once finished, the vehicle must remain still.
	run_until(19.0);  // 3 more seconds past the trajectory's end at t=16
	const auto poseAfterEnd = veh->getPose();
	std::cout << "  @t=19 (after end) pose=" << poseAfterEnd << "\n";
	EXPECT_NEAR(poseAfterEnd.x, pose.x, 0.05);
	EXPECT_NEAR(poseAfterEnd.y, pose.y, 0.05);
}

void run_loop_test(const std::string& dynClass)
{
	std::cout << "[TEST] square trajectory (loop) | " << dynClass << "\n";

	const double simulStep = 0.001;

	mvsim::World world;
	world.headless(true);
	world.set_gravity(9.81);
	world.set_simul_timestep(simulStep);
	world.internal_initialize();

	auto veh = make_vehicle(world, dynClass, /*loop*/ true, kSquareWaypoints);

	const int steps = static_cast<int>(16.0 / simulStep);

	// First loop:
	for (int i = 0; i < steps; ++i)
	{
		world.run_simulation(simulStep);
	}
	const auto poseAfterFirstLoop = veh->getPose();
	std::cout << "  @t=16 (1st loop) pose=" << poseAfterFirstLoop << "\n";
	EXPECT_NEAR(poseAfterFirstLoop.x, 0.0, 0.3);
	EXPECT_NEAR(poseAfterFirstLoop.y, 0.0, 0.3);

	// Second loop: the vehicle must keep moving and come back close to the
	// same corner again (i.e. it must NOT have stopped, unlike the no-loop
	// case).
	for (int i = 0; i < steps; ++i)
	{
		world.run_simulation(simulStep);
	}
	const auto poseAfterSecondLoop = veh->getPose();
	std::cout << "  @t=32 (2nd loop) pose=" << poseAfterSecondLoop << "\n";
	EXPECT_NEAR(poseAfterSecondLoop.x, 0.0, 0.3);
	EXPECT_NEAR(poseAfterSecondLoop.y, 0.0, 0.3);
}

}  // namespace

int main()
{
	run_square_test("differential");
	run_square_test("ackermann");

	run_loop_test("differential");
	run_loop_test("ackermann");

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
