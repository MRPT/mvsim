/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Per-wheel joint commands ("ros2_control" controller class, without ROS):
// velocity and effort commands, joint states, and determinism.

#include <mvsim/VehicleBase.h>
#include <mvsim/WheelJointsInterface.h>
#include <mvsim/World.h>

#include <iostream>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{
const char* kWorldXml = R"(
<mvsim_world version="1.0">
  <simul_timestep>0.005</simul_timestep>
  <element class="ground_grid"/>
  <vehicle name="r1">
    <dynamics class="differential_4_wheels">
      <lf_wheel pos="0.13  0.16" mass="1.0" width="0.03" diameter="0.20" />
      <rf_wheel pos="0.13 -0.16" mass="1.0" width="0.03" diameter="0.20" joint_name="custom_rf" />
      <lr_wheel pos="-0.13  0.16" mass="1.0" width="0.03" diameter="0.20" />
      <rr_wheel pos="-0.13 -0.16" mass="1.0" width="0.03" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
      <controller class="ros2_control">
        <command_interface>${CMD_IF}</command_interface>
        <KP>18.3</KP> <KI>474.5</KI> <max_torque>100</max_torque>
      </controller>
    </dynamics>
    <friction class="default"> <mu>0.8</mu> <C_damping>0.05</C_damping> </friction>
    <init_pose>0 0 0</init_pose>
  </vehicle>
</mvsim_world>
)";

struct Result
{
	mrpt::math::TPose3D pose;
	std::vector<mvsim::WheelJointsInterface::JointState> states;
};

Result run(const std::string& cmdIf, const std::vector<double>& cmds, double duration)
{
	std::string xml = kWorldXml;
	const std::string tag = "${CMD_IF}";
	xml.replace(xml.find(tag), tag.size(), cmdIf);

	mvsim::World world;
	world.headless(true);
	world.load_from_XML(xml, ".");

	auto veh = world.getListOfVehicles().begin()->second;
	auto* joints = veh->getControllerInterface()->wheelJointsInterface();
	EXPECT_TRUE(joints != nullptr);

	// Joint names:
	EXPECT_TRUE(veh->getWheelInfo(0).joint_name == "lr_wheel_joint");
	EXPECT_TRUE(veh->getWheelInfo(1).joint_name == "rr_wheel_joint");
	EXPECT_TRUE(veh->getWheelInfo(2).joint_name == "lf_wheel_joint");
	EXPECT_TRUE(veh->getWheelInfo(3).joint_name == "custom_rf");

	joints->setCommands(cmds);
	for (double t = 0; t < duration; t += 0.01)
	{
		world.run_simulation(0.01);
	}
	return {veh->getPose(), joints->getStates()};
}

void test_velocity()
{
	// 5 rad/s * 0.1 m = 0.5 m/s forward, for 4 s:
	const auto r = run("velocity", {5.0, 5.0, 5.0, 5.0}, 4.0);
	std::cout << "velocity: final pose " << r.pose << "\n";
	EXPECT_NEAR(r.pose.x, 2.0, 0.15);
	EXPECT_NEAR(r.pose.y, 0.0, 0.01);
	EXPECT_TRUE(r.states.size() == 4U);
	for (const auto& s : r.states)
	{
		EXPECT_NEAR(s.velocity, 5.0, 0.1);
		// joint position = traveled distance / radius:
		EXPECT_NEAR(s.position, r.pose.x / 0.1, 0.5);
		EXPECT_GT(s.position, 0.0);
	}

	// Turning in place (skid steer): left wheels backwards, right forward
	const auto r2 = run("velocity", {-3.0, 3.0, -3.0, 3.0}, 2.0);
	std::cout << "turn: final pose " << r2.pose << "\n";
	EXPECT_GT(r2.pose.yaw, 0.3);
	EXPECT_NEAR(r2.pose.x, 0.0, 0.2);  // skid steer drifts a bit
}

void test_effort()
{
	const auto r = run("effort", {1.0, 1.0, 1.0, 1.0}, 2.0);
	std::cout << "effort: final pose " << r.pose << "\n";
	EXPECT_GT(r.pose.x, 0.1);
	for (const auto& s : r.states)
	{
		EXPECT_NEAR(s.effort, 1.0, 1e-9);
	}
}

void test_determinism()
{
	const auto a = run("velocity", {4.0, 6.0, 4.0, 6.0}, 3.0);
	const auto b = run("velocity", {4.0, 6.0, 4.0, 6.0}, 3.0);
	EXPECT_NEAR(a.pose.x, b.pose.x, 1e-12);
	EXPECT_NEAR(a.pose.y, b.pose.y, 1e-12);
	EXPECT_NEAR(a.pose.yaw, b.pose.yaw, 1e-12);
}
}  // namespace

int main()
{
	try
	{
		test_velocity();
		test_effort();
		test_determinism();
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << std::endl;
		return 1;
	}
	if (g_failures)
	{
		std::cerr << g_failures << " failure(s)\n";
		return 1;
	}
	std::cout << "All tests passed.\n";
	return 0;
}
