/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/math/TPose3D.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <cmath>
#include <iostream>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{

void run_drift_test(const std::string& ctrl_class, const std::string& test_name)
{
	std::cout << "[TEST] " << test_name << " | " << ctrl_class << "\n";

	const double simulStep = 0.005;
	const double final_time = 3.0;
	const int steps = static_cast<int>(final_time / simulStep);

	// Create world with an elevation map via XML
	const std::string worldXml = R"(
<mvsim_world version="1.0">
  <simul_timestep>)" + std::to_string(simulStep) +
								 R"(</simul_timestep>
  <gravity>9.81</gravity>

  <element class="elevation_map">
    <resolution>6.0</resolution>
    <elevation_data_matrix>
      [1.0 0.0 -1.0;
       1.5 0.0 -1.5;
       1.0 0.0 -1.0]
    </elevation_data_matrix>
    <corner_min_x>-8</corner_min_x>
    <corner_min_y>-8</corner_min_y>
  </element>

  <vehicle name="test_veh">
    <dynamics class="differential_4_wheels">
      <lf_wheel pos="0.3  0.6" mass="1.0" width="0.03" diameter="0.20" />
      <rf_wheel pos="0.3 -0.6" mass="1.0" width="0.03" diameter="0.20" />
      <lr_wheel pos="-0.3  0.6" mass="1.0" width="0.03" diameter="0.20" />
      <rr_wheel pos="-0.3 -0.6" mass="1.0" width="0.03" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
      <controller class=")" + ctrl_class +
								 R"(">
        <KP>4</KP><KI>3</KI><KD>0</KD>
        <max_torque>20</max_torque>
      </controller>
    </dynamics>
    <friction class="default">
      <mu>0.8</mu>
      <C_damping>0.05</C_damping>
    </friction>
    <init_pose>0 0 0</init_pose>
  </vehicle>
</mvsim_world>
)";

	mvsim::World world;
	world.headless(true);
	world.load_from_XML(worldXml, ".");
	// Note: load_from_XML calls internal_initialize(), so no need to call it again.

	// Get the vehicle
	auto& vehs = world.getListOfVehicles();
	ASSERT_(!vehs.empty());
	auto& veh = vehs.begin()->second;

	// Record initial pose
	const auto initPose = veh->getPose();
	std::printf("  Init pose: x=%.6f y=%.6f yaw=%.6f\n", initPose.x, initPose.y, initPose.yaw);

	// Run simulation with ZERO velocity command (vehicle should stay still)
	// No need to set twist , default is zero.

	for (int i = 0; i < steps; ++i)
	{
		world.run_simulation(simulStep);
	}

	const auto finalPose = veh->getPose();
	std::printf("  Final pose: x=%.6f y=%.6f yaw=%.6f\n", finalPose.x, finalPose.y, finalPose.yaw);

	const double drift_x = std::abs(finalPose.x - initPose.x);
	const double drift_y = std::abs(finalPose.y - initPose.y);
	const double drift_yaw = std::abs(finalPose.yaw - initPose.yaw);

	std::printf("  Drift: dx=%.6f dy=%.6f dyaw=%.6f\n", drift_x, drift_y, drift_yaw);

	// Vehicle should not drift more than 1mm or 0.01 rad in 3 seconds
	const double max_drift_pos = 0.001;	 // 1 mm
	const double max_drift_yaw = 0.01;	// ~0.6 deg

	EXPECT_LT(drift_x, max_drift_pos);
	EXPECT_LT(drift_y, max_drift_pos);
	EXPECT_LT(drift_yaw, max_drift_yaw);
}

}  // namespace

int main()
{
	try
	{
		run_drift_test("twist_ideal", "Drift on elevation (ideal controller)");
		run_drift_test("twist_pid", "Drift on elevation (PID controller)");

		if (g_failures == 0)
		{
			std::printf("\nAll drift-on-elevation tests passed successfully.\n");
		}
		else
		{
			std::fprintf(stderr, "\n%d test(s) FAILED.\n", g_failures);
		}

		return g_failures == 0 ? 0 : 1;
	}
	catch (const std::exception& e)
	{
		std::fprintf(stderr, "Exception caught: %s\n", e.what());
		return 1;
	}
}
