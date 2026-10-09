/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// A 2D LiDAR tilted down must see the ground when ray-traced in 3D
// (raytrace_3d=true). Requires off-screen OpenGL rendering (skipped if not
// available).

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mvsim/World.h>

#include <atomic>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "test_utils.h"

int g_failures = 0;

namespace
{
// LiDAR 1.0 m over a flat ground, pitched down "pitchDeg", optionally inside
// a square room of walls:
std::string worldXml(
	bool raytrace3d, double pitchDeg = 30.0, double fovDeg = 90.0, int nRays = 91,
	bool walls = false)
{
	const std::string wallsXml = !walls ? std::string() : R"(
  <block:class name="wall">
    <static>true</static> <zmin>0</zmin> <zmax>3</zmax>
    <shape><pt>-0.1 -10</pt><pt>0.1 -10</pt><pt>0.1 10</pt><pt>-0.1 10</pt></shape>
  </block:class>
  <block name="w1" class="wall"> <init_pose>4.6 0 0</init_pose> </block>
  <block name="w2" class="wall"> <init_pose>-3.4 0 0</init_pose> </block>
  <block name="w3" class="wall"> <init_pose>0.5 3 90</init_pose> </block>
  <block name="w4" class="wall"> <init_pose>0.5 -5 90</init_pose> </block>
)";
	return std::string(R"(
<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <element class="horizontal_plane">
    <cutout>-50 -50 50 50</cutout>
  </element>)") +
		   wallsXml + R"(
  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
    <sensor class="laser" name="laser1">
      <pose_3d>0.5 0 1.0 0 )" +
		   std::to_string(pitchDeg) + R"( 0</pose_3d>
      <fov_degrees>)" +
		   std::to_string(fovDeg) + R"(</fov_degrees>
      <nrays>)" +
		   std::to_string(nRays) + R"(</nrays>
      <sensor_period>0.05</sensor_period>
      <range_std_noise>0</range_std_noise>
      <angle_std_noise_deg>0</angle_std_noise_deg>
      <max_range>30</max_range>
      <raytrace_3d>)" +
		   (raytrace3d ? "true" : "false") + R"(</raytrace_3d>
    </sensor>
  </vehicle>
</mvsim_world>
)";
}

/** Runs the world and returns the first scan, or nullptr if no OpenGL */
mrpt::obs::CObservation2DRangeScan::Ptr getScan(const std::string& xml)
{
	mvsim::World world;
	world.headless(true);
	world.load_from_XML(xml, ".");

	std::mutex obsMtx;
	mrpt::obs::CObservation2DRangeScan::Ptr scan;
	world.registerCallbackOnObservation(
		[&](const mvsim::Simulable&, const mrpt::obs::CObservation::Ptr& o)
		{
			if (auto s = std::dynamic_pointer_cast<mrpt::obs::CObservation2DRangeScan>(o); s)
			{
				std::lock_guard<std::mutex> lck(obsMtx);
				scan = s;
			}
		});

	std::atomic_bool stop = false;
	std::thread th(
		[&]()
		{
			try
			{
				while (!stop && !world.simulator_must_close())
				{
					world.internalGraphicsLoopTasksForSimulation();
					std::this_thread::sleep_for(std::chrono::milliseconds(2));
				}
				world.internalFreeOpenGLResourcesForSimulation();
			}
			catch (const std::exception& e)
			{
				std::cerr << e.what() << std::endl;
			}
		});

	for (int i = 0; i < 30 && !world.simulator_must_close(); i++)
	{
		world.run_simulation(0.01);
		std::lock_guard<std::mutex> lck(obsMtx);
		if (scan)
		{
			break;
		}
	}
	stop = true;
	th.join();
	std::lock_guard<std::mutex> lck(obsMtx);
	return scan;
}

void test_tilted_3d()
{
	const auto scan = getScan(worldXml(true));
	if (!scan)
	{
		std::cout << "[SKIPPED] No OpenGL rendering available.\n";
		return;
	}
	const size_t N = scan->getScanSize();
	EXPECT_TRUE(N == 91U);

	// Ray at angle "a" in the tilted scan plane: it reaches the ground (z=-1
	// wrt the sensor) at range = h / (sin(pitch) * cos(a))
	const double h = 1.0;
	const double pitch = mrpt::DEG2RAD(30.0);
	for (size_t i = 0; i < N; i += 15)
	{
		const double a = scan->getScanAngle(i);
		const double expected = h / (std::sin(pitch) * std::cos(a));
		EXPECT_TRUE(scan->getScanRangeValidity(i));
		EXPECT_NEAR(scan->getScanRange(i), expected, 0.02 * expected);
	}
	std::cout << "Center ray range: " << scan->getScanRange(N / 2) << " (expected 2.0)\n";
}

void test_tilted_2d_mode()
{
	// 2D mode ignores the tilt: nothing to see in an empty world
	const auto scan = getScan(worldXml(false));
	EXPECT_TRUE(scan != nullptr);
	if (!scan)
	{
		return;
	}
	size_t valid = 0;
	for (size_t i = 0; i < scan->getScanSize(); i++)
	{
		valid += scan->getScanRangeValidity(i) ? 1 : 0;
	}
	EXPECT_TRUE(valid == 0U);
}
// Without tilt, the 3D (OpenGL) and 2D (planar) modes must agree on vertical
// walls, for apertures needing one or several renders:
void test_3d_vs_2d(double fovDeg, int nRays)
{
	const auto s3 = getScan(worldXml(true, 0.0, fovDeg, nRays, true));
	if (!s3)
	{
		std::cout << "[SKIPPED] No OpenGL rendering available.\n";
		return;
	}
	const auto s2 = getScan(worldXml(false, 0.0, fovDeg, nRays, true));
	EXPECT_TRUE(s2 && s2->getScanSize() == s3->getScanSize());
	if (!s2)
	{
		return;
	}
	size_t compared = 0;
	for (size_t i = 0; i < s3->getScanSize(); i++)
	{
		if (!s2->getScanRangeValidity(i) || !s3->getScanRangeValidity(i))
		{
			continue;
		}
		EXPECT_NEAR(s3->getScanRange(i), s2->getScanRange(i), 0.03 * s2->getScanRange(i));
		compared++;
	}
	EXPECT_GT(compared, 0.9 * s3->getScanSize());
}
}  // namespace

int main()
{
	try
	{
		test_tilted_3d();
		test_tilted_2d_mode();
		test_3d_vs_2d(90.0, 91);
		test_3d_vs_2d(180.0, 361);
		test_3d_vs_2d(270.0, 541);
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
