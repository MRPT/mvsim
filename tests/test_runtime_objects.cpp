/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Runtime (visual-only) objects: spawn/move/remove from the API, ground truth
// snapshots, and visibility of ground decals to a camera sensor (off-screen
// rendering; skipped if no OpenGL context can be created).

#include <mrpt/obs/CObservationImage.h>
#include <mvsim/World.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "test_utils.h"

int g_failures = 0;

namespace
{
using Shape = mvsim::RuntimeObjectDescription::Shape;

// A robot with a camera 3 m ahead of it, 2 m high, looking straight down.
const char* kWorldXml = R"(
<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <element class="horizontal_plane">
    <cutout>-20 -20 20 20</cutout>
    <color>#808080</color>
  </element>
  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
      <controller class="twist_pid" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
    ${CAMERA_SENSOR}
  </vehicle>
</mvsim_world>
)";

const char* kCameraXml = R"(
    <sensor class="camera" name="cam">
      <pose_3d>3.0 0.0 2.0 -90.0 0.0 180.0</pose_3d>
      <sensor_period>0.05</sensor_period>
      <ncols>64</ncols> <nrows>48</nrows>
      <cx>32</cx> <cy>24</cy> <fx>40</fx> <fy>40</fy>
      <clip_min>0.01</clip_min> <clip_max>100</clip_max>
      <preview_win_visible>false</preview_win_visible>
    </sensor>
)";

std::string worldXml(bool withCamera)
{
	std::string s = kWorldXml;
	const std::string tag = "${CAMERA_SENSOR}";
	s.replace(s.find(tag), tag.size(), withCamera ? kCameraXml : "");
	return s;
}

mvsim::RuntimeObjectDescription makeDecal(const std::string& name, double x, double y)
{
	mvsim::RuntimeObjectDescription d;
	d.name = name;
	d.shape = Shape::Rectangle;
	d.pose = {x, y, 0.005, 0, 0, 0};
	d.size = {0.6, 0.6, 0};
	d.color = {0xff, 0x00, 0x00};
	return d;
}

void test_container(mvsim::World& world)
{
	auto& ro = world.runtimeObjects();
	EXPECT_TRUE(ro.size() == 0U);

	ro.spawn(makeDecal("marks/1", 1, 2));
	ro.spawn(makeDecal("marks/2", 3, 4));
	ro.spawn(makeDecal("target", 5, 6));
	EXPECT_TRUE(ro.size() == 3U);

	// Replace by name:
	ro.spawn(makeDecal("target", 7, 8));
	EXPECT_TRUE(ro.size() == 3U);
	EXPECT_NEAR(ro.getPose("target")->x, 7.0, 1e-9);

	EXPECT_TRUE(ro.setPose("target", {1, 1, 0, 0, 0, 0}));
	EXPECT_TRUE(!ro.setPose("nonexistent", {1, 1, 0, 0, 0, 0}));
	EXPECT_NEAR(ro.getPose("target")->y, 1.0, 1e-9);

	// Invalid input:
	{
		mvsim::RuntimeObjectDescription bad;
		bad.name = "bad";
		bad.shape = Shape::Polygon;
		bool thrown = false;
		try
		{
			ro.spawn(bad);
		}
		catch (const std::exception&)
		{
			thrown = true;
		}
		EXPECT_TRUE(thrown);
	}

	// Ground truth includes vehicles and runtime objects:
	{
		world.run_simulation(0.05);
		const auto snap = world.getGroundTruthSnapshot();
		bool hasR1 = false;
		bool hasTarget = false;
		for (const auto& o : snap.objects)
		{
			hasR1 = hasR1 || o.name == "r1";
			hasTarget = hasTarget || o.name == "target";
		}
		EXPECT_TRUE(hasR1);
		EXPECT_TRUE(hasTarget);
		EXPECT_GT(snap.simul_time, 0.0);
		EXPECT_TRUE(world.getGroundTruthSnapshot("marks/").objects.size() == 2U);
	}

	EXPECT_TRUE(ro.removeByPrefix("marks/") == 2U);
	EXPECT_TRUE(ro.remove({"target", "nonexistent"}) == 1U);
	EXPECT_TRUE(ro.size() == 0U);

	// Many spawn/remove cycles, with the scene updated in between:
	for (int i = 0; i < 5000; i++)
	{
		ro.spawn(makeDecal("tmp" + std::to_string(i % 50), i * 0.01, 0));
		if (i % 100 == 99)
		{
			EXPECT_TRUE(ro.removeByPrefix("tmp") == 50U);
		}
		if (i % 10 == 0)
		{
			world.run_simulation(0.01);
		}
	}
	EXPECT_TRUE(ro.size() == 0U);
}

/** Returns true if the center pixel of the last camera image is red */
bool centerIsRed(const mrpt::obs::CObservationImage& obs)
{
	const auto& img = obs.image;
	const auto x = static_cast<int>(img.getWidth() / 2);
	const auto y = static_cast<int>(img.getHeight() / 2);
	const auto r = img.at<uint8_t>(x, y, 0);
	const auto g = img.at<uint8_t>(x, y, 1);
	const auto b = img.at<uint8_t>(x, y, 2);
	std::cout << "Center pixel RGB: " << int(r) << " " << int(g) << " " << int(b) << "\n";
	return r > g + 100 && r > b + 100;
}

void test_camera(mvsim::World& world)
{
	std::mutex obsMtx;
	mrpt::obs::CObservationImage::Ptr lastObs;
	world.registerCallbackOnObservation(
		[&](const mvsim::Simulable&, const mrpt::obs::CObservation::Ptr& o)
		{
			if (auto img = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o); img)
			{
				std::lock_guard<std::mutex> lck(obsMtx);
				lastObs = img;
			}
		});

	// Off-screen rendering thread, as in "mvsim launch --headless":
	std::atomic_bool glFailed = false;
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
				glFailed = true;
			}
		});

	const auto runAndGetImage = [&]()
	{
		{
			std::lock_guard<std::mutex> lck(obsMtx);
			lastObs.reset();
		}
		for (int i = 0; i < 50 && !world.simulator_must_close(); i++)
		{
			world.run_simulation(0.01);
			std::lock_guard<std::mutex> lck(obsMtx);
			if (lastObs)
			{
				return lastObs;
			}
		}
		std::lock_guard<std::mutex> lck(obsMtx);
		return lastObs;
	};

	auto obs = runAndGetImage();
	if (!obs || glFailed || world.simulator_must_close())
	{
		std::cout << "[SKIPPED] Camera test: no OpenGL rendering available.\n";
		stop = true;
		th.join();
		return;
	}
	EXPECT_TRUE(!centerIsRed(*obs));

	// Spawn a decal right under the camera: must be seen in the next frame
	world.runtimeObjects().spawn(makeDecal("decal", 3.0, 0.0));
	obs = runAndGetImage();
	EXPECT_TRUE(obs && centerIsRed(*obs));

	// Move it away:
	world.runtimeObjects().setPose("decal", {10.0, 0.0, 0.005, 0, 0, 0});
	obs = runAndGetImage();
	EXPECT_TRUE(obs && !centerIsRed(*obs));

	// Back, then remove:
	world.runtimeObjects().setPose("decal", {3.0, 0.0, 0.005, 0, 0, 0});
	obs = runAndGetImage();
	EXPECT_TRUE(obs && centerIsRed(*obs));
	world.runtimeObjects().remove({"decal"});
	obs = runAndGetImage();
	EXPECT_TRUE(obs && !centerIsRed(*obs));

	// GUI-only overlays are not seen by sensors:
	auto overlay = makeDecal("overlay", 3.0, 0.0);
	overlay.visible_to_sensors = false;
	world.runtimeObjects().spawn(overlay);
	obs = runAndGetImage();
	EXPECT_TRUE(obs && !centerIsRed(*obs));

	stop = true;
	th.join();
	std::cout << "Camera test done.\n";
}

}  // namespace

int main()
{
	try
	{
		{
			mvsim::World world;
			world.headless(true);
			world.load_from_XML(worldXml(false), ".");
			test_container(world);
		}
		{
			mvsim::World world;
			world.headless(true);
			world.load_from_XML(worldXml(true), ".");
			test_camera(world);
		}
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
