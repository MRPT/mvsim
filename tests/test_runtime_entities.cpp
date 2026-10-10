/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Entities (vehicles, blocks, elements) inserted into and removed from a
// running simulation from world XML: physics, names, threading, and
// visibility to a camera sensor (off-screen rendering; skipped if no OpenGL
// context can be created).

#include <mrpt/obs/CObservationImage.h>
#include <mvsim/VehicleBase.h>
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
const char* kVehicleDynamics = R"(
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
      <controller class="twist_pid" />
    </dynamics>)";

std::string vehicleXml(const std::string& name, double x, double y, const std::string& extra = {})
{
	return "<vehicle name=\"" + name + "\">" + kVehicleDynamics + "<init_pose>" +
		   std::to_string(x) + " " + std::to_string(y) + " 0</init_pose>" + extra +
		   "</vehicle>";
}

// A static red box of 1x1 m:
std::string blockXml(const std::string& name, double x, double y)
{
	return "<block name=\"" + name +
		   "\"><static>true</static><zmin>0</zmin><zmax>1</zmax><color>#ff0000</color>"
		   "<shape><pt>-0.5 -0.5</pt><pt>0.5 -0.5</pt><pt>0.5 0.5</pt><pt>-0.5 0.5</pt>"
		   "</shape><init_pose>" +
		   std::to_string(x) + " " + std::to_string(y) + " 0</init_pose></block>";
}

std::string worldXml(const std::string& extra = {})
{
	return R"(<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <element class="horizontal_plane">
    <cutout>-30 -30 30 30</cutout>
    <color>#808080</color>
  </element>)" +
		   vehicleXml("r1", 0, 0) + extra + "</mvsim_world>";
}

void drive(mvsim::World& w, const std::string& veh, double vx, double seconds)
{
	auto& v = *w.getListOfVehicles().find(veh)->second;
	for (int i = 0; i < static_cast<int>(seconds / 0.01); i++)
	{
		v.getControllerInterface()->setTwistCommand({vx, 0, 0});
		w.run_simulation(0.01);
	}
}

double poseX(mvsim::World& w, const std::string& name)
{
	return w.getListOfVehicles().find(name)->second->getPose().x;
}

void testBlockInsertRemove()
{
	mvsim::World w;
	w.headless(true);
	w.load_from_XML(worldXml());

	// A wall 2 m ahead stops the robot:
	const auto names = w.insertEntitiesFromXML(blockXml("box", 2.0, 0.0));
	EXPECT_TRUE(names.size() == 1 && names.at(0) == "box");
	EXPECT_TRUE(w.getListOfBlocks().count("box") == 1);

	drive(w, "r1", 1.0, 4.0);
	EXPECT_LT(poseX(w, "r1"), 1.5);

	// Once removed, it moves on:
	EXPECT_TRUE(w.removeEntity("box"));
	EXPECT_TRUE(w.getListOfBlocks().count("box") == 0);
	EXPECT_FALSE(w.removeEntity("box"));
	drive(w, "r1", 1.0, 3.0);
	EXPECT_GT(poseX(w, "r1"), 3.0);
}

void testVehicleInsertRemove()
{
	mvsim::World w;
	w.headless(true);
	w.load_from_XML(worldXml());

	std::vector<std::string> events;
	w.registerCallbackOnEntityChange(
		[&](const mvsim::World::EntityChange& c)
		{ events.push_back((c.added ? "+" : "-") + c.name); });

	// Several entities in one call, with a <mvsim_world> root:
	const auto names = w.insertEntitiesFromXML(
		"<mvsim_world version=\"1.0\">" + vehicleXml("r2", 0, 5) + vehicleXml("r3", 0, -5) +
		"</mvsim_world>");
	EXPECT_TRUE(names.size() == 2);
	EXPECT_TRUE(w.getListOfVehicles().size() == 3);
	EXPECT_TRUE(events.size() == 2);

	drive(w, "r2", 1.0, 2.0);
	EXPECT_GT(poseX(w, "r2"), 1.5);

	// Indices are unique, even after removals:
	const auto idx3 = w.getListOfVehicles().find("r3")->second->getVehicleIndex();
	EXPECT_TRUE(w.removeEntity("r2"));
	w.insertEntitiesFromXML(vehicleXml("r4", 5, 5));
	const auto idx4 = w.getListOfVehicles().find("r4")->second->getVehicleIndex();
	EXPECT_TRUE(idx4 != idx3);

	// Removed entities vanish from the ground truth:
	drive(w, "r1", 0.5, 0.5);
	bool found = false;
	for (const auto& o : w.getGroundTruthSnapshot().objects)
	{
		found = found || o.name == "r2";
	}
	EXPECT_FALSE(found);
	EXPECT_TRUE(events.size() == 4 && events.at(2) == "-r2" && events.at(3) == "+r4");
}

void testErrors()
{
	mvsim::World w;
	w.headless(true);
	w.load_from_XML(worldXml(blockXml("b1", 5, 5)));
	const auto nBlocks = w.getListOfBlocks().size();

	// Duplicated name: nothing is inserted, not even the valid one before it.
	bool thrown = false;
	try
	{
		w.insertEntitiesFromXML(blockXml("ok", 8, 8) + blockXml("b1", 9, 9));
	}
	catch (const std::exception&)
	{
		thrown = true;
	}
	EXPECT_TRUE(thrown);
	EXPECT_TRUE(w.getListOfBlocks().size() == nBlocks);

	thrown = false;
	try
	{
		w.insertEntitiesFromXML("<block name='x'><unclosed></block>");
	}
	catch (const std::exception&)
	{
		thrown = true;
	}
	EXPECT_TRUE(thrown);

	// Unnamed elements get a name, so they can be removed:
	const auto names = w.insertEntitiesFromXML(
		"<element class='ground_grid'><interval>1</interval></element>");
	EXPECT_TRUE(names.size() == 1 && names.at(0).rfind("element_", 0) == 0);
	EXPECT_TRUE(w.removeEntity(names.at(0)));

	drive(w, "r1", 0.5, 0.5);  // still alive
}

void testFromOtherThread()
{
	mvsim::World w;
	w.headless(true);
	w.load_from_XML(worldXml());

	std::atomic_bool done = false;
	std::vector<std::string> names;
	std::thread th(
		[&]()
		{
			auto fut = w.runInSimulationThread([&]()
											   { names = w.insertEntitiesFromXML(
													 blockXml("fromThread", 4, 4)); });
			fut.get();
			done = true;
		});
	for (int i = 0; i < 1000 && !done; i++)
	{
		w.run_simulation(0.01);
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
	th.join();
	EXPECT_TRUE(done && names.size() == 1 && w.getListOfBlocks().count("fromThread") == 1);
}

// The red box under a camera that looks down:
bool centerIsRed(const mrpt::obs::CObservationImage& o)
{
	const auto& img = o.image;
	const auto x = static_cast<int>(img.getWidth() / 2);
	const auto y = static_cast<int>(img.getHeight() / 2);
	const auto r = img.at<uint8_t>(x, y, 0);
	const auto g = img.at<uint8_t>(x, y, 1);
	const auto b = img.at<uint8_t>(x, y, 2);
	std::cout << "Center pixel RGB: " << int(r) << " " << int(g) << " " << int(b) << "\n";
	return r > g + 100 && r > b + 100;
}

void testCameraVisibility()
{
	mvsim::World w;
	w.headless(true);
	w.load_from_XML(worldXml());

	std::mutex obsMtx;
	mrpt::obs::CObservationImage::Ptr lastObs;
	w.registerCallbackOnObservation(
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
				while (!stop && !w.simulator_must_close())
				{
					w.internalGraphicsLoopTasksForSimulation();
					std::this_thread::sleep_for(std::chrono::milliseconds(2));
				}
				w.internalFreeOpenGLResourcesForSimulation();
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
		for (int i = 0; i < 50 && !w.simulator_must_close(); i++)
		{
			w.run_simulation(0.01);
			std::lock_guard<std::mutex> lck(obsMtx);
			if (lastObs)
			{
				return lastObs;
			}
		}
		std::lock_guard<std::mutex> lck(obsMtx);
		return lastObs;
	};

	// A robot with a camera 3 m ahead of it, 2 m high, looking down, inserted
	// at runtime:
	w.insertEntitiesFromXML(vehicleXml("camBot", -10, 0, R"(
    <sensor class="camera" name="cam">
      <pose_3d>3.0 0.0 2.0 -90.0 0.0 180.0</pose_3d>
      <sensor_period>0.05</sensor_period>
      <ncols>64</ncols> <nrows>48</nrows>
      <cx>32</cx> <cy>24</cy> <fx>40</fx> <fy>40</fy>
      <clip_min>0.01</clip_min> <clip_max>100</clip_max>
      <preview_win_visible>false</preview_win_visible>
    </sensor>)"));

	auto obs = runAndGetImage();
	if (!obs || glFailed || w.simulator_must_close())
	{
		std::cout << "[SKIPPED] Camera test: no OpenGL rendering available.\n";
		stop = true;
		th.join();
		return;
	}
	EXPECT_TRUE(!centerIsRed(*obs));

	w.insertEntitiesFromXML(blockXml("redBox", -7.0, 0.0));
	obs = runAndGetImage();
	EXPECT_TRUE(obs && centerIsRed(*obs));

	w.removeEntity("redBox");
	obs = runAndGetImage();
	EXPECT_TRUE(obs && !centerIsRed(*obs));

	// Removing the robot with the camera, while rendering: no more images
	w.removeEntity("camBot");
	runAndGetImage();
	obs = runAndGetImage();
	EXPECT_TRUE(!obs);

	stop = true;
	th.join();
	EXPECT_FALSE(glFailed.load());
}
}  // namespace

int main()
{
	try
	{
		testBlockInsertRemove();
		testVehicleInsertRemove();
		testErrors();
		testFromOtherThread();
		testCameraVisibility();
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
