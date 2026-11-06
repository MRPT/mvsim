/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// HumanActor with follow_terrain: its height follows an elevation map ramp
// (with or without a path), while actors without it keep the waypoints height.

#include <mvsim/HumanActor.h>
#include <mvsim/World.h>

#include <cmath>
#include <exception>
#include <iostream>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{
// Ramp along X: z = 0, 1, 2 at x = -10, 0, 10 (z = 1 + x/10).
const char* WORLD_XML = R"(
<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <element class="elevation_map">
    <resolution>10.0</resolution>
    <elevation_data_matrix>[0 0 0; 1 1 1; 2 2 2]</elevation_data_matrix>
    <corner_min_x>-10</corner_min_x>
    <corner_min_y>-10</corner_min_y>
  </element>
  <actor name="walker">
    <follow_terrain>true</follow_terrain>
    <walking_speed>1.0</walking_speed>
    <init_pose>-5 0 0</init_pose>
    <path loop="false">
      <waypoint>-5 0 0</waypoint>
      <waypoint>5 0 0</waypoint>
    </path>
  </actor>
  <actor name="stander">
    <follow_terrain>true</follow_terrain>
    <init_pose>5 -3 0</init_pose>
  </actor>
  <actor name="floater">
    <walking_speed>1.0</walking_speed>
    <init_pose>-5 1 0</init_pose>
    <path loop="false">
      <waypoint>-5 1 0</waypoint>
      <waypoint>5 1 0</waypoint>
    </path>
  </actor>
</mvsim_world>
)";

// Steep ramp (z = 10 + x): the walking speed must not depend on the height
// difference between the ground and the waypoints.
const char* STEEP_WORLD_XML = R"(
<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <element class="elevation_map">
    <resolution>10.0</resolution>
    <elevation_data_matrix>[0 0 0; 10 10 10; 20 20 20]</elevation_data_matrix>
    <corner_min_x>-10</corner_min_x>
    <corner_min_y>-10</corner_min_y>
  </element>
  <actor name="climber">
    <follow_terrain>true</follow_terrain>
    <walking_speed>1.0</walking_speed>
    <init_pose>-5 0 0</init_pose>
    <path loop="false">
      <waypoint>-5 0 0</waypoint>
      <waypoint>5 0 0</waypoint>
      <waypoint>-5 0 180</waypoint>
    </path>
  </actor>
</mvsim_world>
)";

mvsim::HumanActor::Ptr actor(mvsim::World& world, const std::string& name)
{
	const auto it = world.getListOfActors().find(name);
	ASSERT_(it != world.getListOfActors().end());
	return it->second;
}
}  // namespace

int main()
{
	try
	{
		mvsim::World world;
		world.headless(true);
		world.load_from_XML(WORLD_XML, ".");

		const auto walker = actor(world, "walker");
		const auto floater = actor(world, "floater");

		// Check the height along the way, as it walks up the ramp:
		for (int step = 0; step < 4; step++)
		{
			world.run_simulation(2.0);
			const auto p = walker->getPose();
			EXPECT_NEAR(p.z, 1.0 + p.x / 10.0, 0.05);
			EXPECT_NEAR(floater->getPose().z, 0.0, 0.05);
		}
		EXPECT_GT(walker->getPose().x, 0.0);
		// Without a path, too:
		EXPECT_NEAR(actor(world, "stander")->getPose().z, 1.5, 0.05);
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
		return 1;
	}

	try
	{
		mvsim::World world;
		world.headless(true);
		world.load_from_XML(STEEP_WORLD_XML, ".");
		const auto climber = actor(world, "climber");

		// 10 m up the ramp, then 5 m back down (planar distance), at 1 m/s:
		world.run_simulation(15.0);
		const auto p = climber->getPose();
		EXPECT_NEAR(p.x, 0.0, 0.5);
		EXPECT_NEAR(p.z, 10.0 + p.x, 0.1);
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
		return 1;
	}

	if (g_failures != 0)
	{
		std::cerr << g_failures << " failure(s)\n";
		return 1;
	}
	std::cout << "All tests passed.\n";
	return 0;
}
