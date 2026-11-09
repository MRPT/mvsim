/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Tests of <random_seed> (reproducible random expressions and per-object
// noise seeds) and of the decimation of the vehicle logs with <log_period>.

#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <iostream>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{
std::string worldXml(const std::string& seed)
{
	return R"(
<mvsim_world version="1.0">
  <simul_timestep>0.001</simul_timestep>
  <random_seed>)" +
		   seed + R"(</random_seed>
  <variable name="RND_U" value="$f{rand()}"/>
  <variable name="RND_N" value="$f{randn()}"/>
  <vehicle:class name="small_robot">
    <dynamics class="differential">
      <l_wheel pos="0.0 0.2" mass="2.0" width="0.05" diameter="0.2"/>
      <r_wheel pos="0.0 -0.2" mass="2.0" width="0.05" diameter="0.2"/>
      <chassis mass="10.0" zmin="0.05" zmax="0.3"/>
      <controller class="twist_pid">
        <KP>10</KP> <KI>1</KI> <max_torque>50</max_torque>
      </controller>
    </dynamics>
    <log_period>0.01</log_period>
  </vehicle:class>
  <vehicle name="veh" class="small_robot">
    <init_pose>0 0 0</init_pose>
  </vehicle>
</mvsim_world>
)";
}

std::string var(const mvsim::World& w, const std::string& name)
{
	const auto& vars = w.user_defined_variables();
	const auto it = vars.find(name);
	return it == vars.end() ? std::string() : it->second;
}
}  // namespace

int main()
{
	// Same seed: same random expressions and noise seeds.
	mvsim::World w1;
	w1.headless(true);
	w1.load_from_XML(worldXml("5"));

	mvsim::World w2;
	w2.headless(true);
	w2.load_from_XML(worldXml("5"));

	EXPECT_TRUE(w1.random_seed().has_value());
	EXPECT_TRUE(w1.random_seed().value_or(0) == 5u);
	EXPECT_TRUE(!var(w1, "RND_U").empty());
	EXPECT_TRUE(var(w1, "RND_U") == var(w2, "RND_U"));
	EXPECT_TRUE(var(w1, "RND_N") == var(w2, "RND_N"));
	EXPECT_TRUE(w1.noiseSeedFor("veh/lidar") == w2.noiseSeedFor("veh/lidar"));
	EXPECT_TRUE(w1.noiseSeedFor("veh/lidar") != w1.noiseSeedFor("veh/imu"));

	// Another seed: other values.
	mvsim::World w3;
	w3.headless(true);
	w3.load_from_XML(worldXml("6"));
	EXPECT_TRUE(var(w1, "RND_U") != var(w3, "RND_U"));

	// Negative seed: non-deterministic, no per-object seeds.
	mvsim::World w4;
	w4.headless(true);
	w4.load_from_XML(worldXml("-1"));
	EXPECT_FALSE(w4.random_seed().has_value());
	EXPECT_FALSE(w4.noiseSeedFor("veh/lidar").has_value());

	// <log_period>: one log row every 10 ms of simulated time.
	{
		auto& veh = *w1.getListOfVehicles().begin()->second;
		int rows = 0;
		veh.getLoggerPtr(mvsim::VehicleBase::LOGGER_IDX_POSE)
			->registerOnRowCallback([&rows](const auto&) { rows++; });
		w1.run_simulation(1.0);
		EXPECT_NEAR(rows, 100, 2);
	}

	if (g_failures)
	{
		std::cerr << "test_random_seed: " << g_failures << " failure(s)\n";
		return 1;
	}
	std::cout << "test_random_seed: all passed\n";
	return 0;
}
