/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// World::getPerformanceStats(): measuring windows, and reset by clear_all().

#include <mrpt/system/datetime.h>
#include <mvsim/World.h>

#include <iostream>

#include "test_utils.h"

int g_failures = 0;

namespace
{
const char* kWorldXml = R"(
<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
      <controller class="twist_pid" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
  </vehicle>
</mvsim_world>
)";
}  // namespace

int main()
{
	try
	{
		mvsim::World w;
		w.headless(true);
		w.load_from_XML(kWorldXml);

		// No window completed yet:
		EXPECT_TRUE(w.getPerformanceStats().steps == 0);

		// 2.5 s: one 2 s window, which includes the first step:
		for (int i = 0; i < 250; i++)
		{
			w.run_simulation(0.01);
		}
		const auto st = w.getPerformanceStats();
		EXPECT_TRUE(st.steps == 200);
		EXPECT_NEAR(st.window_simul_time, 2.0, 1e-6);
		EXPECT_GT(st.window_wall_time, 0);
		EXPECT_GT(st.window_end_wall_time, 0);
		EXPECT_LT(mrpt::Clock::nowDouble() - st.window_end_wall_time, 60.0);

		// Clearing the world (as done when loading one) resets them:
		w.clear_all();
		EXPECT_TRUE(w.getPerformanceStats().steps == 0);
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
