/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Static blocks are not moved in Box2D at each time step, only when their pose
// is set: check that setting it still moves their physical body.

#include <box2d/b2_body.h>
#include <mrpt/core/bits_math.h>
#include <mvsim/World.h>

#include <iostream>

#include "test_utils.h"

int g_failures = 0;

namespace
{
const char* WORLD_XML = R"(
<mvsim_world version="1.0">
  <simul_timestep>0.005</simul_timestep>
  <block name="fixed">
    <static>true</static>
    <init_pose>2 0 0</init_pose>
    <shape> <pt>-0.5 -0.5</pt> <pt>0.5 -0.5</pt> <pt>0.5 0.5</pt> <pt>-0.5 0.5</pt> </shape>
    <zmin>0</zmin> <zmax>1</zmax>
  </block>
</mvsim_world>
)";

void expectBodyAt(const mvsim::Block& b, double x, double y, double yawDeg)
{
	const b2Body* body = b.b2d_body();
	EXPECT_TRUE(body != nullptr);
	if (!body)
	{
		return;
	}
	EXPECT_NEAR(body->GetPosition().x, x, 1e-4);
	EXPECT_NEAR(body->GetPosition().y, y, 1e-4);
	EXPECT_NEAR(body->GetAngle(), mrpt::DEG2RAD(yawDeg), 1e-4);

	const auto p = b.getPose();
	EXPECT_NEAR(p.x, x, 1e-4);
	EXPECT_NEAR(p.y, y, 1e-4);
	EXPECT_NEAR(p.yaw, mrpt::DEG2RAD(yawDeg), 1e-4);
}
}  // namespace

int main()
{
	mvsim::World world;
	world.headless(true);
	world.load_from_XML(WORLD_XML);

	const auto& blocks = world.getListOfBlocks();
	EXPECT_TRUE(blocks.size() == 1);
	const auto& block = *blocks.begin()->second;
	EXPECT_TRUE(block.isStatic());

	world.run_simulation(0.05);
	expectBodyAt(block, 2, 0, 0);

	// Moved by the user (e.g. a service call), then left there:
	block.setPose(mrpt::math::TPose3D(-3, 4, 0, mrpt::DEG2RAD(30.0), 0, 0));
	world.run_simulation(0.05);
	expectBodyAt(block, -3, 4, 30);
	world.run_simulation(0.05);
	expectBodyAt(block, -3, 4, 30);

	// ...and moved again:
	block.setPose(mrpt::math::TPose3D(1, -1, 0, mrpt::DEG2RAD(-90.0), 0, 0));
	world.run_simulation(0.05);
	expectBodyAt(block, 1, -1, -90);

	if (g_failures)
	{
		std::cerr << g_failures << " failures\n";
		return 1;
	}
	std::cout << "All tests passed\n";
	return 0;
}
