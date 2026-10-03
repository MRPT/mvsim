/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Tests of World elevation queries (horizontal planes, blocks), which use a
// 2D spatial index of world elements and blocks.

#include <mrpt/core/bits_math.h>
#include <mvsim/World.h>

#include <iostream>
#include <iterator>

#include "test_utils.h"

int g_failures = 0;

namespace
{
const char* WORLD_XML = R"(
<mvsim_world version="1.0">
  <element class="horizontal_plane">
    <x_min>-10</x_min> <x_max>10</x_max> <y_min>-10</y_min> <y_max>10</y_max> <z>0</z>
  </element>
  <element class="horizontal_plane">
    <x_min>2</x_min> <x_max>4</x_max> <y_min>2</y_min> <y_max>4</y_max> <z>1</z>
  </element>
  <element class="horizontal_plane">
    <init_pose>20 0 45</init_pose>
    <x_min>-1</x_min> <x_max>1</x_max> <y_min>-1</y_min> <y_max>1</y_max> <z>0.5</z>
  </element>
  <element class="horizontal_plane">
    <x_min>2000</x_min> <x_max>4000</x_max> <y_min>2000</y_min> <y_max>4000</y_max> <z>-1</z>
  </element>
  <block>
    <static>true</static>
    <init_pose>500 500 0</init_pose>
    <shape> <pt>-150 -150</pt> <pt>150 -150</pt> <pt>150 150</pt> <pt>-150 150</pt> </shape>
    <zmin>0</zmin> <zmax>0.2</zmax>
  </block>
  <block>
    <static>true</static>
    <init_pose>0 -20 0</init_pose>
    <shape> <pt>-6 -1</pt> <pt>6 -1</pt> <pt>6 1</pt> <pt>-6 1</pt> </shape>
    <zmin>0</zmin> <zmax>0.3</zmax>
  </block>
</mvsim_world>
)";

float elev(const mvsim::World& w, double x, double y, double z = 5.0)
{
	return w.getHighestElevationUnder(
		mrpt::math::TPoint3Df(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)));
}
}  // namespace

int main()
{
	mvsim::World world;
	world.headless(true);
	world.load_from_XML(WORLD_XML);

	// Floor, platform, and the highest one under the query point only:
	EXPECT_NEAR(elev(world, 0, 0), 0.0, 1e-4);
	EXPECT_NEAR(elev(world, 3, 3), 1.0, 1e-4);
	EXPECT_NEAR(elev(world, 3, 3, 0.5), 0.0, 1e-4);

	// A rotated plane (a square rotated 45 deg):
	EXPECT_NEAR(elev(world, 21.3, 0), 0.5, 1e-4);
	EXPECT_NEAR(elev(world, 21.0, 1.0), 0.0, 1e-4);

	// Nothing there:
	EXPECT_NEAR(elev(world, 50, 50), 0.0, 1e-4);

	// A very large plane (too large to be indexed by cell):
	EXPECT_NEAR(elev(world, 3000, 3000), -1.0, 1e-4);

	// A very large block (too large to be indexed by cell):
	EXPECT_NEAR(elev(world, 500, 500), 0.2, 1e-4);
	EXPECT_NEAR(elev(world, 620, 380), 0.2, 1e-4);

	// The middle of a large block, far from its vertices:
	EXPECT_NEAR(elev(world, 0, -20), 0.3, 1e-4);
	EXPECT_NEAR(elev(world, 5.5, -20.5), 0.3, 1e-4);

	// Moving a plane (the 2nd one, the platform) updates the queries:
	const auto& elements = world.getListOfWorldElements();
	(*std::next(elements.begin()))->setPose(mrpt::math::TPose3D(-6, 0, 0, 0, 0, 0));
	EXPECT_NEAR(elev(world, 3, 3), 0.0, 1e-4);
	EXPECT_NEAR(elev(world, -3, 3), 1.0, 1e-4);

	if (g_failures)
	{
		std::cerr << g_failures << " failures\n";
		return 1;
	}
	std::cout << "All tests passed\n";
	return 0;
}
