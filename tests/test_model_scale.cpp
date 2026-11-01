/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Per-axis model scale (model_scale / visual_scale as "SX SY SZ") and model
// offsets must be reflected in the collision footprint of each block, even
// when many blocks share the same model file.

#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mvsim/CollisionShapeCache.h>
#include <mvsim/World.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{
struct BBox2D
{
	double minX = std::numeric_limits<double>::max();
	double maxX = -std::numeric_limits<double>::max();
	double minY = std::numeric_limits<double>::max();
	double maxY = -std::numeric_limits<double>::max();

	double width() const { return maxX - minX; }
	double height() const { return maxY - minY; }
};

BBox2D footprintOf(const mvsim::World& world, const std::string& blockName)
{
	BBox2D bb;
	const auto& blocks = world.getListOfBlocks();
	const auto it = blocks.find(blockName);
	EXPECT_TRUE(it != blocks.end());
	if (it == blocks.end())
	{
		return bb;
	}
	const auto& shape = it->second->collisionShape();
	EXPECT_TRUE(shape.has_value());
	if (!shape)
	{
		return bb;
	}
	for (const auto& pt : shape->getContour())
	{
		bb.minX = std::min(bb.minX, pt.x);
		bb.maxX = std::max(bb.maxX, pt.x);
		bb.minY = std::min(bb.minY, pt.y);
		bb.maxY = std::max(bb.maxY, pt.y);
	}
	return bb;
}

// A model made of separate pieces (like the leaves of a plant) must get a
// footprint covering all of them, not only one.
void testSparseModelFootprint()
{
	auto glTris = mrpt::viz::CSetOfTriangles::Create();
	for (const double x0 : {0.0, 1.0})
	{
		mrpt::viz::TTriangle t(
			mrpt::math::TPoint3Df(x0 - 0.1, -0.1, 0), mrpt::math::TPoint3Df(x0 + 0.1, -0.1, 0),
			mrpt::math::TPoint3Df(x0, 0.1, 0.5));
		glTris->insertTriangle(t);
	}

	const auto shape =
		mvsim::CollisionShapeCache::Instance().get(*glTris, -1.0f, 1.0f, {}, {1.0f, 1.0f, 1.0f});

	BBox2D bb;
	for (const auto& pt : shape.getContour())
	{
		bb.minX = std::min(bb.minX, pt.x);
		bb.maxX = std::max(bb.maxX, pt.x);
	}
	EXPECT_NEAR(bb.minX, -0.1, 0.05);
	EXPECT_NEAR(bb.maxX, 1.1, 0.05);
}

std::string worldXml()
{
	const std::string model =
		mrpt::system::pathJoin({MVSIM_TEST_DIR, "../models/simple_camera.dae"});

	return R"(
<mvsim_world version="1.0">
  <simul_timestep>0.005</simul_timestep>
  <block:class name="cam">
    <static>true</static>
    <shape_from_visual/>
    <visual> <model_uri>)" +
		   model + R"(</model_uri> </visual>
  </block:class>
  <block:class name="cam_wide">
    <static>true</static>
    <shape_from_visual/>
    <visual> <model_uri>)" +
		   model + R"(</model_uri> <model_scale>2 1 1</model_scale> </visual>
  </block:class>
  <block:class name="cam_offset">
    <static>true</static>
    <shape_from_visual/>
    <visual> <model_uri>)" +
		   model + R"(</model_uri> <model_offset_x>1.0</model_offset_x> </visual>
  </block:class>

  <block class="cam" name="ref"> <init_pose>0 0 0</init_pose> </block>
  <block class="cam_wide" name="wide"> <init_pose>5 0 0</init_pose> </block>
  <block class="cam" name="tall_y"> <init_pose>10 0 0</init_pose> <visual_scale>1 3 1</visual_scale> </block>
  <block class="cam" name="half"> <init_pose>15 0 0</init_pose> <visual_scale>0.5</visual_scale> </block>
  <block class="cam_offset" name="offset"> <init_pose>20 0 0</init_pose> </block>
</mvsim_world>
)";
}
}  // namespace

int main()
{
	testSparseModelFootprint();

	mvsim::World world;
	world.headless(true);
	world.load_from_XML(worldXml());

	const auto ref = footprintOf(world, "ref");
	EXPECT_TRUE(ref.width() > 1e-3 && ref.height() > 1e-3);

	// Collision footprints are simplified polygons: compare with a tolerance.
	const double tol = 0.05 * std::max(ref.width(), ref.height());

	// Class model_scale "2 1 1": twice as wide in x only.
	const auto wide = footprintOf(world, "wide");
	EXPECT_NEAR(wide.width(), 2 * ref.width(), 2 * tol);
	EXPECT_NEAR(wide.height(), ref.height(), tol);

	// Per-instance visual_scale "1 3 1" overrides the class scale.
	const auto tallY = footprintOf(world, "tall_y");
	EXPECT_NEAR(tallY.width(), ref.width(), tol);
	EXPECT_NEAR(tallY.height(), 3 * ref.height(), 3 * tol);

	// A single value keeps working as a uniform scale.
	const auto half = footprintOf(world, "half");
	EXPECT_NEAR(half.width(), 0.5 * ref.width(), tol);
	EXPECT_NEAR(half.height(), 0.5 * ref.height(), tol);

	// Model offsets move the footprint, in the block local frame.
	const auto offset = footprintOf(world, "offset");
	EXPECT_NEAR(offset.minX, ref.minX + 1.0, tol);
	EXPECT_NEAR(offset.maxX, ref.maxX + 1.0, tol);
	EXPECT_NEAR(offset.width(), ref.width(), tol);

	if (g_failures)
	{
		std::cerr << g_failures << " failures\n";
		return 1;
	}
	std::cout << "All tests passed\n";
	return 0;
}
