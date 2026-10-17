/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// The <include> and <if> tags must work at any depth inside the instance
// definitions of vehicles and blocks, exactly as they do in their class
// definitions.

#include <mrpt/system/filesystem.h>
#include <mvsim/Sensors/SensorBase.h>
#include <mvsim/World.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "test_utils.h"

int g_failures = 0;

namespace
{
std::string testPath(const std::string& relPath)
{
	return mrpt::system::pathJoin({MVSIM_TEST_DIR, relPath});
}

std::string worldXml()
{
	const std::string imuFile = testPath("../definitions/imu.sensor.xml");
	const std::string robotFile = testPath("../definitions/small_robot.vehicle.xml");
	const std::string shapeFile = testPath("xml_if_include_block_shape.xml");

	// Note that sensor_publish=true makes the included file use the variables
	// PARENT_NAME and NAME, which must be kept until the sensor is built.
	const std::string imu = R"(<include file=")" + imuFile + R"(" sensor_name=")";

	std::string s = R"(
<mvsim_world version="1.0">
  <simul_timestep>0.005</simul_timestep>
  <variable name="YES" value="1"/>
  <variable name="NO" value="0"/>
  <include file=")" +
					robotFile + R"("/>

  <!-- Plain include (regression) -->
  <vehicle name="v_plain" class="small_robot">
    <init_pose>0 0 0</init_pose>
    )" + imu + R"(plain" sensor_publish="true"/>
  </vehicle>

  <!-- <if> around an <include>, true condition -->
  <vehicle name="v_if_true" class="small_robot">
    <init_pose>0 0 0</init_pose>
    <if condition="${YES}">
      )" + imu + R"(if_true" sensor_publish="true"/>
    </if>
  </vehicle>

  <!-- <if> around an <include>, false condition -->
  <vehicle name="v_if_false" class="small_robot">
    <init_pose>0 0 0</init_pose>
    <if condition="${NO}">
      )" + imu + R"(if_false"/>
    </if>
  </vehicle>

  <!-- Nested <if> tags, mixing includes and inline sensors -->
  <vehicle name="v_nested" class="small_robot">
    <init_pose>0 0 0</init_pose>
    )" + imu + R"(top"/>
    <if condition="${YES}">
      )" + imu + R"(a"/>
      <sensor class="imu" name="inline_a"/>
      <if condition="${NO}">
        )" + imu + R"(never"/>
      </if>
      <if condition="${YES}">
        )" + imu + R"(b" sensor_publish="true"/>
      </if>
    </if>
    <sensor class="imu" name="inline_top"/>
  </vehicle>

  <!-- Block instances: <if> and <include> inside them -->
  <block:class name="box">
    <static>true</static>
    <zmin>0</zmin> <zmax>1</zmax>
  </block:class>
  <block class="box" name="b_plain"> <init_pose>0 0 0</init_pose> </block>
  <block class="box" name="b_if_true">
    <init_pose>0 0 0</init_pose>
    <if condition="${YES}">
      <include file=")" +
					shapeFile + R"(" half_size="2.0"/>
    </if>
  </block>
  <block class="box" name="b_if_false">
    <init_pose>0 0 0</init_pose>
    <if condition="${NO}">
      <include file=")" +
					shapeFile + R"(" half_size="2.0"/>
    </if>
  </block>
</mvsim_world>
)";
	return s;
}

std::vector<std::string> sensorNames(const mvsim::World& world, const std::string& vehicleName)
{
	std::vector<std::string> names;
	const auto& vehicles = world.getListOfVehicles();
	const auto it = vehicles.find(vehicleName);
	EXPECT_TRUE(it != vehicles.end());
	if (it == vehicles.end())
	{
		return names;
	}
	for (const auto& sensor : it->second->getSensors())
	{
		names.push_back(sensor->getName());
	}
	std::sort(names.begin(), names.end());
	return names;
}

void expectSensors(
	const mvsim::World& world, const std::string& vehicleName, std::vector<std::string> expected)
{
	std::sort(expected.begin(), expected.end());
	const auto got = sensorNames(world, vehicleName);
	if (got != expected)
	{
		std::fprintf(
			stderr, "FAIL vehicle '%s': got %zu sensors, expected %zu\n", vehicleName.c_str(),
			got.size(), expected.size());
		for (const auto& n : got)
		{
			std::fprintf(stderr, "  got: %s\n", n.c_str());
		}
		++g_failures;
	}
}

double blockWidth(const mvsim::World& world, const std::string& blockName)
{
	const auto& blocks = world.getListOfBlocks();
	const auto it = blocks.find(blockName);
	EXPECT_TRUE(it != blocks.end());
	if (it == blocks.end())
	{
		return 0;
	}
	const auto& poly = it->second->blockShape();
	double minX = 1e9;
	double maxX = -1e9;
	for (const auto& pt : poly)
	{
		minX = std::min(minX, pt.x);
		maxX = std::max(maxX, pt.x);
	}
	return maxX - minX;
}
}  // namespace

int main()
{
	mvsim::World world;
	world.headless(true);
	world.load_from_XML(worldXml());

	expectSensors(world, "v_plain", {"plain"});
	expectSensors(world, "v_if_true", {"if_true"});
	expectSensors(world, "v_if_false", {});
	expectSensors(world, "v_nested", {"top", "a", "inline_a", "b", "inline_top"});

	// Default block shape is 1x1; the included one is 4x4:
	const double defaultWidth = blockWidth(world, "b_plain");
	EXPECT_NEAR(blockWidth(world, "b_if_true"), 4.0, 1e-6);
	EXPECT_NEAR(blockWidth(world, "b_if_false"), defaultWidth, 1e-6);

	if (g_failures)
	{
		std::cerr << g_failures << " failures\n";
		return 1;
	}
	std::cout << "All tests passed\n";
	return 0;
}
