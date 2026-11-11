/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Tests of <include> of a complete world file (with its <mvsim_world> root)
// from another world, with paths relative to the included file.

#include <mrpt/system/filesystem.h>
#include <mvsim/World.h>

#include <fstream>
#include <iostream>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{
void writeFile(const std::string& path, const std::string& text)
{
	std::ofstream f(path);
	f << text;
}
}  // namespace

int main()
{
	const std::string dir = mrpt::system::getTempFileName() + "_world_include";
	const std::string sceneDir = dir + "/scene";
	mrpt::system::createDirectory(dir);
	mrpt::system::createDirectory(sceneDir);

	// A scene world, loadable by itself, with a file relative to it:
	writeFile(sceneDir + "/box.block.xml", R"(
<block:class name="box">
  <static>true</static>
  <geometry type="box" lx="1" ly="1" lz="1"/>
</block:class>
)");
	writeFile(sceneDir + "/scene.world.xml", R"(
<mvsim_world version="1.0">
  <georeference>
    <latitude>36.8</latitude>
    <longitude>-2.7</longitude>
    <height>20</height>
  </georeference>
  <include file="box.block.xml"/>
  <block name="box1" class="box">
    <init_pose>5 0 0</init_pose>
  </block>
</mvsim_world>
)");

	// The main world adds a vehicle to that scene:
	writeFile(dir + "/main.world.xml", R"(
<mvsim_world version="1.0">
  <include file="scene/scene.world.xml"/>
  <vehicle:class name="small_robot">
    <dynamics class="differential">
      <l_wheel pos="0.0 0.2" mass="2.0" width="0.05" diameter="0.2"/>
      <r_wheel pos="0.0 -0.2" mass="2.0" width="0.05" diameter="0.2"/>
      <chassis mass="10.0" zmin="0.05" zmax="0.3"/>
    </dynamics>
  </vehicle:class>
  <vehicle name="veh" class="small_robot">
    <init_pose>1 2 0</init_pose>
  </vehicle>
</mvsim_world>
)");

	mvsim::World w;
	w.headless(true);
	w.load_from_XML_file(dir + "/main.world.xml");

	EXPECT_TRUE(w.getListOfBlocks().count("box1") == 1);
	EXPECT_TRUE(w.getListOfVehicles().count("veh") == 1);
	EXPECT_NEAR(w.georeferenceOptions().georefCoord.lat.decimal_value, 36.8, 1e-9);

	mrpt::system::deleteFilesInDirectory(dir, true /*delete dir*/);

	if (g_failures)
	{
		std::cerr << "test_world_include: " << g_failures << " failure(s)\n";
		return 1;
	}
	std::cout << "test_world_include: all passed\n";
	return 0;
}
