/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/format.h>
#include <mvsim/WorldElements/ElevationMap.h>
#include <mvsim/WorldElements/GroundGrid.h>
#include <mvsim/WorldElements/HorizontalPlane.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>
#include <mvsim/WorldElements/PointCloud.h>

#include <map>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>
#include <sstream>	// std::stringstream
#include <string>

using namespace mvsim;

TClassFactory_worldElements mvsim::classFactory_worldElements;

// Explicit registration calls seem to be one (the unique?) way to assure
// registration takes place:
void register_all_world_elements()
{
	static bool done = false;
	if (done)
		return;
	else
		done = true;

	REGISTER_WORLD_ELEMENT("ground_grid", GroundGrid)
	REGISTER_WORLD_ELEMENT("occupancy_grid", OccupancyGridMap)
	REGISTER_WORLD_ELEMENT("elevation_map", ElevationMap)
	REGISTER_WORLD_ELEMENT("horizontal_plane", HorizontalPlane)
	REGISTER_WORLD_ELEMENT("pointcloud", PointCloud)
}

WorldElementBase::Ptr WorldElementBase::factory(
	World* parent, const rapidxml::xml_node<char>* root, const char* class_name)
{
	register_all_world_elements();

	using namespace std;
	using namespace rapidxml;

	string sName;
	if (!root)
	{
		sName = string(class_name);
	}
	else
	{
		if (0 != strcmp(root->name(), "element"))
			throw runtime_error(mrpt::format(
				"[WorldElementBase::factory] XML root element is '%s' "
				"('<element>' expected)",
				root->name()));

		// Get class name:
		const xml_attribute<>* attrib_class = root->first_attribute("class");
		if (!attrib_class || !attrib_class->value())
			throw runtime_error(
				"[WorldElementBase::factory] Missing mandatory attribute "
				"'class' in node <element>");

		sName = string(attrib_class->value());
	}
	auto we = classFactory_worldElements.create(sName, parent, root);

	if (!we)
		throw runtime_error(mrpt::format(
			"[WorldElementBase::factory] Unknown world element type '%s'",
			root ? root->name() : "(root=nullptr)"));

	return we;
}
