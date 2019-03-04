/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#include <mvsim/WorldElements/OccupancyGridMap.h>
#include <mvsim/WorldElements/GroundGrid.h>
#include <mvsim/WorldElements/ElevationMap.h>

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include <rapidxml_print.hpp>

#include <mrpt/version.h>
#if MRPT_VERSION<0x199
#include <mrpt/utils/utils_defs.h>  // mrpt::format()
using namespace mrpt::utils;
#else
#include <mrpt/core/format.h>
using namespace mrpt;
#endif

#include <sstream>  // std::stringstream
#include <map>
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
}

WorldElementBase* WorldElementBase::factory(
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
			throw runtime_error(
				mrpt::format(
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
	WorldElementBase* we =
		classFactory_worldElements.create(sName, parent, root);

	if (!we)
		throw runtime_error(
			mrpt::format(
				"[WorldElementBase::factory] Unknown world element type '%s'",
				root->name()));

	return we;
}
