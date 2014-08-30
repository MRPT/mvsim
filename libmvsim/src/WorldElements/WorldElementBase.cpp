/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mvsim/WorldElements/OccupancyGridMap.h>
#include <mvsim/WorldElements/GroundGrid.h>
#include <mvsim/WorldElements/ElevationMap.h>

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include <rapidxml_print.hpp>
#include <mrpt/utils/utils_defs.h>  // mrpt::format()


#include <sstream>      // std::stringstream
#include <map>
#include <string>

using namespace mvsim;

TClassFactory_worldElements mvsim::classFactory_worldElements;

// Explicit registration calls seem to be one (the unique?) way to assure registration takes place:
void register_all_world_elements()
{
	static bool done = false;
	if (done) return; else done=true;

	REGISTER_WORLD_ELEMENT("ground_grid",GroundGrid)		
	REGISTER_WORLD_ELEMENT("occupancy_grid",OccupancyGridMap)
	REGISTER_WORLD_ELEMENT("elevation_map",ElevationMap)
}


WorldElementBase* WorldElementBase::factory(World* parent, const rapidxml::xml_node<char> *root, const char * class_name)
{
	register_all_world_elements();

	using namespace std;
	using namespace rapidxml;

	string sName;
	if (!root) {
		sName = string(class_name);
	}
	else 
	{
		if (0!=strncmp(root->name(),"world:",strlen("world:"))) throw runtime_error(mrpt::format("[WorldElementBase::factory] XML root element is '%s' ('world:*' expected)",root->name()));

		// Get the world:* final part as the name of the class (e.g. "world:gridmap"  -> "gridmap"):
		sName  = string(root->name()).substr(strlen("world:"));
	}
	WorldElementBase* we = classFactory_worldElements.create(sName, parent, root);

	if (!we) throw runtime_error(mrpt::format("[WorldElementBase::factory] Unknown world element type '%s'",root->name()));
	
	return we;
}
