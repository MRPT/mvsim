/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/WorldElements/OccupancyGridMap.h>

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include <rapidxml_print.hpp>
#include <mrpt/utils/utils_defs.h>  // mrpt::format()


#include <sstream>      // std::stringstream
#include <map>
#include <string>

using namespace mv2dsim;

TClassFactory_worldElements mv2dsim::classFactory_worldElements;

// Explicit registration calls seem to be one (the unique?) way to assure registration takes place:
void register_all_world_elements()
{
	static bool done = false;
	if (done) return; else done=true;

	REGISTER_WORLD_ELEMENT("gridmap",OccupancyGridMap)
}


WorldElementBase* WorldElementBase::factory(World* parent, const rapidxml::xml_node<char> *root)
{
	register_all_world_elements();

	using namespace std;
	using namespace rapidxml;

	if (!root) throw runtime_error("[WorldElementBase::factory] XML node is NULL");
	if (0!=strncmp(root->name(),"world:",strlen("world:"))) throw runtime_error(mrpt::format("[WorldElementBase::factory] XML root element is '%s' ('world:*' expected)",root->name()));

	// Get the world:* final part as the name of the class (e.g. "world:gridmap"  -> "gridmap"):
	const string sName  = string(root->name()).substr(strlen("world:"));
	WorldElementBase* we = classFactory_worldElements.create(sName, parent, root);

	if (!we) throw runtime_error(mrpt::format("[WorldElementBase::factory] Unknown world element type '%s'",root->name()));
	
	return we;
}
