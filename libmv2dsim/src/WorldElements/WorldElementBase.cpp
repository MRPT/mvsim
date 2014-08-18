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


WorldElementBase* WorldElementBase::factory(World* parent, const rapidxml::xml_node<char> *root)
{
	//classFactory_worldElements.create("",parent,root);

	using namespace std;
	using namespace rapidxml;

	if (!root) throw runtime_error("[WorldElementBase::factory] XML node is NULL");
	if (0!=strncmp(root->name(),"world:",strlen("world:"))) throw runtime_error(mrpt::format("[WorldElementBase::factory] XML root element is '%s' ('world:*' expected)",root->name()));

	WorldElementBase* we = NULL;
	if (!strcmp(root->name(),"world:gridmap"))
	{
		we = new mv2dsim::OccupancyGridMap(parent,root);
	}

	if (!we)
		throw runtime_error(mrpt::format("[WorldElementBase::factory] Unknown world element type '%s'",root->name()));


	return we;
}
