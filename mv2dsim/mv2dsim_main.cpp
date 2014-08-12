/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/mv2dsim.h>

#include <rapidxml_utils.hpp>
#include <iostream>

	
int main(int argc, char **argv) 
{
    try 
	{
		if (argc!=2) return -1;

		mv2dsim::World  world;
		
		// Load from XML:
		rapidxml::file<> fil_xml(argv[1]);
		world.load_from_XML( fil_xml.data() );


    } catch (const std::exception& e) 
	{
        std::cerr << "ERROR: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
