/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#include <mvsim/World.h>

#include <mrpt/utils/utils_defs.h>  // mrpt::format()
#include <mrpt/system/filesystem.h> // extractFileDirectory()

#include <iostream> // for debugging
#include <algorithm> // count()
#include <stdexcept>
#include <map>

// XML parsing:
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

/** Load an entire world description into this object from a specification in XML format.
	* \exception std::exception On any error, with what() giving a descriptive error message
	*/
void World::load_from_XML(const std::string &xml_text, const std::string &fileNameForPath)
{
	using namespace std;
	using namespace rapidxml;

	// Extract base path of file:
	m_base_path = mrpt::system::trim( mrpt::system::extractFileDirectory(fileNameForPath) );
	//printf("[World] INFO: Using base path='%s'\n",m_base_path.c_str());

	mrpt::synch::CCriticalSectionLocker csl( &m_world_cs ); // Protect multithread access

	// Clear the existing world.
	this->clear_all(false /* critical section is already acquired */);

	// Parse the XML input:
	rapidxml::xml_document<> xml;
	char* input_str = const_cast<char*>(xml_text.c_str());
	try {
		xml.parse<0>(input_str);
	}
	catch (rapidxml::parse_error &e) {
		unsigned int line = static_cast<long>(std::count(input_str, e.where<char>(), '\n') + 1);
		throw std::runtime_error( mrpt::format("XML parse error (Line %u): %s", static_cast<unsigned>(line), e.what() ) );
	}

	// Sanity checks:
	const xml_node<> *root = xml.first_node();
	if (!root) throw runtime_error("XML parse error: No root node found (empty file?)");
	if (0!=strcmp(root->name(),"mvsim_world")) throw runtime_error(mrpt::format("XML root element is '%s' ('mvsim_world' expected)",root->name()));

	// Optional: format version attrib:
	const xml_attribute<> *attrb_version = root->first_attribute("version");
	int version_major = 1, version_min = 0;
	if (attrb_version)
	{
		int ret = sscanf(attrb_version->value(),"%i.%i",&version_major, &version_min);
		if (ret!=2) throw runtime_error(mrpt::format("Error parsing version attribute: '%s' ('%%i.%%i' expected)",attrb_version->value()));
	}

	// load general parameters:
	// ------------------------------------------------
	std::map<std::string,TParamEntry> other_world_params;
	other_world_params["gravity"] = TParamEntry("%lf", &this->m_gravity);
	other_world_params["simul_timestep"] = TParamEntry("%lf", &this->m_simul_timestep);
	other_world_params["b2d_vel_iters"] = TParamEntry("%i", &this->m_b2d_vel_iters);
	other_world_params["b2d_pos_iters"] = TParamEntry("%i", &this->m_b2d_pos_iters);

	MRPT_TODO("Export this list of params to ROS dynamic reconfigure");

	// Process all nodes:
	// ------------------------------------------------
	xml_node<> *node = root->first_node();
	while (node)
	{
		// <element class='*'> entries:
		if (!strcmp(node->name(),"element"))
		{
			WorldElementBase *we = WorldElementBase::factory(this,node);
			this->m_world_elements.push_back(we);
		}
		// <vehicle> entries:
		else if (!strcmp(node->name(),"vehicle"))
		{
			VehicleBase* veh = VehicleBase::factory(this,node);
			// make sure the name is not duplicated:
			m_vehicles.insert( TListVehicles::value_type(veh->getName(),veh));
		}
		// <vehicle:class> entries:
		else if (!strcmp(node->name(),"vehicle:class"))
		{
			VehicleBase::register_vehicle_class(node);
		}
		// <gui> </gui> params:
		else if (!strcmp(node->name(),"gui"))
		{
			m_gui_options.parse_from(*node);
		}
		else
		{
			// Default: Check if it's a parameter:
			if (!parse_xmlnode_as_param(*node,other_world_params) )
			{
				// Unknown element!!
				std::cerr << "[World::load_from_XML] *Warning* Ignoring unknown XML node type '"<< node->name() <<"'\n";
			}
		}

		// Move on to next node:
		node = node->next_sibling(NULL);
	}

}
