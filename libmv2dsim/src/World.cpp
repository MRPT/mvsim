/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#include <mv2dsim/World.h>

#include <mrpt/utils/utils_defs.h>  // mrpt::format()
#include <iostream> // for debugging
#include <algorithm> // count()
#include <stdexcept>

// XML parsing:
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>

using namespace mv2dsim;
using namespace std;

// Default ctor: inits empty world.
World::World() : 
	m_box2d_world( b2Vec2_zero )
{
	this->clear_all();
}

// Dtor.
World::~World()
{
	this->clear_all();
}

// Resets the entire simulation environment to an empty world.
void World::clear_all(bool acquire_mt_lock)
{
	try
	{
		if (acquire_mt_lock) m_world_cs.enter();

		// Clear m_vehicles:
		for(std::list<VehicleBase*>::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it)
			delete *it;
		m_vehicles.clear();

		if (acquire_mt_lock) m_world_cs.leave();
	}
	catch (std::exception &)
	{
		if (acquire_mt_lock) m_world_cs.leave();
		throw; // re-throw
	}
}

/** Load an entire world description into this object from a specification in XML format.
	* \exception std::exception On any error, with what() giving a descriptive error message
	*/
void World::load_from_XML(const std::string &xml_text)
{
	using namespace std;
	using namespace rapidxml;

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
		throw std::runtime_error( mrpt::format("XML parse error (Line %u): %s", static_cast<unsigned>(line), line, e.what() ) );
	}

	// Sanity checks:
	const xml_node<> *root = xml.first_node();
	if (!root) throw runtime_error("XML parse error: No root node found (empty file?)");
	if (0!=strcmp(root->name(),"mv2dsim_world")) throw runtime_error(mrpt::format("XML root element is '%s' ('mv2dsim_world' expected)",root->name()));
	
	// Optional: format version attrib:
	const xml_attribute<> *attrb_version = root->first_attribute("version");
	int version_major = 1, version_min = 0;
	if (attrb_version)
		sscanf(attrb_version->value(),"%i.%i",&version_major, &version_min);

	// Process all nodes:
	xml_node<> *node = root->first_node();
	while (node)
	{
		if (!strcmp(node->name(),"world:gridmap")) 
		{
		} 
		else if (!strcmp(node->name(),"vehicle")) 
		{
			VehicleBase* veh = VehicleBase::factory(node);
			this->m_vehicles.push_back( veh );
		} 
		else 
		{
			// Unknown element!!
			std::cerr << "[World::load_from_XML] *Warning* Ignoring unknown XML node type '"<< node->name() <<"'\n";
		}

		// Move on to next node:
		node = node->next_sibling(NULL);
	}
		
}
