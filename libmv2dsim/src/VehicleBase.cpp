/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleBase.h>
#include <mv2dsim/VehicleDynamics/VehicleDifferential.h>

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include <mrpt/utils/utils_defs.h>  // mrpt::format()

#include <sstream>      // std::stringstream

using namespace mv2dsim;
using namespace std;


// Protected ctor:
VehicleBase::VehicleBase() : 
	m_b2_world(NULL)
{
}

/** Class factory: Creates a vehicle from XML description of type "<vehicle>...</vehicle>".  */
VehicleBase* VehicleBase::factory( const rapidxml::xml_node<char> *root)
{
	using namespace std;
	using namespace rapidxml;

	if (!root) throw runtime_error("[VehicleBase::factory] XML node is NULL");
	if (0!=strcmp(root->name(),"vehicle")) throw runtime_error(mrpt::format("[VehicleBase::factory] XML root element is '%s' ('vehicle' expected)",root->name()));

	// Class factory according to: <dynamics class="XXX">
	// -------------------------------------------------
	const xml_node<> *dyn_node = root->first_node("dynamics");
	if (!dyn_node) throw runtime_error("[VehicleBase::factory] Missing XML node <dynamics>");

	const xml_attribute<> *dyn_class = dyn_node->first_attribute("class");
	if (!dyn_class || !dyn_class->value() ) throw runtime_error("[VehicleBase::factory] Missing mandatory attribute 'class' in node <dynamics>");

	VehicleBase *veh = NULL;
	if (!strcmp("differential",dyn_class->value()))
	{
		veh = new DynamicsDifferential;
	}
	else 
	{
		throw runtime_error(mrpt::format("[VehicleBase::factory] Unknown vehicle dynamics class '%s'",dyn_class->value()));		
	}

	// Initialize class-specific params:
	// -------------------------------------------------
	veh->dynamics_load_params_from_xml(dyn_node);
	
	// Initialize here all common params shared by any polymorphic class:
	// -------------------------------------------------
	// attrib: name
		
	// initial pose:


	// Vehicle controller:
	// -------------------------------------------------

	// Friction model:
	// -------------------------------------------------

	// Sensors:
	// -------------------------------------------------


	return veh;
}

VehicleBase* VehicleBase::factory( const std::string &xml_text)
{
	// Parse the string as if it was an XML file:
	std::stringstream s;
	s.str( xml_text );

	rapidxml::file<> fil(s);
	char* input_str = const_cast<char*>(xml_text.c_str());
	rapidxml::xml_document<> xml;
	try {
		xml.parse<0>(input_str);
	}
	catch (rapidxml::parse_error &e) {
		unsigned int line = static_cast<long>(std::count(input_str, e.where<char>(), '\n') + 1);
		throw std::runtime_error( mrpt::format("[VehicleBase::factory] XML parse error (Line %u): %s", static_cast<unsigned>(line), line, e.what() ) );
	}
	return VehicleBase::factory(xml.first_node());
}

/** Loads vehicle params from input XML node of type "<vehicle>...</vehicle>". 
	* See derived classes & documentation for a list of accepted params.  
	*/
void VehicleBase::load_params_from_xml(const rapidxml::xml_node<char> *xml_node)
{
}

/// \overload
void VehicleBase::load_params_from_xml(const std::string &xml_text)
{

}
