/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/FrictionModels/FrictionBase.h>
#include <mv2dsim/FrictionModels/LinearFriction.h>
#include <mv2dsim/VehicleBase.h>

#include <rapidxml.hpp>
//#include <rapidxml_utils.hpp>
//#include <rapidxml_print.hpp>
//#include <mrpt/utils/utils_defs.h>  // mrpt::format()

//#include <sstream>      // std::stringstream
//#include <map>
//#include <string>

using namespace mv2dsim;

TClassFactory_friction mv2dsim::classFactory_friction;

// Explicit registration calls seem to be one (the unique?) way to assure registration takes place:
void register_all_friction()
{
	static bool done = false;
	if (done) return; else done=true;

	REGISTER_FRICTION("linear",LinearFriction)
}

FrictionBase::FrictionBase(VehicleBase & my_vehicle) :
	m_world(my_vehicle.getWorldObject()),
	m_my_vehicle(my_vehicle)
{
}

FrictionBase::~FrictionBase() 
{ 
}

FrictionBase* FrictionBase::factory(VehicleBase & parent, const rapidxml::xml_node<char> *xml_node)
{
	register_all_friction();

	using namespace std;
	using namespace rapidxml;

	if (!xml_node || 0!=strcmp(xml_node->name(),"friction") )
		throw runtime_error("[FrictionBase::factory] Expected XML node <friction>");

	// Parse:
	const xml_attribute<> *frict_class = xml_node->first_attribute("class");
	if (!frict_class || !frict_class->value() ) throw runtime_error("[FrictionBase::factory] Missing mandatory attribute 'class' in node <friction>");

	return classFactory_friction.create(frict_class->value(),parent,xml_node);
}

