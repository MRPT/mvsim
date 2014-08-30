/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mvsim/VehicleBase.h>
#include <mvsim/Sensors/LaserScanner.h>

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include <rapidxml_print.hpp>
#include <mrpt/utils/utils_defs.h>  // mrpt::format()

#include <sstream>      // std::stringstream
#include <map>
#include <string>

using namespace mvsim;

TClassFactory_sensors mvsim::classFactory_sensors;

// Explicit registration calls seem to be one (the unique?) way to assure registration takes place:
void register_all_sensors()
{
	static bool done = false;
	if (done) return; else done=true;

	REGISTER_SENSOR("laser",LaserScanner)
}

SensorBase::SensorBase(VehicleBase&vehicle) : 
	VisualObject( vehicle.getWorldObject() ),
	m_vehicle(vehicle)
{ 
}
SensorBase::~SensorBase() 
{ 
}

SensorBase* SensorBase::factory(VehicleBase& parent, const rapidxml::xml_node<char> *root)
{
	register_all_sensors();

	using namespace std;
	using namespace rapidxml;

	if (!root) throw runtime_error("[SensorBase::factory] XML node is NULL");
	if (0!=strcmp(root->name(),"sensor")) throw runtime_error(mrpt::format("[SensorBase::factory] XML root element is '%s' ('sensor' expected)",root->name()));

	// Get "class" attrib:
	const xml_attribute<> *sensor_class = root->first_attribute("class");
	if (!sensor_class || !sensor_class->value() ) throw runtime_error("[VehicleBase::factory] Missing mandatory attribute 'class' in node <sensor>");

	const string sName  = string(sensor_class->value());

	// Class factory:
	SensorBase* we = classFactory_sensors.create(sName, parent, root);

	if (!we) throw runtime_error(mrpt::format("[SensorBase::factory] Unknown sensor type '%s'",root->name()));
	
	return we;
}
