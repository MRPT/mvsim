/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/VehicleBase.h>
#include <mvsim/Sensors/LaserScanner.h>

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include <rapidxml_print.hpp>
#if MRPT_VERSION>=0x199
#include <mrpt/core/format.h>
#else
#include <mrpt/utils/utils_defs.h>  // mrpt::format()
#endif

#include <sstream>  // std::stringstream
#include <map>
#include <string>

using namespace mvsim;

TClassFactory_sensors mvsim::classFactory_sensors;

// Explicit registration calls seem to be one (the unique?) way to assure
// registration takes place:
void register_all_sensors()
{
	static bool done = false;
	if (done)
		return;
	else
		done = true;

	REGISTER_SENSOR("laser", LaserScanner)
}

SensorBase::SensorBase(VehicleBase& vehicle)
	: VisualObject(vehicle.getWorldObject()),
	  m_sensor_period(0.1),
	  m_vehicle(vehicle),
	  m_sensor_last_timestamp(0)
{
}

SensorBase::~SensorBase() {}
SensorBase* SensorBase::factory(
	VehicleBase& parent, const rapidxml::xml_node<char>* root)
{
	register_all_sensors();

	using namespace std;
	using namespace rapidxml;

	if (!root) throw runtime_error("[SensorBase::factory] XML node is nullptr");
	if (0 != strcmp(root->name(), "sensor"))
		throw runtime_error(
			mrpt::format(
				"[SensorBase::factory] XML root element is '%s' ('sensor' "
				"expected)",
				root->name()));

	// Get "class" attrib:
	const xml_attribute<>* sensor_class = root->first_attribute("class");
	if (!sensor_class || !sensor_class->value())
		throw runtime_error(
			"[VehicleBase::factory] Missing mandatory attribute 'class' in "
			"node <sensor>");

	const string sName = string(sensor_class->value());

	// Class factory:
	SensorBase* we = classFactory_sensors.create(sName, parent, root);

	if (!we)
		throw runtime_error(
			mrpt::format(
				"[SensorBase::factory] Unknown sensor type '%s'",
				root->name()));

	return we;
}
