/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/format.h>
#include <mvsim/Sensors/DepthCameraSensor.h>
#include <mvsim/Sensors/LaserScanner.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <map>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>
#include <sstream>	// std::stringstream
#include <string>

#include "xml_utils.h"

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include "GenericObservation.pb.h"
#endif

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
	REGISTER_SENSOR("rgbd_camera", DepthCameraSensor)
}

SensorBase::SensorBase(Simulable& vehicle)
	: VisualObject(vehicle.getSimulableWorldObject()),
	  Simulable(vehicle.getSimulableWorldObject()),
	  m_vehicle(vehicle)
{
}

SensorBase::~SensorBase() = default;

SensorBase::Ptr SensorBase::factory(
	Simulable& parent, const rapidxml::xml_node<char>* root)
{
	register_all_sensors();

	using namespace std;
	using namespace rapidxml;

	if (!root) throw runtime_error("[SensorBase::factory] XML node is nullptr");
	if (0 != strcmp(root->name(), "sensor"))
		throw runtime_error(mrpt::format(
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
	auto we = classFactory_sensors.create(sName, parent, root);

	if (!we)
		throw runtime_error(mrpt::format(
			"[SensorBase::factory] Unknown sensor type '%s'", root->name()));

	return we;
}

bool SensorBase::parseSensorPublish(
	const rapidxml::xml_node<char>* node,
	const std::map<std::string, std::string>& varValues)
{
	MRPT_START

	if (node == nullptr) return false;

	TParameterDefinitions params;
	params["publish_topic"] = TParamEntry("%s", &publishTopic_);

	// Parse XML params:
	parse_xmlnode_children_as_param(*node, params, varValues);

	return true;
	MRPT_END
}

void SensorBase::reportNewObservation(
	const std::shared_ptr<mrpt::obs::CObservation>& obs,
	const TSimulContext& context)
{
	// Notify the world:
	m_world->dispatchOnObservation(m_vehicle, obs);

	// Publish:
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	if (!publishTopic_.empty())
	{
		mvsim_msgs::GenericObservation msg;
		msg.set_unixtimestamp(mrpt::Clock::toDouble(obs->timestamp));
		msg.set_sourceobjectid(m_vehicle.getName());

		std::vector<uint8_t> serializedData;
		mrpt::serialization::ObjectToOctetVector(obs.get(), serializedData);

		msg.set_mrptserializedobservation(
			serializedData.data(), serializedData.size());

		context.world->commsClient().publishTopic(publishTopic_, msg);
	}
#endif
}

void SensorBase::registerOnServer(mvsim::Client& c)
{
	// Default base stuff:
	Simulable::registerOnServer(c);

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	// Topic:
	if (!publishTopic_.empty())
		c.advertiseTopic<mvsim_msgs::GenericObservation>(publishTopic_);
#endif
}
