/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/format.h>
#include <mvsim/Sensors/CameraSensor.h>
#include <mvsim/Sensors/DepthCameraSensor.h>
#include <mvsim/Sensors/GNSS.h>
#include <mvsim/Sensors/IMU.h>
#include <mvsim/Sensors/LaserScanner.h>
#include <mvsim/Sensors/Lidar3D.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <map>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>
#include <sstream>	// std::stringstream
#include <string>

#include "parse_utils.h"
#include "xml_utils.h"

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <mvsim/mvsim-msgs/GenericObservation.pb.h>
#include <mvsim/mvsim-msgs/ObservationLidar2D.pb.h>
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
	REGISTER_SENSOR("camera", CameraSensor)
	REGISTER_SENSOR("lidar3d", Lidar3D)
	REGISTER_SENSOR("imu", IMU)
	REGISTER_SENSOR("gnss", GNSS)
}

static auto gAllSensorsOriginViz = mrpt::opengl::CSetOfObjects::Create();
static auto gAllSensorsFOVViz = mrpt::opengl::CSetOfObjects::Create();
static std::mutex gAllSensorVizMtx;

mrpt::opengl::CSetOfObjects::Ptr SensorBase::GetAllSensorsOriginViz()
{
	auto lck = mrpt::lockHelper(gAllSensorVizMtx);
	return gAllSensorsOriginViz;
}

mrpt::opengl::CSetOfObjects::Ptr SensorBase::GetAllSensorsFOVViz()
{
	auto lck = mrpt::lockHelper(gAllSensorVizMtx);
	return gAllSensorsFOVViz;
}

void SensorBase::RegisterSensorFOVViz(const mrpt::opengl::CSetOfObjects::Ptr& o)
{
	auto lck = mrpt::lockHelper(gAllSensorVizMtx);
	gAllSensorsFOVViz->insert(o);
}
void SensorBase::RegisterSensorOriginViz(const mrpt::opengl::CSetOfObjects::Ptr& o)
{
	auto lck = mrpt::lockHelper(gAllSensorVizMtx);
	gAllSensorsOriginViz->insert(o);
}

SensorBase::SensorBase(Simulable& vehicle)
	: VisualObject(vehicle.getSimulableWorldObject()),
	  Simulable(vehicle.getSimulableWorldObject()),
	  vehicle_(vehicle)
{
}

SensorBase::~SensorBase() = default;

SensorBase::Ptr SensorBase::factory(Simulable& parent, const rapidxml::xml_node<char>* root)
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
		throw runtime_error(
			mrpt::format("[SensorBase::factory] Unknown sensor type '%s'", root->name()));

	// parse the optional visual model:
	we->parseVisual(*root);

	return we;
}

bool SensorBase::parseSensorPublish(
	const rapidxml::xml_node<char>* node, const std::map<std::string, std::string>& varValues)
{
	MRPT_START

	if (node == nullptr) return false;

	// Parse XML params:
	{
		TParameterDefinitions params;
		params["publish_topic"] = TParamEntry("%s", &publishTopic_);

		parse_xmlnode_children_as_param(*node, params, varValues);
	}

	// Parse the "enabled" attribute:
	{
		bool publishEnabled = true;
		TParameterDefinitions auxPar;
		auxPar["enabled"] = TParamEntry("%bool", &publishEnabled);
		parse_xmlnode_attribs(*node, auxPar, varValues);

		// Reset publish topic if enabled==false
		if (!publishEnabled) publishTopic_.clear();
	}

	return true;
	MRPT_END
}

void SensorBase::reportNewObservation(
	const std::shared_ptr<mrpt::obs::CObservation>& obs, const TSimulContext& context)
{
	if (!obs) return;

	// Notify the world:
	world_->dispatchOnObservation(vehicle_, obs);

	// Publish:
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	if (!publishTopic_.empty())
	{
		mvsim_msgs::GenericObservation msg;
		msg.set_unixtimestamp(mrpt::Clock::toDouble(obs->timestamp));
		msg.set_sourceobjectid(vehicle_.getName());

		std::vector<uint8_t> serializedData;
		mrpt::serialization::ObjectToOctetVector(obs.get(), serializedData);

		msg.set_mrptserializedobservation(serializedData.data(), serializedData.size());

		context.world->commsClient().publishTopic(publishTopic_, msg);
	}
#endif
}

void SensorBase::reportNewObservation_lidar_2d(
	const std::shared_ptr<mrpt::obs::CObservation2DRangeScan>& obs, const TSimulContext& context)
{
	using namespace std::string_literals;

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	if (publishTopic_.empty()) return;

	mvsim_msgs::ObservationLidar2D msg;
	msg.set_unixtimestamp(mrpt::Clock::toDouble(obs->timestamp));
	msg.set_sourceobjectid(vehicle_.getName());

	msg.set_aperture(obs->aperture);
	for (size_t i = 0; i < obs->getScanSize(); i++)
	{
		msg.add_scanranges(obs->getScanRange(i));
		msg.add_validranges(obs->getScanRangeValidity(i));
	}

	msg.set_ccw(obs->rightToLeft);
	msg.set_maxrange(obs->maxRange);

	const auto& p = obs->sensorPose;
	auto sp = msg.mutable_sensorpose();
	sp->set_x(p.x());
	sp->set_y(p.y());
	sp->set_z(p.z());
	sp->set_yaw(p.yaw());
	sp->set_pitch(p.pitch());
	sp->set_roll(p.roll());

	context.world->commsClient().publishTopic(publishTopic_ + "_scan"s, msg);
#endif
}

void SensorBase::registerOnServer(mvsim::Client& c)
{
	// Default base stuff:
	Simulable::registerOnServer(c);

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	// Topic:
	if (!publishTopic_.empty()) c.advertiseTopic<mvsim_msgs::GenericObservation>(publishTopic_);
#endif
}

void SensorBase::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	// Attribs:
	TParameterDefinitions attribs;
	attribs["name"] = TParamEntry("%s", &name_);
	parse_xmlnode_attribs(*root, attribs, {}, "[SensorBase]");

	varValues_ = world_->user_defined_variables();
	varValues_["NAME"] = name_;
	varValues_["PARENT_NAME"] = vehicle_.getName();

	// Parse common sensor XML params:
	this->parseSensorPublish(root->first_node("publish"), varValues_);

	TParameterDefinitions params;
	params["sensor_period"] = TParamEntry("%lf", &sensor_period_);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues_);
}

void SensorBase::make_sure_we_have_a_name(const std::string& prefix)
{
	if (!name_.empty()) return;

	size_t nextIdx = 0;
	if (auto v = dynamic_cast<VehicleBase*>(&vehicle_); v) nextIdx = v->getSensors().size() + 1;

	name_ = mrpt::format("%s%u", prefix.c_str(), static_cast<unsigned int>(nextIdx));
}

bool SensorBase::should_simulate_sensor(const TSimulContext& context)
{
	// to fix edge cases with sensor period a multiple of simulation timestep:
	const double timeEpsilon = 1e-6;

	if (context.simul_time < sensor_last_timestamp_ + sensor_period_ - timeEpsilon) return false;

	if ((context.simul_time - sensor_last_timestamp_) >= 2 * sensor_period_)
	{
		std::cout << "[mvsim::SensorBase] WARNING: "
					 "At least one sensor sample has been lost due to too coarse "
					 "discrete time steps. sensor_period="
				  << sensor_period_ << " [s], (simul_time - sensor_last_timestamp)="
				  << (context.simul_time - sensor_last_timestamp_) << " [s]." << std::endl;
	}

	sensor_last_timestamp_ = context.simul_time;

	return true;
}
