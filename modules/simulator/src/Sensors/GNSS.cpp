/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/version.h>
#include <mrpt/viz/stock_objects.h>
#include <mvsim/Sensors/GNSS.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <cmath>
#include <cstdlib>
#include <sstream>

#include "xml_utils.h"

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
// #include <mvsim/mvsim-msgs/ObservationXXX.pb.h>
#endif

using namespace mvsim;
using namespace rapidxml;

GNSS::GNSS(Simulable& parent, const rapidxml::xml_node<char>* root) : SensorBase(parent)
{
	GNSS::loadConfigFrom(root);
}

GNSS::~GNSS() {}

namespace
{
mrpt::obs::GnssFixType parseFixType(const std::string& s)
{
	using mrpt::obs::GnssFixType;
	if (s == "no_fix")
	{
		return GnssFixType::NO_FIX;
	}
	if (s == "single")
	{
		return GnssFixType::AUTONOMOUS;
	}
	if (s == "dgps")
	{
		return GnssFixType::DGPS;
	}
	if (s == "rtk_float")
	{
		return GnssFixType::RTK_FLOAT;
	}
	if (s == "rtk_fixed")
	{
		return GnssFixType::RTK_FIXED;
	}
	THROW_EXCEPTION_FMT(
		"Invalid GNSS fix_type '%s' (valid: no_fix, single, dgps, rtk_float, rtk_fixed)",
		s.c_str());
}

/** Parses an optional, finite and non-negative noise value of an <event> */
std::optional<double> parseOptionalStd(const std::string& s, const char* name)
{
	if (s.empty())
	{
		return std::nullopt;
	}
	char* end = nullptr;
	const double v = std::strtod(s.c_str(), &end);
	ASSERTMSG_(
		end != s.c_str() && *end == '\0' && std::isfinite(v) && v >= 0,
		mrpt::format(
			"GNSS <event>: '%s' must be a finite, non-negative number, got '%s'", name, s.c_str()));
	return v;
}

/** NMEA GGA fix quality for a fix type */
uint8_t ggaFixQuality(mrpt::obs::GnssFixType t)
{
	using mrpt::obs::GnssFixType;
	switch (t)
	{
		case GnssFixType::NO_FIX:
			return 0;
		case GnssFixType::AUTONOMOUS:
			return 1;
		case GnssFixType::RTK_FIXED:
			return 4;
		case GnssFixType::RTK_FLOAT:
			return 5;
		default:
			return 2;  // DGPS
	}
}
}  // namespace

void GNSS::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	SensorBase::loadConfigFrom(root);
	SensorBase::make_sure_we_have_a_name("imu");

	TParameterDefinitions params;
	params["pose"] = TParamEntry("%pose2d_ptr3d", &obs_model_.sensorPose);
	params["pose_3d"] = TParamEntry("%pose3d", &obs_model_.sensorPose);
	params["sensor_period"] = TParamEntry("%lf", &sensor_period_);
	params["horizontal_std_noise"] = TParamEntry("%lf", &horizontal_std_noise_);
	params["vertical_std_noise"] = TParamEntry("%lf", &vertical_std_noise_);

	std::string fixType = "dgps";
	params["fix_type"] = TParamEntry("%s", &fixType);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues_);

	fix_type_ = parseFixType(fixType);

	// Degradation events:
	events_.clear();
	for (auto n = root->first_node("event"); n; n = n->next_sibling("event"))
	{
		Event ev;
		std::string evFixType;
		std::string offset;
		std::string hStd;
		std::string vStd;
		TParameterDefinitions attribs;
		attribs["start"] = TParamEntry("%lf", &ev.start);
		attribs["end"] = TParamEntry("%lf", &ev.end);
		attribs["outage"] = TParamEntry("%bool", &ev.outage);
		attribs["fix_type"] = TParamEntry("%s", &evFixType);
		attribs["horizontal_std_noise"] = TParamEntry("%s", &hStd);
		attribs["vertical_std_noise"] = TParamEntry("%s", &vStd);
		attribs["offset"] = TParamEntry("%s", &offset);
		parse_xmlnode_attribs(*n, attribs, varValues_, "[GNSS]");

		ASSERTMSG_(
			n->first_attribute("start") && n->first_attribute("end"),
			"GNSS <event>: both 'start' and 'end' are required");
		ASSERTMSG_(ev.end > ev.start, "GNSS <event>: 'end' must be greater than 'start'");
		ASSERTMSG_(
			!n->first_attribute("fix_type") || !evFixType.empty(),
			"GNSS <event>: 'fix_type' cannot be empty");
		if (!evFixType.empty())
		{
			ev.fix_type = parseFixType(evFixType);
		}
		ev.horizontal_std_noise = parseOptionalStd(hStd, "horizontal_std_noise");
		ev.vertical_std_noise = parseOptionalStd(vStd, "vertical_std_noise");
		if (!offset.empty())
		{
			std::stringstream ss(offset);
			ss >> ev.offset.x >> ev.offset.y >> ev.offset.z;
			ASSERTMSG_(!ss.fail(), "GNSS <event>: 'offset' must be 'dx dy dz'");
		}
		events_.push_back(ev);
	}

	// Pass params to the template obj:
	obs_model_.sensorLabel = name_;

	// Init ENU covariance:
	auto& C = obs_model_.covariance_enu.emplace();
	const double var_xy = mrpt::square(horizontal_std_noise_);
	const double var_z = mrpt::square(vertical_std_noise_);

	C.setDiagonal(std::vector<double>{var_xy, var_xy, var_z});
}

void GNSS::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::viz::Scene>& viz,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::viz::Scene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	// 1st time?
	if (!gl_sensor_origin_ && viz)
	{
		gl_sensor_origin_ = mrpt::viz::CSetOfObjects::Create();
		gl_sensor_origin_->castShadows(false);
		gl_sensor_origin_corner_ = mrpt::viz::stock_objects::CornerXYZSimple(0.15f);

		gl_sensor_origin_->insert(gl_sensor_origin_corner_);

		gl_sensor_origin_->setVisibility(false);
		insertIntoScene(viz->get(), gl_sensor_origin_);
		insertIntoScene(SensorBase::GetAllSensorsOriginViz(), gl_sensor_origin_);
	}

	const mrpt::poses::CPose3D p =
		mrpt::poses::CPose3D(vehicle_.getChassisPose()) + obs_model_.sensorPose;
	const auto pp = parent()->applyWorldRenderOffset(p);

	if (gl_sensor_origin_)
	{
		gl_sensor_origin_->setPose(pp);
	}
	if (glCustomVisual_)
	{
		glCustomVisual_->setPose(pp);
	}
}

void GNSS::simul_pre_timestep([[maybe_unused]] const TSimulContext& context) {}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void GNSS::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);

	if (SensorBase::should_simulate_sensor(context))
	{
		internal_simulate_gnss(context);
	}

	// Keep sensor global pose up-to-date:
	const auto& p = vehicle_.getChassisPose();
	const auto globalSensorPose = p + obs_model_.sensorPose.asTPose();
	Simulable::setPose(globalSensorPose, false /*do not notify*/);
}

std::string GNSS::fixTypeName(mrpt::obs::GnssFixType t)
{
	using mrpt::obs::GnssFixType;
	switch (t)
	{
		case GnssFixType::NO_FIX:
			return "no_fix";
		case GnssFixType::AUTONOMOUS:
			return "single";
		case GnssFixType::RTK_FLOAT:
			return "rtk_float";
		case GnssFixType::RTK_FIXED:
			return "rtk_fixed";
		default:
			return "dgps";
	}
}

void GNSS::internal_simulate_gnss(const TSimulContext& context)
{
	auto tle = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.GNSS");

	if (!noiseSeeded_)
	{
		noiseSeeded_ = true;
		seedNoiseGenerator(rng_);
	}

	auto outObs = simulateReading(
		mrpt::poses::CPose3D(vehicle().getChassisPose()), world_->get_simul_time(),
		world_->get_simul_timestamp(), rng_);
	if (!outObs)
	{
		return;
	}

	// Save:
	{
		std::lock_guard<std::mutex> csl(last_obs_cs_);
		last_obs_ = std::move(outObs);
	}

	// publish as generic Protobuf (mrpt serialized) object:
	SensorBase::reportNewObservation(last_obs_, context);
}

GNSS::Quality GNSS::qualityAt(double t) const
{
	Quality q;
	q.fix_type = fix_type_;
	q.horizontal_std_noise = horizontal_std_noise_;
	q.vertical_std_noise = vertical_std_noise_;
	for (const auto& ev : events_)
	{
		if (t < ev.start || t >= ev.end)
		{
			continue;
		}
		if (ev.outage)
		{
			q.outage = true;
			return q;
		}
		q.fix_type = ev.fix_type.value_or(q.fix_type);
		q.horizontal_std_noise = ev.horizontal_std_noise.value_or(q.horizontal_std_noise);
		q.vertical_std_noise = ev.vertical_std_noise.value_or(q.vertical_std_noise);
		q.offset = q.offset + ev.offset;
	}
	return q;
}

mrpt::obs::CObservationGPS::Ptr GNSS::simulateReading(
	const mrpt::poses::CPose3D& vehiclePose, double t, const mrpt::Clock::time_point& stamp,
	mrpt::random::CRandomGenerator& rng, double noiseScale) const
{
	using mrpt::obs::CObservationGPS;

	// Are we into a no-coverage area? (regions are in world coordinates)
	const auto noCoverageProp = world()->getPropertyAt(
		"gps_no_coverage", (vehiclePose + obs_model_.sensorPose).translation());
	if (noCoverageProp.has_value())
	{
		const std::any& anyVal = *noCoverageProp;
		const bool* noCoverage = std::any_cast<bool>(&anyVal);
		if (noCoverage == nullptr)
		{
			THROW_EXCEPTION("'gps_no_coverage' property must be a bool");
		}
		if (*noCoverage == true)
		{
			// We don't have GPS coverage here. Skip.
			return {};
		}
	}

	// The vehicle pose relative to the georeference origin (in UTM worlds,
	// world coordinates are UTM coordinates):
	mrpt::poses::CPose3D vehPoseLocal = vehiclePose;
	const auto& georef = world()->georeferenceOptions();
	if (georef.world_is_utm)
	{
		auto posLocal = vehPoseLocal.translation() - georef.utmRef;

		vehPoseLocal.x(posLocal.x);
		vehPoseLocal.y(posLocal.y);
		vehPoseLocal.z(posLocal.z);
	}

	const auto worldRotation =
		mrpt::poses::CPose3D::FromYawPitchRoll(georef.world_to_enu_rotation, .0, .0);

	// The sensor position in ENU, without noise:
	const mrpt::math::TPoint3D sensorPtNoNoise =
		(worldRotation + (vehPoseLocal + obs_model_.sensorPose)).translation();

	// Quality and degradation events:
	const Quality q = qualityAt(t);
	if (q.outage)
	{
		return {};	// no data at all
	}
	const auto fixType = q.fix_type;
	const double hStd = q.horizontal_std_noise;
	const double vStd = q.vertical_std_noise;
	const mrpt::math::TPoint3D offset = q.offset;

	auto outObs = CObservationGPS::Create(obs_model_);

	outObs->timestamp = stamp;
	outObs->sensorLabel = name_;
	outObs->fix_type = fixType;
	outObs->covariance_enu.emplace();
	outObs->covariance_enu->setDiagonal(
		std::vector<double>{mrpt::square(hStd), mrpt::square(hStd), mrpt::square(vStd)});

	// noise:
	const mrpt::math::TPoint3D noise = {
		rng.drawGaussian1D(0.0, hStd * noiseScale), rng.drawGaussian1D(0.0, hStd * noiseScale),
		rng.drawGaussian1D(0.0, vStd * noiseScale)};

	const mrpt::math::TPoint3D sensorPt = sensorPtNoNoise + noise + offset * noiseScale;

	// convert from ENU (world coordinates) to geodetic:
	const thread_local auto WGS84 = mrpt::topography::TEllipsoid::Ellipsoid_WGS84();

	const mrpt::topography::TGeodeticCoords& georefCoord = georef.georefCoord;

	// Warn the user if settings not set:
	if (!georef.is_set)
	{
		thread_local bool once = false;
		if (!once)
		{
			once = true;
			world()->logStr(
				mrpt::system::LVL_WARN,
				"World <georeference> parameters are not set, and they are required for "
				"properly define GNSS sensor simulation");
		}
		return {};
	}

	mrpt::topography::TGeocentricCoords gcPt;
	mrpt::topography::ENUToGeocentric(sensorPt, georefCoord, gcPt, WGS84);

	mrpt::topography::TGeodeticCoords ptCoords;
	mrpt::topography::geocentricToGeodetic(gcPt, ptCoords, WGS84);

	// Fill in observation:
	mrpt::obs::gnss::Message_NMEA_GGA msgGGA;
	auto& f = msgGGA.fields;
	f.thereis_HDOP = true;
	f.HDOP = mrpt::d2f(hStd / 5.0);	 // approximation

	mrpt::system::TTimeParts tp;
	mrpt::system::timestampToParts(outObs->timestamp, tp);
	f.UTCTime.hour = tp.hour;
	f.UTCTime.minute = tp.minute;
	f.UTCTime.sec = tp.second;
	f.fix_quality = ggaFixQuality(fixType);

	f.latitude_degrees = ptCoords.lat.decimal_value;
	f.longitude_degrees = ptCoords.lon.decimal_value;

	f.altitude_meters = ptCoords.height;
	f.orthometric_altitude = ptCoords.height;
	f.corrected_orthometric_altitude = ptCoords.height;
	f.satellitesUsed = 7;  // How to simulate this? :-)

	outObs->setMsg(msgGGA);

	return outObs;
}

void GNSS::notifySimulableSetPose(const mrpt::math::TPose3D&)
{
	// The editor has moved the sensor in global coordinates.
	// Convert back to local:
	// const auto& p = vehicle_.getChassisPose();
	// sensor_params_.sensorPose = mrpt::poses::CPose3D(newPose - p);
}

void GNSS::registerOnServer(mvsim::Client& c)
{
	using namespace std::string_literals;

	SensorBase::registerOnServer(c);

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	// Topic:
	if (!publishTopic_.empty())
	{
		// c.advertiseTopic<mvsim_msgs::ObservationIMU>(publishTopic_ +
		// "_scan"s);
	}
#endif
}
