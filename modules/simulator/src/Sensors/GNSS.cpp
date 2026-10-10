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
		double hStd = -1;
		double vStd = -1;
		TParameterDefinitions attribs;
		attribs["start"] = TParamEntry("%lf", &ev.start);
		attribs["end"] = TParamEntry("%lf", &ev.end);
		attribs["outage"] = TParamEntry("%bool", &ev.outage);
		attribs["fix_type"] = TParamEntry("%s", &evFixType);
		attribs["horizontal_std_noise"] = TParamEntry("%lf", &hStd);
		attribs["vertical_std_noise"] = TParamEntry("%lf", &vStd);
		attribs["offset"] = TParamEntry("%s", &offset);
		parse_xmlnode_attribs(*n, attribs, varValues_, "[GNSS]");

		ASSERTMSG_(ev.end > ev.start, "GNSS <event>: 'end' must be greater than 'start'");
		if (!evFixType.empty())
		{
			ev.fix_type = parseFixType(evFixType);
		}
		if (hStd >= 0)
		{
			ev.horizontal_std_noise = hStd;
		}
		if (vStd >= 0)
		{
			ev.vertical_std_noise = vStd;
		}
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
		viz->get().insert(gl_sensor_origin_);
		SensorBase::RegisterSensorOriginViz(gl_sensor_origin_);
	}

	const mrpt::poses::CPose3D p = vehicle_.getCPose3D() + obs_model_.sensorPose;
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
	const auto& p = vehicle_.getPose();
	const auto globalSensorPose = p + obs_model_.sensorPose.asTPose();
	Simulable::setPose(globalSensorPose, false /*do not notify*/);
}

void GNSS::internal_simulate_gnss(const TSimulContext& context)
{
	using mrpt::obs::CObservationGPS;

	auto tle = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.GNSS");

	// Where the GPS sensor is in the world frame:
	mrpt::poses::CPose3D vehPoseInWorld = vehicle().getCPose3D();
	const auto& georef = world()->georeferenceOptions();
	if (georef.world_is_utm)
	{
		auto posLocal = vehPoseInWorld.translation() - georef.utmRef;

		vehPoseInWorld.x(posLocal.x);
		vehPoseInWorld.y(posLocal.y);
		vehPoseInWorld.z(posLocal.z);
	}

	const auto worldRotation =
		mrpt::poses::CPose3D::FromYawPitchRoll(georef.world_to_enu_rotation, .0, .0);

	const mrpt::math::TPoint3D sensorPtNoNoise =
		(worldRotation + (vehPoseInWorld + obs_model_.sensorPose)).translation();

	// Are we into a no-coverage area?
	const auto noCoverageProp = world_->getPropertyAt("gps_no_coverage", sensorPtNoNoise);
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
			return;
		}
	}

	// Quality and degradation events:
	auto fixType = fix_type_;
	double hStd = horizontal_std_noise_;
	double vStd = vertical_std_noise_;
	mrpt::math::TPoint3D offset = {0, 0, 0};
	const double t = world_->get_simul_time();	// the observation timestamp
	for (const auto& ev : events_)
	{
		if (t < ev.start || t >= ev.end)
		{
			continue;
		}
		if (ev.outage)
		{
			return;	 // no data at all
		}
		fixType = ev.fix_type.value_or(fixType);
		hStd = ev.horizontal_std_noise.value_or(hStd);
		vStd = ev.vertical_std_noise.value_or(vStd);
		offset = offset + ev.offset;
	}

	auto outObs = CObservationGPS::Create(obs_model_);

	outObs->timestamp = world_->get_simul_timestamp();
	outObs->sensorLabel = name_;
	outObs->fix_type = fixType;
	outObs->covariance_enu.emplace();
	outObs->covariance_enu->setDiagonal(
		std::vector<double>{mrpt::square(hStd), mrpt::square(hStd), mrpt::square(vStd)});

	// noise:
	const mrpt::math::TPoint3D noise = {
		rng_.drawGaussian1D(0.0, hStd), rng_.drawGaussian1D(0.0, hStd),
		rng_.drawGaussian1D(0.0, vStd)};

	const mrpt::math::TPoint3D sensorPt = sensorPtNoNoise + noise + offset;

	// convert from ENU (world coordinates) to geodetic:
	const thread_local auto WGS84 = mrpt::topography::TEllipsoid::Ellipsoid_WGS84();

	const mrpt::topography::TGeodeticCoords& georefCoord = georef.georefCoord;

	// Warn the user if settings not set:
	if (georefCoord.lat.decimal_value == 0 && georefCoord.lon.decimal_value == 0)
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
		return;
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

	// Save:
	{
		std::lock_guard<std::mutex> csl(last_obs_cs_);
		last_obs_ = std::move(outObs);
	}

	// publish as generic Protobuf (mrpt serialized) object:
	SensorBase::reportNewObservation(last_obs_, context);
}

void GNSS::notifySimulableSetPose(const mrpt::math::TPose3D&)
{
	// The editor has moved the sensor in global coordinates.
	// Convert back to local:
	// const auto& p = vehicle_.getPose();
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
