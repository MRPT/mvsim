/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/topography/conversions.h>
#include <mrpt/version.h>
#include <mvsim/Sensors/GNSS.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

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

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues_);

	// Pass params to the template obj:
	obs_model_.sensorLabel = name_;

	// Init ENU covariance:
	auto& C = obs_model_.covariance_enu.emplace();
	const double var_xy = mrpt::square(horizontal_std_noise_);
	const double var_z = mrpt::square(vertical_std_noise_);

	C.setDiagonal(std::vector<double>{var_xy, var_xy, var_z});
}

void GNSS::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	// 1st time?
	if (!gl_sensor_origin_ && viz)
	{
		gl_sensor_origin_ = mrpt::opengl::CSetOfObjects::Create();
#if MRPT_VERSION >= 0x270
		gl_sensor_origin_->castShadows(false);
#endif
		gl_sensor_origin_corner_ = mrpt::opengl::stock_objects::CornerXYZSimple(0.15f);

		gl_sensor_origin_->insert(gl_sensor_origin_corner_);

		gl_sensor_origin_->setVisibility(false);
		viz->get().insert(gl_sensor_origin_);
		SensorBase::RegisterSensorOriginViz(gl_sensor_origin_);
	}

	const mrpt::poses::CPose3D p = vehicle_.getCPose3D() + obs_model_.sensorPose;
	const auto pp = parent()->applyWorldRenderOffset(p);

	if (gl_sensor_origin_) gl_sensor_origin_->setPose(pp);
	if (glCustomVisual_) glCustomVisual_->setPose(pp);
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

	auto outObs = CObservationGPS::Create(obs_model_);

	outObs->timestamp = world_->get_simul_timestamp();
	outObs->sensorLabel = name_;

	// noise:
	const mrpt::math::TPoint3D noise = {
		rng_.drawGaussian1D(0.0, horizontal_std_noise_),
		rng_.drawGaussian1D(0.0, horizontal_std_noise_),
		rng_.drawGaussian1D(0.0, vertical_std_noise_)};

	// Where the GPS sensor is in the world frame:
	const auto& georef = world()->georeferenceOptions();

	const auto worldRotation =
		mrpt::poses::CPose3D::FromYawPitchRoll(georef.world_to_enu_rotation, .0, .0);

	mrpt::poses::CPose3D vehPoseInWorld = vehicle().getCPose3D();

	if (georef.world_is_utm)
	{
		auto posLocal = vehPoseInWorld.translation() - georef.utmRef;

		vehPoseInWorld.x(posLocal.x);
		vehPoseInWorld.y(posLocal.y);
		vehPoseInWorld.z(posLocal.z);
	}

	const mrpt::math::TPoint3D sensorPt =
		(worldRotation + (vehPoseInWorld + outObs->sensorPose)).translation() + noise;

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
	f.HDOP = horizontal_std_noise_ / 5.0;  // approximation

	mrpt::system::TTimeParts tp;
	mrpt::system::timestampToParts(outObs->timestamp, tp);
	f.UTCTime.hour = tp.hour;
	f.UTCTime.minute = tp.minute;
	f.UTCTime.sec = tp.second;
	f.fix_quality = 2;	// DGPS fix

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
