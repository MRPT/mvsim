/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/Lie/SO.h>
#include <mrpt/version.h>
#include <mvsim/Sensors/IMU.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include "xml_utils.h"

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
// #include <mvsim/mvsim-msgs/ObservationXXX.pb.h>
#endif

using namespace mvsim;
using namespace rapidxml;

IMU::IMU(Simulable& parent, const rapidxml::xml_node<char>* root) : SensorBase(parent)
{
	IMU::loadConfigFrom(root);
}

IMU::~IMU() {}

void IMU::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	SensorBase::loadConfigFrom(root);
	SensorBase::make_sure_we_have_a_name("imu");

	TParameterDefinitions params;
	params["pose"] = TParamEntry("%pose2d_ptr3d", &obs_model_.sensorPose);
	params["pose_3d"] = TParamEntry("%pose3d", &obs_model_.sensorPose);
	params["sensor_period"] = TParamEntry("%lf", &sensor_period_);
	params["angular_velocity_std_noise"] = TParamEntry("%lf", &angularVelocityStdNoise_);
	params["orientation_std_noise"] = TParamEntry("%lf", &orientationStdNoise_);
	params["linear_acceleration_std_noise"] = TParamEntry("%lf", &linearAccelerationStdNoise_);
	params["measure_orientation"] = TParamEntry("%bool", &measure_orientation_);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues_);

	// Pass params to the template obj:
	obs_model_.sensorLabel = name_;
}

void IMU::internalGuiUpdate(
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

	if (gl_sensor_origin_)
	{
		gl_sensor_origin_->setPose(pp);
	}
	if (glCustomVisual_)
	{
		glCustomVisual_->setPose(pp);
	}
}

void IMU::simul_pre_timestep([[maybe_unused]] const TSimulContext& context) {}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void IMU::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);

	if (SensorBase::should_simulate_sensor(context))
	{
		internal_simulate_imu(context);
	}
	// Keep sensor global pose up-to-date:
	const auto& p = vehicle_.getPose();
	const auto globalSensorPose = p + obs_model_.sensorPose.asTPose();
	Simulable::setPose(globalSensorPose, false /*do not notify*/);
}

void IMU::internal_simulate_imu(const TSimulContext& context)
{
	using mrpt::obs::CObservationIMU;

	auto tle = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.IMU");

	auto outObs = CObservationIMU::Create(obs_model_);

	outObs->timestamp = world_->get_simul_timestamp();
	outObs->sensorLabel = name_;

	// angular velocity:
	mrpt::math::TVector3D w(0.0, 0.0, vehicle_.getTwist().omega);
	rng_.drawGaussian1DVector(w, 0.0, angularVelocityStdNoise_);

	outObs->set(mrpt::obs::IMU_WX, w.x);
	outObs->set(mrpt::obs::IMU_WY, w.y);
	outObs->set(mrpt::obs::IMU_WZ, w.z);

	// linear acceleration:
	const auto g = mrpt::math::TVector3D(.0, .0, -world_->get_gravity());

	mrpt::math::TVector3D linAccNoise(0, 0, 0);
	rng_.drawGaussian1DVector(linAccNoise, 0.0, linearAccelerationStdNoise_);

	const mrpt::math::TVector3D linAccLocal =
		vehicle_.getPose().inverseComposePoint(vehicle_.getLinearAcceleration() + g) + linAccNoise;

	outObs->set(mrpt::obs::IMU_X_ACC, linAccLocal.x);
	outObs->set(mrpt::obs::IMU_Y_ACC, linAccLocal.y);
	outObs->set(mrpt::obs::IMU_Z_ACC, linAccLocal.z);

	// Orientation:
	if (measure_orientation_)
	{
		// If using geo-referenced coordinates, get world to ENU rotation:
		const auto& w2enu = parent()->georeferenceOptions().world_to_enu_rotation;

		// Get vehicle pose on the world:
		const auto& p = vehicle_.getCPose3D();

		// compose with sensor pose:
		auto& rng = mrpt::random::getRandomGenerator();
		mrpt::math::CVectorFixed<double, 3> oriNoiseVec;
		rng.drawGaussian1DVector(oriNoiseVec, .0, orientationStdNoise_);

		const auto oriNoiseRot = mrpt::poses::Lie::SO<3>::exp(oriNoiseVec);

		const mrpt::poses::CPose3D sensorPoseEnu =
			mrpt::poses::CPose3D::FromYawPitchRoll(w2enu, .0, .0) + (p + obs_model_.sensorPose) +
			mrpt::poses::CPose3D::FromRotationAndTranslation(
				oriNoiseRot, mrpt::math::TVector3D(0, 0, 0));

		mrpt::math::CQuaternionDouble q;
		sensorPoseEnu.getAsQuaternion(q);

		outObs->set(mrpt::obs::IMU_ORI_QUAT_W, q.r());
		outObs->set(mrpt::obs::IMU_ORI_QUAT_X, q.x());
		outObs->set(mrpt::obs::IMU_ORI_QUAT_Y, q.y());
		outObs->set(mrpt::obs::IMU_ORI_QUAT_Z, q.z());
	}

	// Save:
	{
		std::lock_guard<std::mutex> csl(last_obs_cs_);
		last_obs_ = std::move(outObs);
	}

	// publish as generic Protobuf (mrpt serialized) object:
	SensorBase::reportNewObservation(last_obs_, context);
}

void IMU::notifySimulableSetPose(const mrpt::math::TPose3D&)
{
	// The editor has moved the sensor in global coordinates.
	// Convert back to local:
	// const auto& p = vehicle_.getPose();
	// sensor_params_.sensorPose = mrpt::poses::CPose3D(newPose - p);
}

void IMU::registerOnServer(mvsim::Client& c)
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
