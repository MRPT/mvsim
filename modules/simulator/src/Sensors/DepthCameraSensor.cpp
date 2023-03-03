/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/random.h>
#include <mrpt/version.h>
#include <mvsim/Sensors/DepthCameraSensor.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace rapidxml;

DepthCameraSensor::DepthCameraSensor(
	Simulable& parent, const rapidxml::xml_node<char>* root)
	: SensorBase(parent)
{
	DepthCameraSensor::loadConfigFrom(root);
}

DepthCameraSensor::~DepthCameraSensor() {}

void DepthCameraSensor::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	gui_uptodate_ = false;

	SensorBase::loadConfigFrom(root);
	SensorBase::make_sure_we_have_a_name("camera");

	fbo_renderer_depth_.reset();
	fbo_renderer_rgb_.reset();

	using namespace mrpt;  // _deg
	sensor_params_.sensorPose =
		mrpt::poses::CPose3D(0, 0, 0.5, 90.0_deg, 0, 90.0_deg);

	// Default values:
	{
		auto& c = sensor_params_.cameraParamsIntensity;
		c.ncols = 640;
		c.nrows = 480;
		c.cx(c.ncols / 2);
		c.cy(c.nrows / 2);
		c.fx(500);
		c.fy(500);
	}
	sensor_params_.cameraParams = sensor_params_.cameraParamsIntensity;

	// Other scalar params:
	TParameterDefinitions params;
	params["pose_3d"] = TParamEntry("%pose3d", &sensor_params_.sensorPose);
	params["relativePoseIntensityWRTDepth"] =
		TParamEntry("%pose3d", &sensor_params_.relativePoseIntensityWRTDepth);

	params["sense_depth"] = TParamEntry("%bool", &sense_depth_);
	params["sense_rgb"] = TParamEntry("%bool", &sense_rgb_);

	auto& depthCam = sensor_params_.cameraParams;
	params["depth_cx"] = TParamEntry("%lf", &depthCam.intrinsicParams(0, 2));
	params["depth_cy"] = TParamEntry("%lf", &depthCam.intrinsicParams(1, 2));
	params["depth_fx"] = TParamEntry("%lf", &depthCam.intrinsicParams(0, 0));
	params["depth_fy"] = TParamEntry("%lf", &depthCam.intrinsicParams(1, 1));

	unsigned int depth_ncols = depthCam.ncols, depth_nrows = depthCam.nrows;
	params["depth_ncols"] = TParamEntry("%u", &depth_ncols);
	params["depth_nrows"] = TParamEntry("%u", &depth_nrows);

	auto& rgbCam = sensor_params_.cameraParamsIntensity;
	params["rgb_cx"] = TParamEntry("%lf", &rgbCam.intrinsicParams(0, 2));
	params["rgb_cy"] = TParamEntry("%lf", &rgbCam.intrinsicParams(1, 2));
	params["rgb_fx"] = TParamEntry("%lf", &rgbCam.intrinsicParams(0, 0));
	params["rgb_fy"] = TParamEntry("%lf", &rgbCam.intrinsicParams(1, 1));

	unsigned int rgb_ncols = depthCam.ncols, rgb_nrows = depthCam.nrows;
	params["rgb_ncols"] = TParamEntry("%u", &rgb_ncols);
	params["rgb_nrows"] = TParamEntry("%u", &rgb_nrows);

	params["rgb_clip_min"] = TParamEntry("%f", &rgbClipMin_);
	params["rgb_clip_max"] = TParamEntry("%f", &rgbClipMax_);
	params["depth_clip_min"] = TParamEntry("%f", &depth_clip_min_);
	params["depth_clip_max"] = TParamEntry("%f", &depth_clip_max_);
	params["depth_resolution"] = TParamEntry("%f", &depth_resolution_);

	params["depth_noise_sigma"] = TParamEntry("%f", &depth_noise_sigma_);
	params["show_3d_pointcloud"] = TParamEntry("%bool", &show_3d_pointcloud_);

	MRPT_TODO("REMOVE??");
	params["ambient_light"] = TParamEntry("%f", &ambient_light_);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues_);

	depthCam.ncols = depth_ncols;
	depthCam.nrows = depth_nrows;

	rgbCam.ncols = rgb_ncols;
	rgbCam.nrows = rgb_nrows;

	// save sensor label here too:
	sensor_params_.sensorLabel = name_;

	sensor_params_.maxRange = depth_clip_max_;
	sensor_params_.rangeUnits = depth_resolution_;
}

void DepthCameraSensor::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::opengl::COpenGLScene>&
		physical,
	[[maybe_unused]] bool childrenOnly)
{
	mrpt::opengl::CSetOfObjects::Ptr glVizSensors;
	if (viz)
	{
		glVizSensors = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
			viz->get().getByName("group_sensors_viz"));
		if (!glVizSensors) return;	// may happen during shutdown
	}

	// 1st time?
	if (!gl_obs_ && glVizSensors)
	{
		gl_obs_ = mrpt::opengl::CPointCloudColoured::Create();
		gl_obs_->setPointSize(2.0f);
		gl_obs_->setLocalRepresentativePoint(
			sensor_params_.sensorPose.translation());
		glVizSensors->insert(gl_obs_);
	}

	if (!gl_sensor_origin_ && viz)
	{
		gl_sensor_origin_ = mrpt::opengl::CSetOfObjects::Create();
		gl_sensor_origin_corner_ =
			mrpt::opengl::stock_objects::CornerXYZSimple(0.15f);

		gl_sensor_origin_->insert(gl_sensor_origin_corner_);

		gl_sensor_origin_->setVisibility(false);
		viz->get().insert(gl_sensor_origin_);
		SensorBase::RegisterSensorOriginViz(gl_sensor_origin_);
	}
	if (!gl_sensor_fov_ && viz)
	{
		gl_sensor_fov_ = mrpt::opengl::CSetOfObjects::Create();
		gl_sensor_fov_->setVisibility(false);
		viz->get().insert(gl_sensor_fov_);
		SensorBase::RegisterSensorFOVViz(gl_sensor_fov_);
	}

	if (!gui_uptodate_)
	{
		{
			std::lock_guard<std::mutex> csl(last_obs_cs_);
			if (last_obs2gui_ && glVizSensors->isVisible())
			{
				if (show_3d_pointcloud_)
				{
					mrpt::obs::T3DPointsProjectionParams pp;
					pp.takeIntoAccountSensorPoseOnRobot = true;
					last_obs2gui_->unprojectInto(*gl_obs_, pp);
					// gl_obs_->recolorizeByCoordinate() ...??
				}

				gl_sensor_origin_corner_->setPose(last_obs2gui_->sensorPose);

				if (!gl_sensor_frustum_)
				{
					gl_sensor_frustum_ = mrpt::opengl::CSetOfObjects::Create();

					const float frustumScale = 0.4e-3;
					auto frustum = mrpt::opengl::CFrustum::Create(
						last_obs2gui_->cameraParams, frustumScale);

					gl_sensor_frustum_->insert(frustum);
					gl_sensor_fov_->insert(gl_sensor_frustum_);
				}

				gl_sensor_frustum_->setPose(last_obs2gui_->sensorPose);

				last_obs2gui_.reset();
			}
		}
		gui_uptodate_ = true;
	}

	// Move with vehicle:
	const auto& p = vehicle_.getPose();

	if (gl_obs_) gl_obs_->setPose(p);
	if (gl_sensor_fov_) gl_sensor_fov_->setPose(p);
	if (gl_sensor_origin_) gl_sensor_origin_->setPose(p);

	if (glCustomVisual_)
		glCustomVisual_->setPose(p + sensor_params_.sensorPose.asTPose());
}

void DepthCameraSensor::simul_pre_timestep(
	[[maybe_unused]] const TSimulContext& context)
{
}

void DepthCameraSensor::simulateOn3DScene(
	mrpt::opengl::COpenGLScene& world3DScene)
{
	using namespace mrpt;  // _deg

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);
		if (!has_to_render_.has_value()) return;
	}

	auto tleWhole =
		mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGBD");

	auto tle1 = mrpt::system::CTimeLoggerEntry(
		world_->getTimeLogger(), "sensor.RGBD.acqGuiMtx");

	tle1.stop();

	if (glCustomVisual_) glCustomVisual_->setVisibility(false);

	// Start making a copy of the pattern observation:
	auto curObsPtr = mrpt::obs::CObservation3DRangeScan::Create(sensor_params_);
	auto& curObs = *curObsPtr;

	// Set timestamp:
	curObs.timestamp = world_->get_simul_timestamp();

	// Create FBO on first use, now that we are here at the GUI / OpenGL thread.
	if (!fbo_renderer_rgb_ && sense_rgb_)
	{
		auto tle2 = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.RGBD.createFBO");

		mrpt::opengl::CFBORender::Parameters p;
		p.width = sensor_params_.cameraParamsIntensity.ncols;
		p.height = sensor_params_.cameraParamsIntensity.nrows;
		p.create_EGL_context = world()->sensor_has_to_create_egl_context();

		fbo_renderer_rgb_ = std::make_shared<mrpt::opengl::CFBORender>(p);
	}

	if (!fbo_renderer_depth_ && sense_depth_)
	{
		auto tle2 = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.RGBD.createFBO");

		mrpt::opengl::CFBORender::Parameters p;
		p.width = sensor_params_.cameraParams.ncols;
		p.height = sensor_params_.cameraParams.nrows;
		p.create_EGL_context = world()->sensor_has_to_create_egl_context();

		fbo_renderer_depth_ = std::make_shared<mrpt::opengl::CFBORender>(p);
	}

	auto viewport = world3DScene.getViewport();

	auto* camDepth = fbo_renderer_depth_
						 ? &fbo_renderer_depth_->getCamera(world3DScene)
						 : nullptr;
	auto* camRGB = fbo_renderer_rgb_
					   ? &fbo_renderer_rgb_->getCamera(world3DScene)
					   : nullptr;

	const auto fixedAxisConventionRot =
		mrpt::poses::CPose3D(0, 0, 0, -90.0_deg, 0.0_deg, -90.0_deg);

	// ----------------------------------------------------------
	// RGB first with its camera intrinsics & clip distances
	// ----------------------------------------------------------

	// RGB camera pose:
	//   vehicle (+) relativePoseOnVehicle (+) relativePoseIntensityWRTDepth
	//
	// Note: relativePoseOnVehicle should be (y,p,r)=(-90deg,0,-90deg) to make
	// the camera to look forward:

	const auto vehiclePose = mrpt::poses::CPose3D(vehicle_.getPose());

	const auto depthSensorPose =
		vehiclePose + curObs.sensorPose + fixedAxisConventionRot;

	const auto rgbSensorPose =
		vehiclePose + curObs.sensorPose + curObs.relativePoseIntensityWRTDepth;

	if (fbo_renderer_rgb_)
	{
		auto tle2 = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.RGBD.renderRGB");

		camRGB->set6DOFMode(true);
		camRGB->setProjectiveFromPinhole(curObs.cameraParamsIntensity);
		camRGB->setPose(rgbSensorPose);

		// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
		viewport->setViewportClipDistances(rgbClipMin_, rgbClipMax_);
		viewport->lightParameters().ambient = ambient_light_;

		fbo_renderer_rgb_->render_RGB(world3DScene, curObs.intensityImage);

		curObs.hasIntensityImage = true;
	}
	else
	{
		curObs.hasIntensityImage = false;
	}

	// ----------------------------------------------------------
	// DEPTH camera next
	// ----------------------------------------------------------
	if (fbo_renderer_depth_)
	{
		auto tle2 = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.RGBD.renderD");

		camDepth->setProjectiveFromPinhole(curObs.cameraParams);

		// Camera pose: vehicle + relativePoseOnVehicle:
		// Note: relativePoseOnVehicle should be (y,p,r)=(90deg,0,90deg) to make
		// the camera to look forward:
		camDepth->set6DOFMode(true);
		camDepth->setPose(depthSensorPose);

		// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
		viewport->setViewportClipDistances(depth_clip_min_, depth_clip_max_);

		auto tle2c = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.RGBD.renderD_core");

		fbo_renderer_depth_->render_depth(world3DScene, depthImage_);

		tle2c.stop();

		// Convert depth image:
		curObs.hasRangeImage = true;
		curObs.range_is_depth = true;

		auto tle2cnv = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.RGBD.renderD_cast");

		// float -> uint16_t with "curObs.rangeUnits" units:
		curObs.rangeImage_setSize(depthImage_.rows(), depthImage_.cols());
		curObs.rangeImage = (depthImage_.asEigen().cwiseMin(curObs.maxRange) /
							 curObs.rangeUnits)
								.cast<uint16_t>();

		tle2cnv.stop();

		// Add random noise:
		if (depth_noise_sigma_ > 0)
		{
			// Each thread must create its own rng:
			thread_local mrpt::random::CRandomGenerator rng;
			thread_local std::vector<int16_t> noiseSeq;
			thread_local size_t noiseIdx = 0;
			constexpr size_t noiseLen = 7823;  // prime
			if (noiseSeq.empty())
			{
				noiseSeq.reserve(noiseLen);
				for (size_t i = 0; i < noiseLen; i++)
				{
					noiseSeq.push_back(static_cast<int16_t>(mrpt::round(
						rng.drawGaussian1D(0.0, depth_noise_sigma_) /
						curObs.rangeUnits)));
				}
			}

			auto tle2noise = mrpt::system::CTimeLoggerEntry(
				world_->getTimeLogger(), "sensor.RGBD.renderD_noise");

			uint16_t* d = curObs.rangeImage.data();
			const size_t N = curObs.rangeImage.size();

			const int16_t maxRangeInts =
				static_cast<int16_t>(curObs.maxRange / curObs.rangeUnits);

			for (size_t i = 0; i < N; i++)
			{
				if (d[i] == 0) continue;  // it was an invalid ray return.

				const int16_t dNoisy =
					static_cast<int16_t>(d[i]) + noiseSeq[noiseIdx++];

				if (noiseIdx >= noiseLen) noiseIdx = 0;

				if (dNoisy > maxRangeInts) continue;

				d[i] = static_cast<uint16_t>(dNoisy);
			}
		}
	}
	else
	{
		curObs.hasRangeImage = false;
	}

	// Store generated obs:
	{
		auto tle3 = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.RGBD.acqObsMtx");

		std::lock_guard<std::mutex> csl(last_obs_cs_);
		last_obs_ = std::move(curObsPtr);
		last_obs2gui_ = last_obs_;
	}

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);

		auto tlePub = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.RGBD.report");

		SensorBase::reportNewObservation(last_obs_, *has_to_render_);

		tlePub.stop();

		if (glCustomVisual_) glCustomVisual_->setVisibility(true);

		gui_uptodate_ = false;
		has_to_render_.reset();
	}
}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void DepthCameraSensor::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);
	if (SensorBase::should_simulate_sensor(context))
	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);
		has_to_render_ = context;
		world_->mark_as_pending_running_sensors_on_3D_scene();
	}
}

void DepthCameraSensor::freeOpenGLResources()
{
	fbo_renderer_depth_.reset();
	fbo_renderer_rgb_.reset();
}
