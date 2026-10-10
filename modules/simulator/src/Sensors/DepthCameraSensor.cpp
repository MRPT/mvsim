/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/random.h>
#include <mrpt/version.h>
#include <mrpt/viz/CFrustum.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/stock_objects.h>
#include <mvsim/Sensors/DepthCameraSensor.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <algorithm>
#include <array>

#include "xml_utils.h"

using namespace mvsim;
using namespace rapidxml;

namespace
{
// The depth camera looks along +X, the RGB camera (and OpenGL) along +Z:
mrpt::poses::CPose3D fixedAxisConventionRot()
{
	return mrpt::poses::CPose3D::FromYawPitchRoll(-M_PI / 2, 0.0, -M_PI / 2);
}

// Pixels are converted in blocks of fixed size, which compilers vectorize:
constexpr size_t DEPTH_BLOCK = 16;

struct DepthToRangeParams
{
	float maxRange = 0;
	float invUnits = 0;
	int maxRangeInts = 0;
};

// Depth [m] to range image units, plus noise. Pixels without a return (0) and
// beyond the maximum range are invalid (0):
void depthBlockToRanges(
	const float* __restrict depths, const int16_t* __restrict noise, uint16_t* __restrict ranges,
	const DepthToRangeParams& p)
{
	for (size_t i = 0; i < DEPTH_BLOCK; i++)
	{
		const float depth = depths[i] <= p.maxRange ? depths[i] : 0.0f;
		const int r = static_cast<int>(depth * p.invUnits);
		const int rNoisy = r + noise[i];
		const bool useNoisy = (r != 0) & (rNoisy > 0) & (rNoisy <= p.maxRangeInts);
		ranges[i] = static_cast<uint16_t>(useNoisy ? rNoisy : r);
	}
}
}  // namespace

DepthCameraSensor::DepthCameraSensor(Simulable& parent, const rapidxml::xml_node<char>* root)
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
	sensor_params_.sensorPose = mrpt::poses::CPose3D(0, 0, 0.5, 90.0_deg, 0, 90.0_deg);

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

	// Lens distortion and noise of the RGB image:
	rgbDistortion_ = CameraDistortionOptions();
	rgbDistortion_.declareParams(params, "rgb_");
	params["rgb_clip_max"] = TParamEntry("%f", &rgbClipMax_);
	params["depth_clip_min"] = TParamEntry("%f", &depth_clip_min_);
	params["depth_clip_max"] = TParamEntry("%f", &depth_clip_max_);
	params["depth_resolution"] = TParamEntry("%f", &depth_resolution_);

	params["depth_noise_sigma"] = TParamEntry("%f", &depth_noise_sigma_);
	params["show_3d_pointcloud"] = TParamEntry("%bool", &show_3d_pointcloud_);
	params["publish_ros_depth_image"] = TParamEntry("%bool", &publish_depth_image_);
	params["ros_depth_image_encoding"] = TParamEntry("%s", &ros_depth_image_encoding_);
	params["publish_ros_colored_pointcloud"] = TParamEntry("%bool", &publish_colored_pointcloud_);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues_);

	ASSERTMSG_(
		ros_depth_image_encoding_ == "16UC1" || ros_depth_image_encoding_ == "32FC1",
		"<ros_depth_image_encoding> must be '16UC1' or '32FC1'");

	depthCam.ncols = depth_ncols;
	depthCam.nrows = depth_nrows;

	rgbCam.ncols = rgb_ncols;
	rgbCam.nrows = rgb_nrows;
	rgbDistortion_.applyTo(rgbCam);

	// save sensor label here too:
	sensor_params_.sensorLabel = name_;

	sensor_params_.maxRange = depth_clip_max_;
	sensor_params_.rangeUnits = depth_resolution_;
	depthNoiseSeq_.clear();	 // regenerated with the new parameters
	depthNoiseIdx_ = 0;

	// A single render gives both images if the RGB camera has the depth
	// camera intrinsics and pose, and a clip range that covers the depth one:
	const auto relPoseError =
		(sensor_params_.relativePoseIntensityWRTDepth - fixedAxisConventionRot()).asVectorVal();
	single_render_pass_ =
		sense_rgb_ && sense_depth_ && rgbCam.ncols == depthCam.ncols &&
		rgbCam.nrows == depthCam.nrows && rgbCam.intrinsicParams == depthCam.intrinsicParams &&
		rgbCam.distortion == mrpt::img::DistortionModel::none && relPoseError.norm() < 1e-6 &&
		rgbClipMin_ == depth_clip_min_ && rgbClipMax_ >= depth_clip_max_;
}

void DepthCameraSensor::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::viz::Scene>& viz,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::viz::Scene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	mrpt::viz::CSetOfObjects::Ptr glVizSensors;
	if (viz)
	{
		glVizSensors = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(
			viz->get().getByName("group_sensors_viz"));
		if (!glVizSensors) return;	// may happen during shutdown
	}

	// 1st time?
	if (!gl_obs_ && glVizSensors)
	{
		gl_obs_ = mrpt::viz::CPointCloudColoured::Create();
		gl_obs_->setPointSize(2.0f);
		gl_obs_->setLocalRepresentativePoint(sensor_params_.sensorPose.translation());
		glVizSensors->insert(gl_obs_);
	}

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
	if (!gl_sensor_fov_ && viz)
	{
		gl_sensor_fov_ = mrpt::viz::CSetOfObjects::Create();
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
					gl_sensor_frustum_ = mrpt::viz::CSetOfObjects::Create();

					const float frustumScale = 0.4e-3;
					auto frustum =
						mrpt::viz::CFrustum::Create(last_obs2gui_->cameraParams, frustumScale);

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

	const auto pp = parent()->applyWorldRenderOffset(p);

	if (gl_obs_)
	{
		gl_obs_->setPose(pp);
	}
	if (gl_sensor_fov_)
	{
		gl_sensor_fov_->setPose(pp);
	}
	if (gl_sensor_origin_)
	{
		gl_sensor_origin_->setPose(pp);
	}

	if (glCustomVisual_)
	{
		glCustomVisual_->setPose(pp + sensor_params_.sensorPose.asTPose());
	}
}

void DepthCameraSensor::simul_pre_timestep([[maybe_unused]] const TSimulContext& context) {}

void DepthCameraSensor::simulateOn3DScene(mrpt::viz::Scene& world3DScene)
{
	using namespace mrpt;  // _deg

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);
		if (!has_to_render_.has_value())
		{
			return;
		}
	}

	auto tleWhole = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGBD");

	auto tle1 = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGBD.acqGuiMtx");

	tle1.stop();

	if (glCustomVisual_)
	{
		glCustomVisual_->setVisibility(false);
	}

	// Start making a copy of the pattern observation:
	auto curObsPtr = mrpt::obs::CObservation3DRangeScan::Create(sensor_params_);
	auto& curObs = *curObsPtr;

	// Set timestamp:
	curObs.timestamp = world_->get_simul_timestamp();

	// Create FBO on first use, now that we are here at the GUI / OpenGL thread.
	if (!fbo_renderer_rgb_ && sense_rgb_)
	{
		auto tle2 =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGBD.createFBO");

		mrpt::opengl::CFBORender::Parameters p;
		p.width = sensor_params_.cameraParamsIntensity.ncols;
		p.height = sensor_params_.cameraParamsIntensity.nrows;
		p.create_EGL_context = world()->sensor_has_to_create_egl_context();

		fbo_renderer_rgb_ = std::make_shared<mrpt::opengl::CFBORender>(p);
		rgbDistortion_.applyTo(*fbo_renderer_rgb_, sensor_params_.cameraParamsIntensity);
	}

	if (!fbo_renderer_depth_ && sense_depth_ && !single_render_pass_)
	{
		auto tle2 =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGBD.createFBO");

		mrpt::opengl::CFBORender::Parameters p;
		p.width = sensor_params_.cameraParams.ncols;
		p.height = sensor_params_.cameraParams.nrows;
		p.create_EGL_context = world()->sensor_has_to_create_egl_context();

		fbo_renderer_depth_ = std::make_shared<mrpt::opengl::CFBORender>(p);
	}

	auto viewport = world3DScene.getViewport();

	if (fbo_renderer_depth_)
	{
		if (!fbo_renderer_depth_->hasCameraOverride())
			fbo_renderer_depth_->setCamera(mrpt::viz::CCamera());
	}
	if (fbo_renderer_rgb_)
	{
		if (!fbo_renderer_rgb_->hasCameraOverride())
			fbo_renderer_rgb_->setCamera(mrpt::viz::CCamera());
	}

	// ----------------------------------------------------------
	// RGB first with its camera intrinsics & clip distances
	// ----------------------------------------------------------

	// RGB camera pose:
	//   vehicle (+) relativePoseOnVehicle (+) relativePoseIntensityWRTDepth
	//
	// Note: relativePoseOnVehicle should be (y,p,r)=(-90deg,0,-90deg) to make
	// the camera to look forward:

	const auto vehiclePose = mrpt::poses::CPose3D(vehicle_.getPose());

	const auto depthSensorPose = vehiclePose + curObs.sensorPose + fixedAxisConventionRot();

	const auto rgbSensorPose =
		vehiclePose + curObs.sensorPose + curObs.relativePoseIntensityWRTDepth;

	if (fbo_renderer_rgb_)
	{
		auto tle2 =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGBD.renderRGB");

		auto& camRGB = fbo_renderer_rgb_->getCameraOverride();
		camRGB.set6DOFMode(true);
		camRGB.setProjectiveFromPinhole(curObs.cameraParamsIntensity);
		camRGB.setPose(world()->applyWorldRenderOffset(rgbSensorPose));

		// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
		viewport->setViewportClipDistances(rgbClipMin_, rgbClipMax_);

		{
			// Fewer shadow cascades than the GUI view, for speed:
			const ViewportShadowSettingsGuard shadowGuard(*viewport);
			viewport->lightParameters().shadow_cascades =
				static_cast<uint8_t>(world()->sensor_shadow_cascades());

			if (single_render_pass_)
			{
				fbo_renderer_rgb_->render_RGBD(world3DScene, curObs.intensityImage, depthImage_);
			}
			else
			{
				fbo_renderer_rgb_->render_RGB(world3DScene, curObs.intensityImage);
			}
		}

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
		auto tle2 = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGBD.renderD");

		auto& camDepth = fbo_renderer_depth_->getCameraOverride();
		camDepth.setProjectiveFromPinhole(curObs.cameraParams);

		// Camera pose: vehicle + relativePoseOnVehicle:
		// Note: relativePoseOnVehicle should be (y,p,r)=(90deg,0,90deg) to make
		// the camera to look forward:
		camDepth.set6DOFMode(true);
		camDepth.setPose(world()->applyWorldRenderOffset(depthSensorPose));

		// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
		viewport->setViewportClipDistances(depth_clip_min_, depth_clip_max_);

		{
			// Shadows do not affect depth images:
			const ViewportShadowSettingsGuard shadowGuard(*viewport);
			viewport->enableShadowCasting(false);

			fbo_renderer_depth_->render_depth(world3DScene, depthImage_);
		}
	}

	if (fbo_renderer_depth_ || (single_render_pass_ && fbo_renderer_rgb_))
	{
		auto tle2cnv =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGBD.convertD");

		curObs.hasRangeImage = true;
		curObs.range_is_depth = true;
		depthToRangeImage(curObs);
	}
	else
	{
		curObs.hasRangeImage = false;
	}

	// Store generated obs:
	{
		auto tle3 =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGBD.acqObsMtx");

		std::lock_guard<std::mutex> csl(last_obs_cs_);
		last_obs_ = std::move(curObsPtr);
		last_obs2gui_ = last_obs_;
	}

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);

		auto tlePub = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGBD.report");

		SensorBase::reportNewObservation(last_obs_, *has_to_render_);

		tlePub.stop();

		if (glCustomVisual_)
		{
			glCustomVisual_->setVisibility(true);
		}

		gui_uptodate_ = false;
		has_to_render_.reset();
	}
}

void DepthCameraSensor::depthToRangeImage(mrpt::obs::CObservation3DRangeScan& obs)
{
	// Precomputed random noise sequence (all zeros without noise):
	constexpr size_t noiseLen = 489 * DEPTH_BLOCK;
	if (depthNoiseSeq_.empty())
	{
		mrpt::random::CRandomGenerator rng;
		depthNoiseSeq_.resize(noiseLen, 0);
		if (depth_noise_sigma_ > 0)
		{
			for (auto& n : depthNoiseSeq_)
			{
				n = static_cast<int16_t>(
					mrpt::round(rng.drawGaussian1D(0.0, depth_noise_sigma_) / obs.rangeUnits));
			}
		}
	}

	obs.rangeImage_setSize(depthImage_.rows(), depthImage_.cols());

	const float* depths = depthImage_.data();
	uint16_t* ranges = obs.rangeImage.data();
	const size_t N = obs.rangeImage.size();

	DepthToRangeParams p;
	p.maxRange = obs.maxRange;
	p.invUnits = 1.0f / obs.rangeUnits;
	p.maxRangeInts = static_cast<int>(p.maxRange * p.invUnits);

	size_t i = 0;
	for (; i + DEPTH_BLOCK <= N; i += DEPTH_BLOCK)
	{
		depthBlockToRanges(depths + i, depthNoiseSeq_.data() + depthNoiseIdx_, ranges + i, p);
		depthNoiseIdx_ = (depthNoiseIdx_ + DEPTH_BLOCK) % noiseLen;
	}
	if (i < N)
	{
		// Last, incomplete block:
		std::array<float, DEPTH_BLOCK> lastDepths{};
		std::array<uint16_t, DEPTH_BLOCK> lastRanges{};
		std::copy(depths + i, depths + N, lastDepths.begin());
		depthBlockToRanges(lastDepths.data(), depthNoiseSeq_.data(), lastRanges.data(), p);
		std::copy_n(lastRanges.begin(), N - i, ranges + i);
	}

	// Valid depths can only be below one range unit with a near clip distance
	// below it. They must not become invalid (0):
	if (depth_clip_min_ < obs.rangeUnits)
	{
		for (size_t k = 0; k < N; k++)
		{
			if (ranges[k] == 0 && depths[k] > 0 && depths[k] <= p.maxRange)
			{
				ranges[k] = 1;
			}
		}
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
	// Keep sensor global pose up-to-date:
	const auto& p = vehicle_.getPose();
	const auto globalSensorPose = p + sensor_params_.sensorPose.asTPose();
	Simulable::setPose(globalSensorPose, false /*do not notify*/);
}

void DepthCameraSensor::notifySimulableSetPose(const mrpt::math::TPose3D& newPose)
{
	// The editor has moved the sensor in global coordinates.
	// Convert back to local:
	const auto& p = vehicle_.getPose();
	sensor_params_.sensorPose = mrpt::poses::CPose3D(newPose - p);
}

void DepthCameraSensor::freeOpenGLResources()
{
	fbo_renderer_depth_.reset();
	fbo_renderer_rgb_.reset();
}
