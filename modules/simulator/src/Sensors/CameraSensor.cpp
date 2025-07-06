/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
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
#include <mvsim/Sensors/CameraSensor.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace rapidxml;

CameraSensor::CameraSensor(Simulable& parent, const rapidxml::xml_node<char>* root)
	: SensorBase(parent)
{
	this->loadConfigFrom(root);
}

CameraSensor::~CameraSensor() {}

void CameraSensor::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	gui_uptodate_ = false;

	SensorBase::loadConfigFrom(root);
	SensorBase::make_sure_we_have_a_name("camera");

	fbo_renderer_rgb_.reset();

	using namespace mrpt;  // _deg
	sensor_params_.cameraPose = mrpt::poses::CPose3D(0, 0, 0.5, 90.0_deg, 0, 90.0_deg);

	// Default values:
	{
		auto& c = sensor_params_.cameraParams;
		c.ncols = 640;
		c.nrows = 480;
		c.cx(c.ncols / 2);
		c.cy(c.nrows / 2);
		c.fx(500);
		c.fy(500);
	}

	// Other scalar params:
	TParameterDefinitions params;
	params["pose_3d"] = TParamEntry("%pose3d", &sensor_params_.cameraPose);

	auto& rgbCam = sensor_params_.cameraParams;
	params["cx"] = TParamEntry("%lf", &rgbCam.intrinsicParams(0, 2));
	params["cy"] = TParamEntry("%lf", &rgbCam.intrinsicParams(1, 2));
	params["fx"] = TParamEntry("%lf", &rgbCam.intrinsicParams(0, 0));
	params["fy"] = TParamEntry("%lf", &rgbCam.intrinsicParams(1, 1));

	unsigned int rgb_ncols = rgbCam.ncols, rgb_nrows = rgbCam.nrows;
	params["ncols"] = TParamEntry("%u", &rgb_ncols);
	params["nrows"] = TParamEntry("%u", &rgb_nrows);

	params["clip_min"] = TParamEntry("%f", &rgbClipMin_);
	params["clip_max"] = TParamEntry("%f", &rgbClipMax_);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues_);

	rgbCam.ncols = rgb_ncols;
	rgbCam.nrows = rgb_nrows;

	// save sensor label here too:
	sensor_params_.sensorLabel = name_;
}

void CameraSensor::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
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
			if (last_obs2gui_)
			{
				gl_sensor_origin_corner_->setPose(last_obs2gui_->sensorPose());

				if (!gl_sensor_frustum_)
				{
					gl_sensor_frustum_ = mrpt::opengl::CSetOfObjects::Create();

					const float frustumScale = 0.4e-3;
					auto frustum =
						mrpt::opengl::CFrustum::Create(last_obs2gui_->cameraParams, frustumScale);

					gl_sensor_frustum_->insert(frustum);
					gl_sensor_fov_->insert(gl_sensor_frustum_);
				}

				using namespace mrpt;  // _deg

				gl_sensor_frustum_->setPose(
					last_obs2gui_->cameraPose +
					(-mrpt::poses::CPose3D::FromYawPitchRoll(-90.0_deg, 0.0_deg, -90.0_deg)));

				last_obs2gui_.reset();
			}
		}
		gui_uptodate_ = true;
	}

	// Move with vehicle:
	const auto& p = vehicle_.getPose();
	const auto pp = parent()->applyWorldRenderOffset(p);

	gl_sensor_fov_->setPose(pp);
	gl_sensor_origin_->setPose(pp);

	const auto globalSensorPose = pp + sensor_params_.cameraPose.asTPose();
	if (glCustomVisual_) glCustomVisual_->setPose(globalSensorPose);
}

void CameraSensor::simul_pre_timestep([[maybe_unused]] const TSimulContext& context) {}

void CameraSensor::simulateOn3DScene(mrpt::opengl::COpenGLScene& world3DScene)
{
	using namespace mrpt;  // _deg

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);
		if (!has_to_render_.has_value()) return;
	}

	auto tleWhole = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGB");

	if (glCustomVisual_) glCustomVisual_->setVisibility(false);

	// Start making a copy of the pattern observation:
	auto curObs = mrpt::obs::CObservationImage::Create(sensor_params_);

	// Set timestamp:
	curObs->timestamp = world_->get_simul_timestamp();

	// Create FBO on first use, now that we are here at the GUI / OpenGL thread.
	if (!fbo_renderer_rgb_)
	{
		mrpt::opengl::CFBORender::Parameters p;
		p.width = sensor_params_.cameraParams.ncols;
		p.height = sensor_params_.cameraParams.nrows;
		p.create_EGL_context = world()->sensor_has_to_create_egl_context();

		fbo_renderer_rgb_ = std::make_shared<mrpt::opengl::CFBORender>(p);
	}

	auto viewport = world3DScene.getViewport();

	auto& cam = fbo_renderer_rgb_->getCamera(world3DScene);

	const auto fixedAxisConventionRot =
		mrpt::poses::CPose3D(0, 0, 0, -90.0_deg, 0.0_deg, -90.0_deg);

	// ----------------------------------------------------------
	// RGB with its camera intrinsics & clip distances
	// ----------------------------------------------------------
	cam.set6DOFMode(true);
	cam.setProjectiveFromPinhole(curObs->cameraParams);

	// RGB camera pose:
	//   vehicle (+) relativePoseOnVehicle (+) relativePoseIntensityWRTDepth
	//
	// Note: relativePoseOnVehicle should be (y,p,r)=(-90deg,0,-90deg) to make
	// the camera to look forward:

	const auto vehiclePose = mrpt::poses::CPose3D(vehicle_.getPose());
	const auto rgbSensorPose = vehiclePose + curObs->cameraPose;

	cam.setPose(world()->applyWorldRenderOffset(rgbSensorPose));

	ASSERT_(fbo_renderer_rgb_);

	auto tle2 = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.RGB.render");

	// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
	viewport->setViewportClipDistances(rgbClipMin_, rgbClipMax_);

	fbo_renderer_rgb_->render_RGB(world3DScene, curObs->image);

	tle2.stop();

	// Store generated obs:
	{
		std::lock_guard<std::mutex> csl(last_obs_cs_);
		last_obs_ = std::move(curObs);
		last_obs2gui_ = last_obs_;
	}

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);
		SensorBase::reportNewObservation(last_obs_, *has_to_render_);

		if (glCustomVisual_) glCustomVisual_->setVisibility(true);

		gui_uptodate_ = false;
		has_to_render_.reset();
	}
}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void CameraSensor::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);
	if (SensorBase::should_simulate_sensor(context))
	{
		has_to_render_ = context;
		world_->mark_as_pending_running_sensors_on_3D_scene();
	}

	// Keep sensor global pose up-to-date:
	const auto& p = vehicle_.getPose();
	const auto globalSensorPose = p + sensor_params_.cameraPose.asTPose();
	Simulable::setPose(globalSensorPose, false /*do not notify*/);
}

void CameraSensor::notifySimulableSetPose(const mrpt::math::TPose3D& newPose)
{
	// The editor has moved the sensor in global coordinates.
	// Convert back to local:
	const auto& p = vehicle_.getPose();
	sensor_params_.cameraPose = mrpt::poses::CPose3D(newPose - p);
}

void CameraSensor::freeOpenGLResources() { fbo_renderer_rgb_.reset(); }
