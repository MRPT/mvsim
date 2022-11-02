/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
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
	m_gui_uptodate = false;

	SensorBase::loadConfigFrom(root);
	SensorBase::make_sure_we_have_a_name("camera");

	m_fbo_renderer_depth.reset();
	m_fbo_renderer_rgb.reset();

	using namespace mrpt;  // _deg
	m_sensor_params.sensorPose =
		mrpt::poses::CPose3D(0, 0, 0.5, 90.0_deg, 0, 90.0_deg);

	// Default values:
	{
		auto& c = m_sensor_params.cameraParamsIntensity;
		c.ncols = 640;
		c.nrows = 480;
		c.cx(c.ncols / 2);
		c.cy(c.nrows / 2);
		c.fx(500);
		c.fy(500);
	}
	m_sensor_params.cameraParams = m_sensor_params.cameraParamsIntensity;

	// Other scalar params:
	TParameterDefinitions params;
	params["pose_3d"] = TParamEntry("%pose3d", &m_sensor_params.sensorPose);
	params["relativePoseIntensityWRTDepth"] =
		TParamEntry("%pose3d", &m_sensor_params.relativePoseIntensityWRTDepth);

	params["sense_depth"] = TParamEntry("%bool", &m_sense_depth);
	params["sense_rgb"] = TParamEntry("%bool", &m_sense_rgb);

	auto& depthCam = m_sensor_params.cameraParams;
	params["depth_cx"] = TParamEntry("%lf", &depthCam.intrinsicParams(0, 2));
	params["depth_cy"] = TParamEntry("%lf", &depthCam.intrinsicParams(1, 2));
	params["depth_fx"] = TParamEntry("%lf", &depthCam.intrinsicParams(0, 0));
	params["depth_fy"] = TParamEntry("%lf", &depthCam.intrinsicParams(1, 1));

	unsigned int depth_ncols = depthCam.ncols, depth_nrows = depthCam.nrows;
	params["depth_ncols"] = TParamEntry("%u", &depth_ncols);
	params["depth_nrows"] = TParamEntry("%u", &depth_nrows);

	auto& rgbCam = m_sensor_params.cameraParamsIntensity;
	params["rgb_cx"] = TParamEntry("%lf", &rgbCam.intrinsicParams(0, 2));
	params["rgb_cy"] = TParamEntry("%lf", &rgbCam.intrinsicParams(1, 2));
	params["rgb_fx"] = TParamEntry("%lf", &rgbCam.intrinsicParams(0, 0));
	params["rgb_fy"] = TParamEntry("%lf", &rgbCam.intrinsicParams(1, 1));

	unsigned int rgb_ncols = depthCam.ncols, rgb_nrows = depthCam.nrows;
	params["rgb_ncols"] = TParamEntry("%u", &rgb_ncols);
	params["rgb_nrows"] = TParamEntry("%u", &rgb_nrows);

	params["rgb_clip_min"] = TParamEntry("%f", &m_rgb_clip_min);
	params["rgb_clip_max"] = TParamEntry("%f", &m_rgb_clip_max);
	params["depth_clip_min"] = TParamEntry("%f", &m_depth_clip_min);
	params["depth_clip_max"] = TParamEntry("%f", &m_depth_clip_max);
	params["depth_resolution"] = TParamEntry("%f", &m_depth_resolution);

	params["depth_noise_sigma"] = TParamEntry("%f", &m_depth_noise_sigma);
	params["show_3d_pointcloud"] = TParamEntry("%bool", &m_show_3d_pointcloud);

	params["ambient_light"] = TParamEntry("%f", &m_ambient_light);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, m_varValues);

	depthCam.ncols = depth_ncols;
	depthCam.nrows = depth_nrows;

	rgbCam.ncols = rgb_ncols;
	rgbCam.nrows = rgb_nrows;

	// save sensor label here too:
	m_sensor_params.sensorLabel = m_name;

	m_sensor_params.maxRange = m_depth_clip_max;
	m_sensor_params.rangeUnits = m_depth_resolution;
}

void DepthCameraSensor::internalGuiUpdate(
	mrpt::opengl::COpenGLScene& viz,
	[[maybe_unused]] mrpt::opengl::COpenGLScene& physical,
	[[maybe_unused]] bool childrenOnly)
{
	auto lck = mrpt::lockHelper(m_gui_mtx);

	auto glVizSensors = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
		viz.getByName("group_sensors_viz"));
	ASSERT_(glVizSensors);

	// 1st time?
	if (!m_gl_obs)
	{
		m_gl_obs = mrpt::opengl::CPointCloudColoured::Create();
		m_gl_obs->setPointSize(2.0f);
		m_gl_obs->setLocalRepresentativePoint(
			m_sensor_params.sensorPose.translation());
		glVizSensors->insert(m_gl_obs);
	}

	if (!m_gl_sensor_origin)
	{
		m_gl_sensor_origin = mrpt::opengl::CSetOfObjects::Create();
		m_gl_sensor_origin_corner =
			mrpt::opengl::stock_objects::CornerXYZSimple(0.15f);

		m_gl_sensor_origin->insert(m_gl_sensor_origin_corner);

		m_gl_sensor_origin->setVisibility(false);
		viz.insert(m_gl_sensor_origin);
		SensorBase::RegisterSensorOriginViz(m_gl_sensor_origin);
	}
	if (!m_gl_sensor_fov)
	{
		m_gl_sensor_fov = mrpt::opengl::CSetOfObjects::Create();
		m_gl_sensor_fov->setVisibility(false);
		viz.insert(m_gl_sensor_fov);
		SensorBase::RegisterSensorFOVViz(m_gl_sensor_fov);
	}

	if (!m_gui_uptodate)
	{
		{
			std::lock_guard<std::mutex> csl(m_last_obs_cs);
			if (m_last_obs2gui && glVizSensors->isVisible())
			{
				if (m_show_3d_pointcloud)
				{
					mrpt::obs::T3DPointsProjectionParams pp;
					pp.takeIntoAccountSensorPoseOnRobot = true;
					m_last_obs2gui->unprojectInto(*m_gl_obs, pp);
					// m_gl_obs->recolorizeByCoordinate() ...??
				}

				m_gl_sensor_origin_corner->setPose(m_last_obs2gui->sensorPose);

				if (!m_gl_sensor_frustum)
				{
					m_gl_sensor_frustum = mrpt::opengl::CSetOfObjects::Create();

					const float frustumScale = 0.4e-3;
					auto frustum = mrpt::opengl::CFrustum::Create(
						m_last_obs2gui->cameraParams, frustumScale);

					m_gl_sensor_frustum->insert(frustum);
					m_gl_sensor_fov->insert(m_gl_sensor_frustum);
				}

				m_gl_sensor_frustum->setPose(m_last_obs2gui->sensorPose);

				m_last_obs2gui.reset();
			}
		}
		m_gui_uptodate = true;
	}

	// Move with vehicle:
	const auto& p = m_vehicle.getPose();

	m_gl_obs->setPose(p);
	m_gl_sensor_fov->setPose(p);
	m_gl_sensor_origin->setPose(p);

	if (m_glCustomVisual)
		m_glCustomVisual->setPose(p + m_sensor_params.sensorPose.asTPose());
}

void DepthCameraSensor::simul_pre_timestep([
	[maybe_unused]] const TSimulContext& context)
{
}

void DepthCameraSensor::simulateOn3DScene(
	mrpt::opengl::COpenGLScene& world3DScene)
{
	using namespace mrpt;  // _deg

	{
		auto lckHasTo = mrpt::lockHelper(m_has_to_render_mtx);
		if (!m_has_to_render.has_value()) return;
	}

	auto tleWhole =
		mrpt::system::CTimeLoggerEntry(m_world->getTimeLogger(), "sensor.RGBD");

	auto tle1 = mrpt::system::CTimeLoggerEntry(
		m_world->getTimeLogger(), "sensor.RGBD.acqGuiMtx");

	auto lck = mrpt::lockHelper(m_gui_mtx);

	tle1.stop();

	if (m_glCustomVisual) m_glCustomVisual->setVisibility(false);

	// Start making a copy of the pattern observation:
	auto curObsPtr =
		mrpt::obs::CObservation3DRangeScan::Create(m_sensor_params);
	auto& curObs = *curObsPtr;

	// Set timestamp:
	curObs.timestamp = m_world->get_simul_timestamp();

	// Create FBO on first use, now that we are here at the GUI / OpenGL thread.
	if (!m_fbo_renderer_rgb && m_sense_rgb)
	{
		auto tle2 = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.RGBD.createFBO");

#if MRPT_VERSION < 0x256
		m_fbo_renderer_rgb = std::make_shared<mrpt::opengl::CFBORender>(
			m_sensor_params.cameraParamsIntensity.ncols,
			m_sensor_params.cameraParamsIntensity.nrows,
			true /* skip GLUT window */);
#else
		mrpt::opengl::CFBORender::Parameters p;
		p.width = m_sensor_params.cameraParamsIntensity.ncols;
		p.height = m_sensor_params.cameraParamsIntensity.nrows;
		p.create_EGL_context = false;  // reuse nanogui context

		m_fbo_renderer_rgb = std::make_shared<mrpt::opengl::CFBORender>(p);
#endif
	}

	if (!m_fbo_renderer_depth && m_sense_depth)
	{
		auto tle2 = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.RGBD.createFBO");

#if MRPT_VERSION < 0x256
		m_fbo_renderer_depth = std::make_shared<mrpt::opengl::CFBORender>(
			m_sensor_params.cameraParams.ncols,
			m_sensor_params.cameraParams.nrows, true /* skip GLUT window */);
#else
		mrpt::opengl::CFBORender::Parameters p;
		p.width = m_sensor_params.cameraParams.ncols;
		p.height = m_sensor_params.cameraParams.nrows;
		p.create_EGL_context = false;  // reuse nanogui context

		m_fbo_renderer_depth = std::make_shared<mrpt::opengl::CFBORender>(p);
#endif
	}

	auto viewport = world3DScene.getViewport();

#if MRPT_VERSION < 0x256
	auto* camDepth = &viewport->getCamera();
	auto* camRGB = &viewport->getCamera();
#else
	auto* camDepth = m_fbo_renderer_depth
						 ? &m_fbo_renderer_depth->getCamera(world3DScene)
						 : nullptr;
	auto* camRGB = m_fbo_renderer_rgb
					   ? &m_fbo_renderer_rgb->getCamera(world3DScene)
					   : nullptr;
#endif

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

	const auto vehiclePose = m_vehicle_pose_at_last_timestamp;

	const auto depthSensorPose =
		vehiclePose + curObs.sensorPose + fixedAxisConventionRot;

	const auto rgbSensorPose =
		vehiclePose + curObs.sensorPose + curObs.relativePoseIntensityWRTDepth;

	if (m_fbo_renderer_rgb)
	{
		auto tle2 = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.RGBD.renderRGB");

		camRGB->set6DOFMode(true);
		camRGB->setProjectiveFromPinhole(curObs.cameraParamsIntensity);
		camRGB->setPose(rgbSensorPose);

		// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
		viewport->setViewportClipDistances(m_rgb_clip_min, m_rgb_clip_max);
		viewport->lightParameters().ambient = {
			m_ambient_light, m_ambient_light, m_ambient_light, 1.0f};

		m_fbo_renderer_rgb->render_RGB(world3DScene, curObs.intensityImage);

		curObs.hasIntensityImage = true;
	}
	else
	{
		curObs.hasIntensityImage = false;
	}

	// ----------------------------------------------------------
	// DEPTH camera next
	// ----------------------------------------------------------
	if (m_fbo_renderer_depth)
	{
		auto tle2 = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.RGBD.renderD");

		camDepth->setProjectiveFromPinhole(curObs.cameraParams);

		// Camera pose: vehicle + relativePoseOnVehicle:
		// Note: relativePoseOnVehicle should be (y,p,r)=(90deg,0,90deg) to make
		// the camera to look forward:
		camDepth->set6DOFMode(true);
		camDepth->setPose(depthSensorPose);

		// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
		viewport->setViewportClipDistances(m_depth_clip_min, m_depth_clip_max);

		auto tle2c = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.RGBD.renderD_core");

		m_fbo_renderer_depth->render_depth(world3DScene, m_depthImage);

		tle2c.stop();

		// Convert depth image:
		curObs.hasRangeImage = true;
		curObs.range_is_depth = true;

		auto tle2cnv = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.RGBD.renderD_cast");

		// float -> uint16_t with "curObs.rangeUnits" units:
		curObs.rangeImage_setSize(m_depthImage.rows(), m_depthImage.cols());
		curObs.rangeImage = (m_depthImage.asEigen().cwiseMin(curObs.maxRange) /
							 curObs.rangeUnits)
								.cast<uint16_t>();

		tle2cnv.stop();

		// Add random noise:
		if (m_depth_noise_sigma > 0)
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
						rng.drawGaussian1D(0.0, m_depth_noise_sigma) /
						curObs.rangeUnits)));
				}
			}

			auto tle2noise = mrpt::system::CTimeLoggerEntry(
				m_world->getTimeLogger(), "sensor.RGBD.renderD_noise");

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
			m_world->getTimeLogger(), "sensor.RGBD.acqObsMtx");

		std::lock_guard<std::mutex> csl(m_last_obs_cs);
		m_last_obs = std::move(curObsPtr);
		m_last_obs2gui = m_last_obs;
	}

	{
		auto lckHasTo = mrpt::lockHelper(m_has_to_render_mtx);

		auto tlePub = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.RGBD.report");

		SensorBase::reportNewObservation(m_last_obs, *m_has_to_render);

		tlePub.stop();

		if (m_glCustomVisual) m_glCustomVisual->setVisibility(true);

		m_gui_uptodate = false;
		m_has_to_render.reset();
	}
}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void DepthCameraSensor::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);
	if (SensorBase::should_simulate_sensor(context))
	{
		auto lckHasTo = mrpt::lockHelper(m_has_to_render_mtx);
		m_has_to_render = context;
		m_world->mark_as_pending_running_sensors_on_3D_scene();
	}
}

void DepthCameraSensor::freeOpenGLResources()
{
	m_fbo_renderer_depth.reset();
	m_fbo_renderer_rgb.reset();
}
