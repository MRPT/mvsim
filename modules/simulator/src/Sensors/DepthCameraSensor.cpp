/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/random.h>
#include <mvsim/Sensors/DepthCameraSensor.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace rapidxml;

DepthCameraSensor::DepthCameraSensor(
	Simulable& parent, const rapidxml::xml_node<char>* root)
	: SensorBase(parent)
{
	this->loadConfigFrom(root);
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

	// 1st time?
	if (!m_gl_obs)
	{
		m_gl_obs = mrpt::opengl::CPointCloudColoured::Create();
		m_gl_obs->setPointSize(2.0f);
		m_gl_obs->setLocalRepresentativePoint(
			m_sensor_params.sensorPose.translation());
		viz.insert(m_gl_obs);
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
			if (m_last_obs2gui)
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
}

void DepthCameraSensor::simul_pre_timestep(
	[[maybe_unused]] const TSimulContext& context)
{
}

void DepthCameraSensor::simulateOn3DScene(
	mrpt::opengl::COpenGLScene& world3DScene)
{
	using namespace mrpt;  // _deg

	if (!m_has_to_render.has_value()) return;

	auto lck = mrpt::lockHelper(m_gui_mtx);

	auto tle = mrpt::system::CTimeLoggerEntry(
		m_world->getTimeLogger(), "DepthCameraSensor");

	// Start making a copy of the pattern observation:
	auto curObs = mrpt::obs::CObservation3DRangeScan::Create(m_sensor_params);

	// Set timestamp:
	curObs->timestamp = mrpt::Clock::now();

	// Create FBO on first use, now that we are here at the GUI / OpenGL thread.
	if (!m_fbo_renderer_rgb)
		m_fbo_renderer_rgb = std::make_shared<mrpt::opengl::CFBORender>(
			m_sensor_params.cameraParamsIntensity.ncols,
			m_sensor_params.cameraParamsIntensity.nrows,
			true /* skip GLUT window */);

	if (!m_fbo_renderer_depth)
		m_fbo_renderer_depth = std::make_shared<mrpt::opengl::CFBORender>(
			m_sensor_params.cameraParams.ncols,
			m_sensor_params.cameraParams.nrows, true /* skip GLUT window */);

	auto viewport = world3DScene.getViewport();

	auto& cam = viewport->getCamera();

	const auto fixedAxisConventionRot =
		mrpt::poses::CPose3D(0, 0, 0, -90.0_deg, 0.0_deg, -90.0_deg);

	// ----------------------------------------------------------
	// RGB first with its camera intrinsics & clip distances
	// ----------------------------------------------------------
	cam.set6DOFMode(true);
	cam.setProjectiveFromPinhole(curObs->cameraParamsIntensity);

	// RGB camera pose:
	//   vehicle (+) relativePoseOnVehicle (+) relativePoseIntensityWRTDepth
	//
	// Note: relativePoseOnVehicle should be (y,p,r)=(-90deg,0,-90deg) to make
	// the camera to look forward:

	const auto vehiclePose = mrpt::poses::CPose3D(m_vehicle.getPose());

	const auto depthSensorPose =
		vehiclePose + curObs->sensorPose + fixedAxisConventionRot;

	const auto rgbSensorPose = vehiclePose + curObs->sensorPose +
							   curObs->relativePoseIntensityWRTDepth;

	cam.setPose(rgbSensorPose);

	// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
	viewport->setViewportClipDistances(m_rgb_clip_min, m_rgb_clip_max);
	viewport->lightParameters().ambient = {
		m_ambient_light, m_ambient_light, m_ambient_light, 1.0f};

	m_fbo_renderer_rgb->render_RGB(world3DScene, curObs->intensityImage);

	curObs->hasIntensityImage = true;

	// ----------------------------------------------------------
	// DEPTH camera next
	// ----------------------------------------------------------
	cam.setProjectiveFromPinhole(curObs->cameraParams);

	// Camera pose: vehicle + relativePoseOnVehicle:
	// Note: relativePoseOnVehicle should be (y,p,r)=(90deg,0,90deg) to make
	// the camera to look forward:
	cam.setPose(depthSensorPose);

	// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
	viewport->setViewportClipDistances(m_depth_clip_min, m_depth_clip_max);

	mrpt::math::CMatrixFloat depthImage;
	m_fbo_renderer_depth->render_depth(world3DScene, depthImage);

	// Convert depth image:
	curObs->hasRangeImage = true;
	curObs->range_is_depth = true;

	// Add random noise:
	if (m_depth_noise_sigma > 0)
	{
		auto& rng = mrpt::random::getRandomGenerator();

		float* d = depthImage.data();
		const size_t N = depthImage.size();
		for (size_t i = 0; i < N; i++)
		{
			if (d[i] == 0) continue;  // it was an invalid ray return.

			const float dNoisy =
				d[i] + rng.drawGaussian1D(0, m_depth_noise_sigma);

			if (dNoisy < 0 || dNoisy > curObs->maxRange) continue;

			d[i] = dNoisy;
		}
	}

	// float -> uint16_t with "curObs->rangeUnits" units:
	curObs->rangeImage_setSize(depthImage.rows(), depthImage.cols());
	curObs->rangeImage =
		(depthImage.asEigen().cwiseMin(curObs->maxRange) / curObs->rangeUnits)
			.cast<uint16_t>();

	// Store generated obs:
	{
		std::lock_guard<std::mutex> csl(m_last_obs_cs);
		m_last_obs = std::move(curObs);
		m_last_obs2gui = m_last_obs;
	}

	SensorBase::reportNewObservation(m_last_obs, *m_has_to_render);

	m_gui_uptodate = false;
	m_has_to_render.reset();
}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void DepthCameraSensor::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);

	// Limit sensor rate:
	if (context.simul_time < m_sensor_last_timestamp + m_sensor_period) return;
	m_sensor_last_timestamp = context.simul_time;
	m_has_to_render = context;
}

void DepthCameraSensor::freeOpenGLResources()
{
	m_fbo_renderer_depth.reset();
	m_fbo_renderer_rgb.reset();
}
