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
#include <mvsim/Sensors/CameraSensor.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace rapidxml;

CameraSensor::CameraSensor(
	Simulable& parent, const rapidxml::xml_node<char>* root)
	: SensorBase(parent)
{
	this->loadConfigFrom(root);
}

CameraSensor::~CameraSensor() {}

void CameraSensor::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	m_gui_uptodate = false;

	SensorBase::loadConfigFrom(root);
	SensorBase::make_sure_we_have_a_name("camera");

	m_fbo_renderer_rgb.reset();

	using namespace mrpt;  // _deg
	m_sensor_params.cameraPose =
		mrpt::poses::CPose3D(0, 0, 0.5, 90.0_deg, 0, 90.0_deg);

	// Default values:
	{
		auto& c = m_sensor_params.cameraParams;
		c.ncols = 640;
		c.nrows = 480;
		c.cx(c.ncols / 2);
		c.cy(c.nrows / 2);
		c.fx(500);
		c.fy(500);
	}

	// Other scalar params:
	TParameterDefinitions params;
	params["pose_3d"] = TParamEntry("%pose3d", &m_sensor_params.cameraPose);

	auto& rgbCam = m_sensor_params.cameraParams;
	params["cx"] = TParamEntry("%lf", &rgbCam.intrinsicParams(0, 2));
	params["cy"] = TParamEntry("%lf", &rgbCam.intrinsicParams(1, 2));
	params["fx"] = TParamEntry("%lf", &rgbCam.intrinsicParams(0, 0));
	params["fy"] = TParamEntry("%lf", &rgbCam.intrinsicParams(1, 1));

	unsigned int rgb_ncols = rgbCam.ncols, rgb_nrows = rgbCam.nrows;
	params["ncols"] = TParamEntry("%u", &rgb_ncols);
	params["nrows"] = TParamEntry("%u", &rgb_nrows);

	params["clip_min"] = TParamEntry("%f", &m_rgb_clip_min);
	params["clip_max"] = TParamEntry("%f", &m_rgb_clip_max);

	params["ambient_light"] = TParamEntry("%f", &m_ambient_light);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, m_varValues);

	rgbCam.ncols = rgb_ncols;
	rgbCam.nrows = rgb_nrows;

	// save sensor label here too:
	m_sensor_params.sensorLabel = m_name;
}

void CameraSensor::internalGuiUpdate(
	mrpt::opengl::COpenGLScene& viz,
	[[maybe_unused]] mrpt::opengl::COpenGLScene& physical,
	[[maybe_unused]] bool childrenOnly)
{
	auto lck = mrpt::lockHelper(m_gui_mtx);

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
				m_gl_sensor_origin_corner->setPose(
					m_last_obs2gui->sensorPose());

				if (!m_gl_sensor_frustum)
				{
					m_gl_sensor_frustum = mrpt::opengl::CSetOfObjects::Create();

					const float frustumScale = 0.4e-3;
					auto frustum = mrpt::opengl::CFrustum::Create(
						m_last_obs2gui->cameraParams, frustumScale);

					m_gl_sensor_frustum->insert(frustum);
					m_gl_sensor_fov->insert(m_gl_sensor_frustum);
				}

				using namespace mrpt;  // _deg

				m_gl_sensor_frustum->setPose(
					m_last_obs2gui->cameraPose +
					(-mrpt::poses::CPose3D::FromYawPitchRoll(
						-90.0_deg, 0.0_deg, -90.0_deg)));

				m_last_obs2gui.reset();
			}
		}
		m_gui_uptodate = true;
	}

	// Move with vehicle:
	const auto& p = m_vehicle.getPose();

	m_gl_sensor_fov->setPose(p);
	m_gl_sensor_origin->setPose(p);

	if (m_glCustomVisual)
		m_glCustomVisual->setPose(p + m_sensor_params.cameraPose.asTPose());
}

void CameraSensor::simul_pre_timestep([
	[maybe_unused]] const TSimulContext& context)
{
}

void CameraSensor::simulateOn3DScene(mrpt::opengl::COpenGLScene& world3DScene)
{
	using namespace mrpt;  // _deg

	if (!m_has_to_render.has_value()) return;

	auto tleWhole =
		mrpt::system::CTimeLoggerEntry(m_world->getTimeLogger(), "sensor.RGB");

	auto lck = mrpt::lockHelper(m_gui_mtx);

	if (m_glCustomVisual) m_glCustomVisual->setVisibility(false);

	// Start making a copy of the pattern observation:
	auto curObs = mrpt::obs::CObservationImage::Create(m_sensor_params);

	// Set timestamp:
	curObs->timestamp = m_world->get_simul_timestamp();

	// Create FBO on first use, now that we are here at the GUI / OpenGL thread.
	if (!m_fbo_renderer_rgb)
		m_fbo_renderer_rgb = std::make_shared<mrpt::opengl::CFBORender>(
			m_sensor_params.cameraParams.ncols,
			m_sensor_params.cameraParams.nrows, true /* skip GLUT window */);

	auto viewport = world3DScene.getViewport();

	auto& cam = viewport->getCamera();

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

	const auto vehiclePose = m_vehicle_pose_at_last_timestamp;
	const auto rgbSensorPose = vehiclePose + curObs->cameraPose;

	cam.setPose(rgbSensorPose);

	ASSERT_(m_fbo_renderer_rgb);

	auto tle2 = mrpt::system::CTimeLoggerEntry(
		m_world->getTimeLogger(), "sensor.RGB.render");

	// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
	viewport->setViewportClipDistances(m_rgb_clip_min, m_rgb_clip_max);
	viewport->lightParameters().ambient = {
		m_ambient_light, m_ambient_light, m_ambient_light, 1.0f};

	m_fbo_renderer_rgb->render_RGB(world3DScene, curObs->image);

	tle2.stop();

	// Store generated obs:
	{
		std::lock_guard<std::mutex> csl(m_last_obs_cs);
		m_last_obs = std::move(curObs);
		m_last_obs2gui = m_last_obs;
	}

	SensorBase::reportNewObservation(m_last_obs, *m_has_to_render);

	if (m_glCustomVisual) m_glCustomVisual->setVisibility(true);

	m_gui_uptodate = false;
	m_has_to_render.reset();
}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void CameraSensor::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);
	if (SensorBase::should_simulate_sensor(context))
	{
		m_has_to_render = context;
		m_world->mark_as_pending_running_sensors_on_3D_scene();
	}
}

void CameraSensor::freeOpenGLResources() { m_fbo_renderer_rgb.reset(); }
