/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/COpenGLScene.h>
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

	// Attribs:
	TParameterDefinitions attribs;
	attribs["name"] = TParamEntry("%s", &this->m_name);

	parse_xmlnode_attribs(*root, attribs, {}, "[DepthCameraSensor]");

	const std::map<std::string, std::string> varValues = {
		{"NAME", m_name}, {"PARENT_NAME", m_vehicle.getName()}};

	using namespace mrpt;  // _deg
	m_sensor_params.sensorPose =
		mrpt::poses::CPose3D(0, 0, 0.5, 90.0_deg, 0, 90.0_deg);

	{
		auto& c = m_sensor_params.cameraParamsIntensity;
		c.ncols = 640;
		c.nrows = 480;
		c.cx(c.ncols / 2);
		c.cy(c.nrows / 2);
		c.fx(500);
		c.fy(500);
	}

	// Other scalar params:
	TParameterDefinitions params;
	params["pose_3d"] = TParamEntry("%pose3d", &m_sensor_params.sensorPose);

	params["sensor_period"] = TParamEntry("%lf", &this->m_sensor_period);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues);

	// Parse common sensor XML params:
	this->parseSensorPublish(root->first_node("publish"), varValues);

	// Pass params to the scan2D obj:
	// m_scan_model.aperture = mrpt::DEG2RAD(fov_deg);

	// Assign a sensible default name/sensor label if none is provided:
	if (m_name.empty())
	{
		size_t nextIdx = 0;
		if (auto v = dynamic_cast<VehicleBase*>(&m_vehicle); v)
			nextIdx = v->getSensors().size() + 1;

		m_name = mrpt::format("camera%u", static_cast<unsigned int>(nextIdx));
	}

	// save sensor label here too:
	m_sensor_params.sensorLabel = m_name;
}

void DepthCameraSensor::internalGuiUpdate(
	mrpt::opengl::COpenGLScene& scene, [[maybe_unused]] bool childrenOnly)
{
	auto lck = mrpt::lockHelper(m_gui_mtx);

	// 1st time?
	if (!m_gl_obs)
	{
		m_gl_obs = mrpt::opengl::CPointCloudColoured::Create();
		m_gl_obs->setPointSize(2.0f);
		m_gl_obs->setLocalRepresentativePoint(
			m_sensor_params.sensorPose.translation());
		scene.insert(m_gl_obs);
	}

	if (!m_gui_uptodate)
	{
		{
			std::lock_guard<std::mutex> csl(m_last_obs_cs);
			if (m_last_obs2gui)
			{
				mrpt::obs::T3DPointsProjectionParams pp;
				pp.takeIntoAccountSensorPoseOnRobot = true;
				m_last_obs2gui->unprojectInto(*m_gl_obs, pp);

				// m_gl_obs->recolorizeByCoordinate() ...??

				m_last_obs2gui.reset();
			}
		}
		m_gui_uptodate = true;
	}

	// Move with vehicle:
	m_gl_obs->setPose(m_vehicle.getPose());
}

void DepthCameraSensor::simul_pre_timestep([
	[maybe_unused]] const TSimulContext& context)
{
}

void DepthCameraSensor::simulateOn3DScene(
	mrpt::opengl::COpenGLScene& world3DScene)
{
	if (!m_has_to_render.has_value()) return;

	auto lck = mrpt::lockHelper(m_gui_mtx);

	auto tle = mrpt::system::CTimeLoggerEntry(
		m_world->getTimeLogger(), "DepthCameraSensor");

	// Start making a copy of the pattern observation:
	auto curObs = mrpt::obs::CObservation3DRangeScan::Create(m_sensor_params);

	// Quick test for camera sensor first test
	if (!m_fbo_renderer)
		m_fbo_renderer = std::make_shared<mrpt::opengl::CFBORender>(
			m_sensor_params.cameraParamsIntensity.ncols,
			m_sensor_params.cameraParamsIntensity.nrows,
			true /* skip GLUT window */);

	auto viewport = world3DScene.getViewport();

	auto& cam = viewport->getCamera();

	auto camBackup = cam;
	cam.set6DOFMode(true);
	cam.setProjectiveFromPinhole(m_sensor_params.cameraParamsIntensity);

	// Camera pose: vehicle + relativePoseOnVehicle:
	// Note: relativePoseOnVehicle should be (y,p,r)=(90deg,0,90deg) to make
	// the camera to look forward:
	auto p =
		mrpt::poses::CPose3D(m_vehicle.getPose()) + m_sensor_params.sensorPose;

	cam.setPose(p);

	// viewport->setCustomBackgroundColor({0.3f, 0.3f, 0.3f, 1.0f});
	// const float clipMax = 25.0f;
	// viewport->setViewportClipDistances(0.1, clipMax);

	viewport->updateMatricesFromCamera();

	mrpt::math::CMatrixFloat depthImage;
	m_fbo_renderer->render_RGBD(
		world3DScene, curObs->intensityImage, depthImage);

	// Convert depth image:
	// TODO!

	cam = camBackup;  // restore
	viewport->updateMatricesFromCamera();

	static int i = 0;
	curObs->intensityImage.saveToFile(mrpt::format("camera_%04i.png", i++));

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
