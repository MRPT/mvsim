/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/random.h>
#include <mrpt/version.h>
#include <mvsim/Sensors/Lidar3D.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace rapidxml;

MRPT_TODO("Also store obs as CObservationRotatingScan?");

Lidar3D::Lidar3D(Simulable& parent, const rapidxml::xml_node<char>* root)
	: SensorBase(parent)
{
	Lidar3D::loadConfigFrom(root);
}

Lidar3D::~Lidar3D() {}

void Lidar3D::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	m_gui_uptodate = false;

	SensorBase::loadConfigFrom(root);
	SensorBase::make_sure_we_have_a_name("laser");

	// Other scalar params:
	TParameterDefinitions params;
	params["pose_3d"] = TParamEntry("%pose3d", &m_sensorPoseOnVeh);
	params["range_std_noise"] = TParamEntry("%lf", &m_rangeStdNoise);
	params["sensor_period"] = TParamEntry("%lf", &m_sensor_period);
	params["max_range"] = TParamEntry("%f", &m_maxRange);
	params["viz_pointSize"] = TParamEntry("%f", &m_viz_pointSize);
	params["ignore_parent_body"] = TParamEntry("%bool", &m_ignore_parent_body);

	params["vert_fov_degrees"] = TParamEntry("%lf_deg", &m_vertical_fov);
	params["vert_nrays"] = TParamEntry("%i", &m_vertNumRays);
	params["horz_nrays"] = TParamEntry("%i", &m_horzNumRays);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, m_varValues);
}

void Lidar3D::internalGuiUpdate(
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
	if (!m_glPoints && glVizSensors)
	{
		m_glPoints = mrpt::opengl::CPointCloudColoured::Create();
		m_glPoints->setPointSize(m_viz_pointSize);
		m_glPoints->setLocalRepresentativePoint({0, 0, 0.10f});

		glVizSensors->insert(m_glPoints);
	}
	if (!m_gl_sensor_origin && viz)
	{
		m_gl_sensor_origin = mrpt::opengl::CSetOfObjects::Create();
		m_gl_sensor_origin_corner =
			mrpt::opengl::stock_objects::CornerXYZSimple(0.15f);

		m_gl_sensor_origin->insert(m_gl_sensor_origin_corner);

		m_gl_sensor_origin->setVisibility(false);
		viz->get().insert(m_gl_sensor_origin);
		SensorBase::RegisterSensorOriginViz(m_gl_sensor_origin);
	}
	if (!m_gl_sensor_fov && viz)
	{
		m_gl_sensor_fov = mrpt::opengl::CSetOfObjects::Create();

		MRPT_TODO("render 3D lidar FOV");
#if 0
		auto fovScan = mrpt::opengl::CPlanarLaserScan::Create();
		m_gl_sensor_fov->insert(fovScan);
#endif
		m_gl_sensor_fov->setVisibility(false);
		viz->get().insert(m_gl_sensor_fov);
		SensorBase::RegisterSensorFOVViz(m_gl_sensor_fov);
	}

	if (!m_gui_uptodate && glVizSensors->isVisible())
	{
		{
			std::lock_guard<std::mutex> csl(m_last_scan_cs);
			if (m_last_scan2gui && m_last_scan2gui->pointcloud)
			{
				m_glPoints->loadFromPointsMap(
					m_last_scan2gui->pointcloud.get());
				m_gl_sensor_origin_corner->setPose(m_last_scan2gui->sensorPose);

				m_last_scan2gui.reset();
			}
		}
		m_gui_uptodate = true;
	}

	const mrpt::poses::CPose3D p = m_vehicle.getCPose3D();

	if (m_glPoints) m_glPoints->setPose(p);
	if (m_gl_sensor_fov) m_gl_sensor_fov->setPose(p);
	if (m_gl_sensor_origin) m_gl_sensor_origin->setPose(p);
	if (m_glCustomVisual) m_glCustomVisual->setPose(p + m_sensorPoseOnVeh);
}

void Lidar3D::simul_pre_timestep([[maybe_unused]] const TSimulContext& context)
{
}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void Lidar3D::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);

	if (SensorBase::should_simulate_sensor(context))
	{
		auto lckHasTo = mrpt::lockHelper(m_has_to_render_mtx);

		// Will run upon next async call of simulateOn3DScene()
		if (m_has_to_render.has_value())
		{
			m_world->logFmt(
				mrpt::system::LVL_WARN,
				"Time for a new sample came without still simulating the "
				"last one (!) for simul_time=%.03f s.",
				m_has_to_render->simul_time);
		}

		m_has_to_render = context;
		m_world->mark_as_pending_running_sensors_on_3D_scene();
	}
}

void Lidar3D::freeOpenGLResources()
{
	// Free fbo:
	m_fbo_renderer_depth.reset();
}

void Lidar3D::simulateOn3DScene(mrpt::opengl::COpenGLScene& world3DScene)
{
	using namespace mrpt;  // _deg

	{
		auto lckHasTo = mrpt::lockHelper(m_has_to_render_mtx);
		if (!m_has_to_render.has_value()) return;
	}

	auto tleWhole = mrpt::system::CTimeLoggerEntry(
		m_world->getTimeLogger(), "sensor.3Dlidar");

	auto tle1 = mrpt::system::CTimeLoggerEntry(
		m_world->getTimeLogger(), "sensor.3Dlidar.acqGuiMtx");

	tle1.stop();

	// The sensor body must be made of transparent material! :-)
	if (m_glCustomVisual) m_glCustomVisual->setVisibility(false);

	// Create empty observation:
	auto curObs = mrpt::obs::CObservationPointCloud::Create();
	curObs->sensorPose = m_sensorPoseOnVeh;
	curObs->timestamp = m_world->get_simul_timestamp();
	curObs->sensorLabel = m_name;

	auto curPtsPtr = mrpt::maps::CSimplePointsMap::Create();
	auto& curPts = *curPtsPtr;
	curObs->pointcloud = curPtsPtr;

	// Create FBO on first use, now that we are here at the GUI / OpenGL thread.
	constexpr double camModel_hFOV = 150.0_deg;
	const int FBO_NROWS = m_vertNumRays * 2;
	// This FBO is for camModel_hFOV only:
	const int FBO_NCOLS = m_horzNumRays;
	const double camModel_vFOV = m_vertical_fov * 1.1;

	mrpt::img::TCamera camModel;
	camModel.ncols = FBO_NCOLS;
	camModel.nrows = FBO_NROWS;
	camModel.cx(camModel.ncols / 2.0);
	camModel.cy(camModel.nrows / 2.0);
	camModel.fx(camModel.cx() / tan(camModel_hFOV * 0.5));	// tan(FOV/2)=cx/fx
	camModel.fy(camModel.cy() / tan(camModel_vFOV * 0.5));

	if (!m_fbo_renderer_depth)
	{
		mrpt::opengl::CFBORender::Parameters p;
		p.width = FBO_NCOLS;
		p.height = FBO_NROWS;
		p.create_EGL_context = world()->sensor_has_to_create_egl_context();

		m_fbo_renderer_depth = std::make_shared<mrpt::opengl::CFBORender>(p);
	}

	const size_t nCols = m_horzNumRays;
	const size_t nRows = m_vertNumRays;

	mrpt::math::CMatrixDouble rangeImage(nRows, nCols);
	rangeImage.setZero();  // 0=invalid (no lidar return)

	auto viewport = world3DScene.getViewport();

	auto& cam = m_fbo_renderer_depth->getCamera(world3DScene);

	const auto fixedAxisConventionRot =
		mrpt::poses::CPose3D(0, 0, 0, -90.0_deg, 0.0_deg, -90.0_deg);

	const auto vehiclePose = mrpt::poses::CPose3D(m_vehicle.getPose());

	// ----------------------------------------------------------
	// Decompose the horizontal lidar FOV into "n" depth images,
	// of camModel_hFOV each.
	// ----------------------------------------------------------
	ASSERT_GT_(m_horzNumRays, 1);
	ASSERT_GT_(m_vertNumRays, 1);

	constexpr bool scanIsCW = true;
	constexpr double aperture = 2 * M_PI;

	const double firstAngle = -aperture * 0.5;

	const unsigned int numRenders =
		std::ceil((aperture / camModel_hFOV) - 1e-3);
	const auto numHorzRaysPerRender = mrpt::round(
		m_horzNumRays * std::min<double>(1.0, (camModel_hFOV / aperture)));

	ASSERT_(numHorzRaysPerRender > 0);

	// Precomputed LUT of bearings to pixel coordinates:
	//                    cx - u
	//  tan(bearing) = --------------
	//                      fx
	//
	thread_local std::vector<size_t> angleIdx2pixelIdx, vertAngleIdx2pixelIdx;
	thread_local std::vector<float> angleIdx2secant, vertAngleIdx2secant;
	if (angleIdx2pixelIdx.empty())
	{
		angleIdx2pixelIdx.resize(numHorzRaysPerRender);
		angleIdx2secant.resize(numHorzRaysPerRender);

		for (int i = 0; i < numHorzRaysPerRender; i++)
		{
			const auto ang = (scanIsCW ? -1 : 1) *
							 (camModel_hFOV * 0.5 -
							  i * camModel_hFOV / (numHorzRaysPerRender - 1));

			const auto pixelIdx = mrpt::saturate_val<int>(
				mrpt::round(camModel.cx() - camModel.fx() * std::tan(ang)), 0,
				camModel.ncols - 1);

			angleIdx2pixelIdx.at(i) = pixelIdx;
			angleIdx2secant.at(i) = 1.0f / std::cos(ang);
		}
	}
	if (vertAngleIdx2pixelIdx.empty())
	{
		vertAngleIdx2pixelIdx.resize(nRows);
		vertAngleIdx2secant.resize(nRows);

		for (size_t i = 0; i < nRows; i++)
		{
			const auto ang =
				(camModel_vFOV * 0.5 - i * camModel_vFOV / (nRows - 1));

			const auto pixelIdx = mrpt::saturate_val<int>(
				mrpt::round(camModel.cy() - camModel.fy() * std::tan(ang)), 0,
				camModel.nrows - 1);

			vertAngleIdx2pixelIdx.at(i) = pixelIdx;
			vertAngleIdx2secant.at(i) = 1.0f / std::cos(ang);
		}
	}

	// ----------------------------------------------------------
	// "DEPTH camera" to generate lidar readings:
	// ----------------------------------------------------------
	cam.set6DOFMode(true);
	cam.setProjectiveFromPinhole(camModel);

	viewport->setViewportClipDistances(0.01, m_maxRange);
	mrpt::math::CMatrixFloat depthImage;

	// make owner's own body invisible?
	auto visVeh = dynamic_cast<VisualObject*>(&m_vehicle);
	auto veh = dynamic_cast<VehicleBase*>(&m_vehicle);
	bool formerVisVehState = true;
	if (m_ignore_parent_body)
	{
		if (visVeh)
		{
			formerVisVehState = visVeh->customVisualVisible();
			visVeh->customVisualVisible(false);
		}
		if (veh) veh->chassisAndWheelsVisible(false);
	}

	for (size_t renderIdx = 0; renderIdx < numRenders; renderIdx++)
	{
		const double thisRenderMidAngle =
			firstAngle + (camModel_hFOV / 2.0 + camModel_hFOV * renderIdx) *
							 (scanIsCW ? 1 : -1);

		const auto thisDepthSensorPoseOnVeh =
			curObs->sensorPose +
			mrpt::poses::CPose3D::FromYawPitchRoll(
				thisRenderMidAngle, 0.0, 0.0) +
			fixedAxisConventionRot;

		const auto thisDepthSensorPose = vehiclePose + thisDepthSensorPoseOnVeh;

		// Camera pose: vehicle + relativePoseOnVehicle:
		// Note: relativePoseOnVehicle should be (y,p,r)=(90deg,0,90deg)
		// to make the camera to look forward:
		cam.setPose(thisDepthSensorPose);

		auto tleRender = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.3Dlidar.renderSubScan");

		m_fbo_renderer_depth->render_depth(world3DScene, depthImage);

		tleRender.stop();

		// Add random noise:
		if (m_rangeStdNoise > 0)
		{
			auto tleStore = mrpt::system::CTimeLoggerEntry(
				m_world->getTimeLogger(), "sensor.3Dlidar.noise");

			// Each thread must create its own rng:
			thread_local mrpt::random::CRandomGenerator rng;

			float* d = depthImage.data();
			const size_t N = depthImage.size();
			for (size_t i = 0; i < N; i++)
			{
				if (d[i] == 0) continue;  // it was an invalid ray return.

				const float dNoisy =
					d[i] + rng.drawGaussian1D(0, m_rangeStdNoise);

				if (dNoisy < 0 || dNoisy > m_maxRange) continue;

				d[i] = dNoisy;
			}
		}

		auto tleStore = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.3Dlidar.storeObs");

		// Convert depth into range and store into polar range images:
		for (int i = 0; i < numHorzRaysPerRender; i++)
		{
			const int iAbs = i + numHorzRaysPerRender * renderIdx;
			if (iAbs >= rangeImage.cols())
				continue;  // we don't need this image part

			const auto u = angleIdx2pixelIdx.at(i);

			for (unsigned int j = 0; j < nRows; j++)
			{
				const auto v = vertAngleIdx2pixelIdx.at(j);

				const float d = depthImage(v, u);
				const float range =
					d * angleIdx2secant.at(i) * vertAngleIdx2secant.at(j);

				if (range <= 0 || range >= m_maxRange) continue;  // invalid

				ASSERTDEB_LT_(j, rangeImage.rows());
				ASSERTDEB_LT_(iAbs, rangeImage.cols());

				rangeImage(j, iAbs) = range;

				// add points:
				const mrpt::math::TPoint3D pt_wrt_cam = {
					d * (u - camModel.cx()) / camModel.fx(),
					d * (v - camModel.cy()) / camModel.fy(), d};
				curPts.insertPoint(
					thisDepthSensorPoseOnVeh.composePoint(pt_wrt_cam));
			}
		}
		tleStore.stop();
	}

	if (m_ignore_parent_body)
	{
		if (visVeh) visVeh->customVisualVisible(formerVisVehState);
		if (veh) veh->chassisAndWheelsVisible(formerVisVehState);
	}

	// Store generated obs:
	{
		auto tle3 = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.3Dlidar.acqObsMtx");

		std::lock_guard<std::mutex> csl(m_last_scan_cs);
		m_last_scan = std::move(curObs);
		m_last_scan2gui = m_last_scan;
	}

	{
		auto lckHasTo = mrpt::lockHelper(m_has_to_render_mtx);

		auto tlePub = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.3Dlidar.report");

		SensorBase::reportNewObservation(m_last_scan, *m_has_to_render);

		tlePub.stop();

		if (m_glCustomVisual) m_glCustomVisual->setVisibility(true);

		m_gui_uptodate = false;

		m_has_to_render.reset();
	}
}
