/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/random.h>
#include <mrpt/version.h>
#include <mvsim/Sensors/LaserScanner.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>

#include "xml_utils.h"

using namespace mvsim;
using namespace rapidxml;

int z_order_cnt = 0;

LaserScanner::LaserScanner(
	Simulable& parent, const rapidxml::xml_node<char>* root)
	: SensorBase(parent), m_z_order(++z_order_cnt)
{
	this->loadConfigFrom(root);
}

LaserScanner::~LaserScanner() {}

void LaserScanner::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	m_gui_uptodate = false;

	SensorBase::loadConfigFrom(root);
	SensorBase::make_sure_we_have_a_name("laser");

	// Other scalar params:
	int nRays = 181;
	double fov_deg = 180;
	m_scan_model.sensorPose.z() = 0.05;

	TParameterDefinitions params;
	params["fov_degrees"] = TParamEntry("%lf", &fov_deg);
	params["nrays"] = TParamEntry("%i", &nRays);
	params["pose"] = TParamEntry("%pose2d_ptr3d", &m_scan_model.sensorPose);
	params["pose_3d"] = TParamEntry("%pose3d", &m_scan_model.sensorPose);
	params["height"] = TParamEntry("%lf", &m_scan_model.sensorPose.z());
	params["range_std_noise"] = TParamEntry("%lf", &m_rangeStdNoise);
	params["maxRange"] = TParamEntry("%f", &m_scan_model.maxRange);
	params["angle_std_noise_deg"] = TParamEntry("%lf_deg", &m_angleStdNoise);
	params["sensor_period"] = TParamEntry("%lf", &m_sensor_period);
	params["bodies_visible"] = TParamEntry("%bool", &m_see_fixtures);

	params["viz_pointSize"] = TParamEntry("%f", &m_viz_pointSize);
	params["viz_visiblePlane"] = TParamEntry("%bool", &m_viz_visiblePlane);
	params["viz_visiblePoints"] = TParamEntry("%bool", &m_viz_visiblePoints);

	params["raytrace_3d"] = TParamEntry("%bool", &m_raytrace_3d);
	params["ignore_parent_body"] = TParamEntry("%bool", &m_ignore_parent_body);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, m_varValues);

	// Pass params to the scan2D obj:
	m_scan_model.aperture = mrpt::DEG2RAD(fov_deg);
	m_scan_model.resizeScan(nRays);
	m_scan_model.stdError = m_rangeStdNoise;

	m_scan_model.sensorLabel = m_name;
}

void LaserScanner::internalGuiUpdate(
	mrpt::opengl::COpenGLScene& viz,
	[[maybe_unused]] mrpt::opengl::COpenGLScene& physical,
	[[maybe_unused]] bool childrenOnly)
{
	auto lck = mrpt::lockHelper(m_gui_mtx);

	auto glVizSensors = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
		viz.getByName("group_sensors_viz"));
	ASSERT_(glVizSensors);

	// 1st time?
	if (!m_gl_scan)
	{
		m_gl_scan = mrpt::opengl::CPlanarLaserScan::Create();
		m_gl_scan->enablePoints(m_viz_visiblePoints);
		m_gl_scan->setPointSize(m_viz_pointSize);
		m_gl_scan->enableSurface(m_viz_visiblePlane);
		// m_gl_scan->setSurfaceColor(0.0f, 0.0f, 1.0f, 0.4f);

		m_gl_scan->setLocalRepresentativePoint({0, 0, 0.10f});

		glVizSensors->insert(m_gl_scan);
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

		auto fovScan = mrpt::opengl::CPlanarLaserScan::Create();
		fovScan->enablePoints(false);
		fovScan->enableSurface(true);

		mrpt::obs::CObservation2DRangeScan s = m_scan_model;
		const float f = 0.30f;
		for (size_t i = 0; i < s.getScanSize(); i++)
		{
			s.setScanRange(i, f);
			s.setScanRangeValidity(i, true);
		}
		fovScan->setScan(s);

		m_gl_sensor_fov->insert(fovScan);

		m_gl_sensor_fov->setVisibility(false);
		viz.insert(m_gl_sensor_fov);
		SensorBase::RegisterSensorFOVViz(m_gl_sensor_fov);
	}

	if (!m_gui_uptodate && glVizSensors->isVisible())
	{
		{
			std::lock_guard<std::mutex> csl(m_last_scan_cs);
			if (m_last_scan2gui)
			{
				m_gl_scan->setScan(*m_last_scan2gui);
				m_gl_sensor_origin_corner->setPose(m_last_scan2gui->sensorPose);

				m_last_scan2gui.reset();
			}
		}
		m_gui_uptodate = true;
	}

	const mrpt::poses::CPose2D& p = m_vehicle.getCPose2D();
	const double z_incrs = 10e-3;  // for m_z_order
	const double z_offset = 1e-2;
	m_gl_scan->setPose(mrpt::poses::CPose3D(
		p.x(), p.y(), z_offset + z_incrs * m_z_order, p.phi(), 0.0, 0.0));

	m_gl_sensor_fov->setPose(p);
	m_gl_sensor_origin->setPose(p);

	if (m_glCustomVisual)
		m_glCustomVisual->setPose(p + m_scan_model.sensorPose);
}

void LaserScanner::simul_pre_timestep([
	[maybe_unused]] const TSimulContext& context)
{
}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void LaserScanner::simul_post_timestep(const TSimulContext& context)
{
	auto lck = mrpt::lockHelper(m_gui_mtx);
	Simulable::simul_post_timestep(context);

	if (SensorBase::should_simulate_sensor(context))
	{
		if (m_raytrace_3d)
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
		else
		{
			// 2D mode:
			internal_simulate_lidar_2d_mode(context);
		}
	}
}

void LaserScanner::internal_simulate_lidar_2d_mode(const TSimulContext& context)
{
	using mrpt::maps::COccupancyGridMap2D;
	using mrpt::obs::CObservation2DRangeScan;

	auto tle = mrpt::system::CTimeLoggerEntry(
		m_world->getTimeLogger(), "LaserScanner");

	// Create an array of scans, each reflecting ranges to one kind of world
	// objects.
	// Finally, we'll take the shortest range in each direction:
	std::list<CObservation2DRangeScan> lstScans;

	const size_t nRays = m_scan_model.getScanSize();
	const double maxRange = m_scan_model.maxRange;

	// Get pose of the robot:
	const mrpt::poses::CPose2D vehPose =
		mrpt::poses::CPose2D(m_vehicle_pose_at_last_timestamp);

	// grid maps:
	// -------------
	m_world->getTimeLogger().enter("LaserScanner.scan.1.gridmap");

	const World::WorldElementList& elements = m_world->getListOfWorldElements();

	for (const auto& element : elements)
	{
		// If not a grid map, ignore:
		const OccupancyGridMap* grid =
			dynamic_cast<const OccupancyGridMap*>(element.get());
		if (!grid) continue;
		const COccupancyGridMap2D& occGrid = grid->getOccGrid();

		// Create new scan:
		lstScans.emplace_back(m_scan_model);
		CObservation2DRangeScan& scan = lstScans.back();

		// Ray tracing over the gridmap:
		occGrid.laserScanSimulator(
			scan, vehPose, 0.5f, m_scan_model.getScanSize(), m_rangeStdNoise, 1,
			m_angleStdNoise);
	}
	m_world->getTimeLogger().leave("LaserScanner.scan.1.gridmap");

	// ray trace on Box2D polygons:
	// ------------------------------
	m_world->getTimeLogger().enter("LaserScanner.scan.2.polygons");
	{
		// Create new scan:
		lstScans.push_back(CObservation2DRangeScan(m_scan_model));
		CObservation2DRangeScan& scan = lstScans.back();

		// Avoid the lidar seeing the vehicle owns shape:
		std::map<b2Fixture*, uintptr_t> orgUserData;

		auto makeFixtureInvisible = [&](b2Fixture* f) {
			if (!f) return;
			orgUserData[f] = f->GetUserData().pointer;
			f->GetUserData().pointer = INVISIBLE_FIXTURE_USER_DATA;
		};
		auto undoInvisibleFixtures = [&]() {
			for (auto& kv : orgUserData)
				kv.first->GetUserData().pointer = kv.second;
		};

		if (auto v = dynamic_cast<VehicleBase*>(&m_vehicle); v)
		{
			makeFixtureInvisible(v->get_fixture_chassis());
			for (auto& f : v->get_fixture_wheels()) makeFixtureInvisible(f);
		}

		// Do Box2D raycasting stuff:
		// ------------------------------
		// This callback finds the closest hit. Polygon 0 is filtered.
		class RayCastClosestCallback : public b2RayCastCallback
		{
		   public:
			RayCastClosestCallback() = default;

			float ReportFixture(
				b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal,
				float fraction) override
			{
				if (!m_see_fixtures || fixture->GetUserData().pointer ==
										   INVISIBLE_FIXTURE_USER_DATA)
				{
					// By returning -1, we instruct the calling code to ignore
					// this fixture and
					// continue the ray-cast to the next fixture.
					return -1.0f;
				}

				m_hit = true;
				m_point = point;
				m_normal = normal;
				// By returning the current fraction, we instruct the calling
				// code to clip the ray and
				// continue the ray-cast to the next fixture. WARNING: do not
				// assume that fixtures
				// are reported in order. However, by clipping, we can always
				// get the closest fixture.
				return fraction;
			}

			bool m_see_fixtures = true;
			bool m_hit = false;
			b2Vec2 m_point{0, 0};
			b2Vec2 m_normal{0, 0};
		};

		const mrpt::poses::CPose2D sensorPose =
			vehPose + mrpt::poses::CPose2D(scan.sensorPose);
		const b2Vec2 sensorPt = b2Vec2(sensorPose.x(), sensorPose.y());

		RayCastClosestCallback callback;
		callback.m_see_fixtures = m_see_fixtures;

		// Scan size:
		ASSERT_(nRays >= 2);
		scan.resizeScanAndAssign(nRays, maxRange, false);
		double A =
			sensorPose.phi() + (scan.rightToLeft ? -0.5 : +0.5) * scan.aperture;
		const double AA =
			(scan.rightToLeft ? 1.0 : -1.0) * (scan.aperture / (nRays - 1));

		// Each thread must create its own rng:
		thread_local mrpt::random::CRandomGenerator rnd;

		for (size_t i = 0; i < nRays; i++, A += AA)
		{
			const b2Vec2 endPt = b2Vec2(
				sensorPt.x + cos(A) * maxRange, sensorPt.y + sin(A) * maxRange);

			callback.m_hit = false;
			m_world->getBox2DWorld()->RayCast(&callback, sensorPt, endPt);
			scan.setScanRangeValidity(i, callback.m_hit);

			float range = 0;
			if (callback.m_hit)
			{
				// Hit:
				range = std::sqrt(
					mrpt::square(callback.m_point.x - sensorPt.x) +
					mrpt::square(callback.m_point.y - sensorPt.y));
				range += rnd.drawGaussian1D_normalized() * m_rangeStdNoise;
			}
			else
			{
				// Miss:
				range = maxRange;
			}
			scan.setScanRange(i, range);
		}  // end for (raycast scan)

		undoInvisibleFixtures();
	}
	m_world->getTimeLogger().leave("LaserScanner.scan.2.polygons");

	// Summarize all scans in one single scan:
	// ----------------------------------------
	m_world->getTimeLogger().enter("LaserScanner.scan.3.merge");

	auto lastScan = CObservation2DRangeScan::Create(m_scan_model);

	lastScan->timestamp = m_world->get_simul_timestamp();
	lastScan->sensorLabel = m_name;

	lastScan->resizeScanAndAssign(nRays, maxRange, false);

	for (const auto& scan : lstScans)
	{
		for (size_t i = 0; i < nRays; i++)
		{
			if (scan.getScanRangeValidity(i))
			{
				lastScan->setScanRange(
					i,
					std::min(lastScan->getScanRange(i), scan.getScanRange(i)));
				lastScan->setScanRangeValidity(i, true);
			}
		}
	}
	m_world->getTimeLogger().leave("LaserScanner.scan.3.merge");

	{
		std::lock_guard<std::mutex> csl(m_last_scan_cs);
		m_last_scan = std::move(lastScan);
		m_last_scan2gui = m_last_scan;
	}

	SensorBase::reportNewObservation(m_last_scan, context);
	m_gui_uptodate = false;
}

void LaserScanner::freeOpenGLResources()
{
	// Free fbo:
	m_fbo_renderer_depth.reset();
}

void LaserScanner::simulateOn3DScene(mrpt::opengl::COpenGLScene& world3DScene)
{
	using namespace mrpt;  // _deg

	{
		auto lckHasTo = mrpt::lockHelper(m_has_to_render_mtx);
		if (!m_has_to_render.has_value()) return;
	}

	auto tleWhole = mrpt::system::CTimeLoggerEntry(
		m_world->getTimeLogger(), "sensor.2Dlidar");

	auto tle1 = mrpt::system::CTimeLoggerEntry(
		m_world->getTimeLogger(), "sensor.2Dlidar.acqGuiMtx");

	auto lck = mrpt::lockHelper(m_gui_mtx);

	tle1.stop();

	if (m_glCustomVisual) m_glCustomVisual->setVisibility(false);

	// Start making a copy of the pattern observation:
	auto curObs = mrpt::obs::CObservation2DRangeScan::Create(m_scan_model);

	const size_t nRays = m_scan_model.getScanSize();
	const double maxRange = m_scan_model.maxRange;

	curObs->timestamp = m_world->get_simul_timestamp();
	curObs->sensorLabel = m_name;

	curObs->resizeScanAndAssign(nRays, maxRange, false);

	// Create FBO on first use, now that we are here at the GUI / OpenGL thread.
	constexpr int FBO_NROWS = 1;
	constexpr int FBO_NCOLS = 500;
	constexpr double camModel_FOV = 150.0_deg;
	mrpt::img::TCamera camModel;
	camModel.ncols = FBO_NCOLS;
	camModel.nrows = FBO_NROWS;
	camModel.cx(camModel.ncols / 2.0);
	camModel.cy(camModel.nrows / 2.0);
	camModel.fx(camModel.cx() / tan(camModel_FOV * 0.5));  // tan(FOV/2)=cx/fx
	camModel.fy(camModel.fx());

	if (!m_fbo_renderer_depth)
	{
#if MRPT_VERSION < 0x256
		m_fbo_renderer_depth = std::make_shared<mrpt::opengl::CFBORender>(
			FBO_NCOLS, FBO_NROWS, true /* skip GLUT window */);
#else
		mrpt::opengl::CFBORender::Parameters p;
		p.width = FBO_NCOLS;
		p.height = FBO_NROWS;
		p.create_EGL_context = false;  // reuse nanogui context

		m_fbo_renderer_depth = std::make_shared<mrpt::opengl::CFBORender>(p);
#endif
	}

	auto viewport = world3DScene.getViewport();

#if MRPT_VERSION < 0x256
	auto& cam = viewport->getCamera();
#else
	auto& cam = m_fbo_renderer_depth->getCamera(world3DScene);
#endif

	const auto fixedAxisConventionRot =
		mrpt::poses::CPose3D(0, 0, 0, -90.0_deg, 0.0_deg, -90.0_deg);

	const auto vehiclePose = m_vehicle_pose_at_last_timestamp;

	// ----------------------------------------------------------
	// Decompose the 2D lidar FOV into "n" depth camera images,
	// of 90deg FOV each.
	// ----------------------------------------------------------
	const auto firstAngle = curObs->getScanAngle(0);  // wrt sensorPose
	const auto lastAngle = curObs->getScanAngle(curObs->getScanSize() - 1);
	const bool scanIsCW = (lastAngle > firstAngle);
	ASSERT_NEAR_(std::abs(lastAngle - firstAngle), curObs->aperture, 1e-3);

	const unsigned int numRenders =
		std::ceil((curObs->aperture / camModel_FOV) - 1e-3);
	const auto numRaysPerRender = mrpt::round(
		nRays * std::min<double>(1.0, (camModel_FOV / curObs->aperture)));

	ASSERT_(numRaysPerRender > 0);

	// Precomputed LUT of bearings to pixel coordinates:
	//                    cx - u
	//  tan(bearing) = --------------
	//                      fx
	//
	thread_local std::vector<size_t> angleIdx2pixelIdx;
	thread_local std::vector<float> angleIdx2secant;
	if (angleIdx2pixelIdx.empty())
	{
		angleIdx2pixelIdx.resize(numRaysPerRender);
		angleIdx2secant.resize(numRaysPerRender);

		for (int i = 0; i < numRaysPerRender; i++)
		{
			const auto ang = (scanIsCW ? -1 : 1) *
							 (camModel_FOV * 0.5 -
							  i * camModel_FOV / (numRaysPerRender - 1));

			const auto pixelIdx = mrpt::saturate_val<int>(
				mrpt::round(camModel.cx() - camModel.fx() * std::tan(ang)), 0,
				camModel.ncols - 1);

			angleIdx2pixelIdx.at(i) = pixelIdx;
			angleIdx2secant.at(i) = 1.0f / std::cos(ang);
		}
	}

	// ----------------------------------------------------------
	// "DEPTH camera" to generate lidar readings:
	// ----------------------------------------------------------
	cam.set6DOFMode(true);
	cam.setProjectiveFromPinhole(camModel);

	viewport->setViewportClipDistances(0.01, curObs->maxRange);
	mrpt::math::CMatrixFloat depthImage;

	// make owner's own body invisible?
	auto visVeh = dynamic_cast<VisualObject*>(&m_vehicle);
	bool formerVisVehState = true;
	if (visVeh && m_ignore_parent_body)
	{
		formerVisVehState = visVeh->customVisualVisible();
		visVeh->customVisualVisible(false);
	}

	for (size_t renderIdx = 0; renderIdx < numRenders; renderIdx++)
	{
		const double thisRenderMidAngle =
			firstAngle + (camModel_FOV / 2.0 + camModel_FOV * renderIdx) *
							 (scanIsCW ? 1 : -1);

		const auto depthSensorPose = vehiclePose + curObs->sensorPose +
									 mrpt::poses::CPose3D::FromYawPitchRoll(
										 thisRenderMidAngle, 0.0, 0.0) +
									 fixedAxisConventionRot;

		// Camera pose: vehicle + relativePoseOnVehicle:
		// Note: relativePoseOnVehicle should be (y,p,r)=(90deg,0,90deg) to make
		// the camera to look forward:
		cam.setPose(depthSensorPose);

		auto tleRender = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.2Dlidar.renderSubScan");

		m_fbo_renderer_depth->render_depth(world3DScene, depthImage);

		tleRender.stop();

		// Add random noise:
		if (m_rangeStdNoise > 0)
		{
			auto tleStore = mrpt::system::CTimeLoggerEntry(
				m_world->getTimeLogger(), "sensor.2Dlidar.noise");

			// Each thread must create its own rng:
			thread_local mrpt::random::CRandomGenerator rng;

			float* d = depthImage.data();
			const size_t N = depthImage.size();
			for (size_t i = 0; i < N; i++)
			{
				if (d[i] == 0) continue;  // it was an invalid ray return.

				const float dNoisy =
					d[i] + rng.drawGaussian1D(0, m_rangeStdNoise);

				if (dNoisy < 0 || dNoisy > curObs->maxRange) continue;

				d[i] = dNoisy;
			}
		}

		auto tleStore = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.2Dlidar.storeObs");

		// Convert depth into range and store into scan observation:
		for (int i = 0; i < numRaysPerRender; i++)
		{
			const auto scanRayIdx = numRaysPerRender * renderIdx + i;
			// done with full scan range?
			if (scanRayIdx >= curObs->getScanSize()) break;

			const auto u = angleIdx2pixelIdx.at(i);

			const float d = depthImage(0, u);
			const float range = d * angleIdx2secant.at(i);

			if (range <= 0 || range >= curObs->maxRange) continue;	// invalid

			curObs->setScanRange(scanRayIdx, range);
			curObs->setScanRangeValidity(scanRayIdx, true);
		}
		tleStore.stop();
	}

	if (visVeh && m_ignore_parent_body)
		visVeh->customVisualVisible(formerVisVehState);

	// Store generated obs:
	{
		auto tle3 = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.2Dlidar.acqObsMtx");

		std::lock_guard<std::mutex> csl(m_last_scan_cs);
		m_last_scan = std::move(curObs);
		m_last_scan2gui = m_last_scan;
	}

	{
		auto lckHasTo = mrpt::lockHelper(m_has_to_render_mtx);

		auto tlePub = mrpt::system::CTimeLoggerEntry(
			m_world->getTimeLogger(), "sensor.2Dlidar.report");

		SensorBase::reportNewObservation(m_last_scan, *m_has_to_render);

		tlePub.stop();

		if (m_glCustomVisual) m_glCustomVisual->setVisibility(true);

		m_gui_uptodate = false;

		m_has_to_render.reset();
	}
}
