/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
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

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <mvsim/mvsim-msgs/ObservationLidar2D.pb.h>
#endif

using namespace mvsim;
using namespace rapidxml;

int z_order_cnt = 0;

LaserScanner::LaserScanner(Simulable& parent, const rapidxml::xml_node<char>* root)
	: SensorBase(parent), z_order_(++z_order_cnt)
{
	LaserScanner::loadConfigFrom(root);
}

LaserScanner::~LaserScanner() {}

void LaserScanner::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	gui_uptodate_ = false;

	SensorBase::loadConfigFrom(root);
	SensorBase::make_sure_we_have_a_name("laser");

	// Other scalar params:
	int nRays = 181;
	double fov_deg = 180;
	scan_model_.sensorPose.z() = 0.05;

	TParameterDefinitions params;
	params["fov_degrees"] = TParamEntry("%lf", &fov_deg);
	params["nrays"] = TParamEntry("%i", &nRays);
	params["pose"] = TParamEntry("%pose2d_ptr3d", &scan_model_.sensorPose);
	params["pose_3d"] = TParamEntry("%pose3d", &scan_model_.sensorPose);
	params["height"] = TParamEntry("%lf", &scan_model_.sensorPose.z());
	params["range_std_noise"] = TParamEntry("%lf", &rangeStdNoise_);
	params["max_range"] = TParamEntry("%f", &scan_model_.maxRange);
	params["angle_std_noise_deg"] = TParamEntry("%lf_deg", &angleStdNoise_);
	params["sensor_period"] = TParamEntry("%lf", &sensor_period_);
	params["bodies_visible"] = TParamEntry("%bool", &see_fixtures_);

	params["viz_pointSize"] = TParamEntry("%f", &viz_pointSize_);
	params["viz_visiblePlane"] = TParamEntry("%bool", &viz_visiblePlane_);
	params["viz_visibleLines"] = TParamEntry("%bool", &viz_visibleLines_);
	params["viz_visiblePoints"] = TParamEntry("%bool", &viz_visiblePoints_);

	params["viz_pointsColor"] = TParamEntry("%color", &viz_pointsColor_);
	params["viz_planeColor"] = TParamEntry("%color", &viz_planeColor_);

	params["raytrace_3d"] = TParamEntry("%bool", &raytrace_3d_);
	params["ignore_parent_body"] = TParamEntry("%bool", &ignore_parent_body_);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues_);

	// Pass params to the scan2D obj:
	scan_model_.aperture = mrpt::DEG2RAD(fov_deg);
	scan_model_.resizeScan(nRays);
	scan_model_.stdError = rangeStdNoise_;

	scan_model_.sensorLabel = name_;
}

void LaserScanner::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	using namespace std::string_literals;

	mrpt::opengl::CSetOfObjects::Ptr glVizSensors;
	if (viz)
	{
		glVizSensors = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
			viz->get().getByName("group_sensors_viz"));
		if (!glVizSensors) return;	// may happen during shutdown
	}

	// 1st time?
	if (!gl_scan_ && glVizSensors)
	{
		gl_scan_ = mrpt::opengl::CPlanarLaserScan::Create();

		gl_scan_->enablePoints(viz_visiblePoints_);
		gl_scan_->setPointSize(viz_pointSize_);
		const mrpt::img::TColorf ptsCol(viz_pointsColor_);
		gl_scan_->setPointsColor(ptsCol.R, ptsCol.G, ptsCol.B, ptsCol.A);

		gl_scan_->enableSurface(viz_visiblePlane_);
		const mrpt::img::TColorf planeCol(viz_planeColor_);
		gl_scan_->setSurfaceColor(planeCol.R, planeCol.G, planeCol.B, planeCol.A);

		gl_scan_->enableLine(viz_visibleLines_);

		gl_scan_->setLocalRepresentativePoint({0, 0, 0.10f});
		gl_scan_->setName("glScan veh:"s + vehicle_.getName() + " sensor:"s + this->name_);

		glVizSensors->insert(gl_scan_);
	}
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
#if MRPT_VERSION >= 0x270
		gl_sensor_fov_->castShadows(false);
#endif

		auto fovScan = mrpt::opengl::CPlanarLaserScan::Create();
		fovScan->enablePoints(false);
		fovScan->enableSurface(true);

		mrpt::obs::CObservation2DRangeScan s = scan_model_;
		const float f = 0.30f;
		for (size_t i = 0; i < s.getScanSize(); i++)
		{
			s.setScanRange(i, f);
			s.setScanRangeValidity(i, true);
		}
		fovScan->setScan(s);

		gl_sensor_fov_->insert(fovScan);

		gl_sensor_fov_->setVisibility(false);
		viz->get().insert(gl_sensor_fov_);
		SensorBase::RegisterSensorFOVViz(gl_sensor_fov_);
	}

	if (!gui_uptodate_ && glVizSensors->isVisible())
	{
		{
			std::lock_guard<std::mutex> csl(last_scan_cs_);
			if (last_scan2gui_)
			{
				gl_scan_->setScan(*last_scan2gui_);
				gl_sensor_origin_corner_->setPose(last_scan2gui_->sensorPose);

				last_scan2gui_.reset();
			}
		}
		gui_uptodate_ = true;
	}

	const mrpt::poses::CPose3D p = vehicle_.getCPose3D();
	const auto pp = parent()->applyWorldRenderOffset(p);

	const double z_incrs = 1e-3;  // for z_order_
	const double z_offset = 1e-3;

	if (gl_scan_)
	{
		auto p2 = pp;
		p2.z_incr(z_offset + z_incrs * z_order_);
		gl_scan_->setPose(p2);
	}

	if (gl_sensor_fov_) gl_sensor_fov_->setPose(pp);
	if (gl_sensor_origin_) gl_sensor_origin_->setPose(pp);
	if (glCustomVisual_) glCustomVisual_->setPose(pp + scan_model_.sensorPose);
}

void LaserScanner::simul_pre_timestep([[maybe_unused]] const TSimulContext& context) {}

// Simulate sensor AFTER timestep, with the updated vehicle dynamical state:
void LaserScanner::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);

	if (SensorBase::should_simulate_sensor(context))
	{
		if (raytrace_3d_)
		{
			auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);

			// Will run upon next async call of simulateOn3DScene()
			if (has_to_render_.has_value())
			{
				world_->logFmt(
					mrpt::system::LVL_WARN,
					"Time for a new sample came without still simulating the "
					"last one (!) for simul_time=%.03f s.",
					has_to_render_->simul_time);
			}

			has_to_render_ = context;
			world_->mark_as_pending_running_sensors_on_3D_scene();
		}
		else
		{
			// 2D mode:
			internal_simulate_lidar_2d_mode(context);
		}
	}

	// Keep sensor global pose up-to-date:
	const auto& p = vehicle_.getPose();
	const auto globalSensorPose = p + scan_model_.sensorPose.asTPose();
	Simulable::setPose(globalSensorPose, false /*do not notify*/);
}

void LaserScanner::notifySimulableSetPose(const mrpt::math::TPose3D& newPose)
{
	// The editor has moved the sensor in global coordinates.
	// Convert back to local:
	const auto& p = vehicle_.getPose();
	scan_model_.sensorPose = mrpt::poses::CPose3D(newPose - p);
}

void LaserScanner::internal_simulate_lidar_2d_mode(const TSimulContext& context)
{
	using mrpt::maps::COccupancyGridMap2D;
	using mrpt::obs::CObservation2DRangeScan;

	auto tle = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "LaserScanner");

	// Create an array of scans, each reflecting ranges to one kind of world
	// objects.
	// Finally, we'll take the shortest range in each direction:
	std::list<CObservation2DRangeScan> lstScans;

	const size_t nRays = scan_model_.getScanSize();
	const double maxRange = scan_model_.maxRange;

	// Get pose of the robot:
	const mrpt::poses::CPose2D vehPose = vehicle_.getCPose2D();

	// grid maps:
	// -------------
	world_->getTimeLogger().enter("LaserScanner.scan.1.gridmap");

	const World::WorldElementList& elements = world_->getListOfWorldElements();

	for (const auto& element : elements)
	{
		// If not a grid map, ignore:
		const OccupancyGridMap* grid = dynamic_cast<const OccupancyGridMap*>(element.get());
		if (!grid) continue;
		const COccupancyGridMap2D& occGrid = grid->getOccGrid();

		// Create new scan:
		lstScans.emplace_back(scan_model_);
		CObservation2DRangeScan& scan = lstScans.back();

		// Ray tracing over the gridmap:
		occGrid.laserScanSimulator(
			scan, vehPose, 0.5f, scan_model_.getScanSize(), rangeStdNoise_, 1, angleStdNoise_);
	}
	world_->getTimeLogger().leave("LaserScanner.scan.1.gridmap");

	// ray trace on Box2D polygons:
	// ------------------------------
	world_->getTimeLogger().enter("LaserScanner.scan.2.polygons");
	{
		// Create new scan:
		lstScans.push_back(CObservation2DRangeScan(scan_model_));
		CObservation2DRangeScan& scan = lstScans.back();

		// Avoid the lidar seeing the vehicle owns shape:
		std::map<b2Fixture*, uintptr_t> orgUserData;

		auto makeFixtureInvisible = [&](b2Fixture* f)
		{
			if (!f) return;
			orgUserData[f] = f->GetUserData().pointer;
			f->GetUserData().pointer = INVISIBLE_FIXTURE_USER_DATA;
		};
		auto undoInvisibleFixtures = [&]()
		{
			for (auto& kv : orgUserData) kv.first->GetUserData().pointer = kv.second;
		};

		if (auto v = dynamic_cast<VehicleBase*>(&vehicle_); v)
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
				if (!see_fixtures_ || fixture->GetUserData().pointer == INVISIBLE_FIXTURE_USER_DATA)
				{
					// By returning -1, we instruct the calling code to ignore
					// this fixture and
					// continue the ray-cast to the next fixture.
					return -1.0f;
				}

				hit_ = true;
				point_ = point;
				normal_ = normal;
				// By returning the current fraction, we instruct the calling
				// code to clip the ray and
				// continue the ray-cast to the next fixture. WARNING: do not
				// assume that fixtures
				// are reported in order. However, by clipping, we can always
				// get the closest fixture.
				return fraction;
			}

			bool see_fixtures_ = true;
			bool hit_ = false;
			b2Vec2 point_{0, 0};
			b2Vec2 normal_{0, 0};
		};

		const mrpt::poses::CPose2D sensorPose = vehPose + mrpt::poses::CPose2D(scan.sensorPose);
		const b2Vec2 sensorPt = b2Vec2(sensorPose.x(), sensorPose.y());

		RayCastClosestCallback callback;
		callback.see_fixtures_ = see_fixtures_;

		// Scan size:
		ASSERT_(nRays >= 2);
		scan.resizeScanAndAssign(nRays, maxRange, false);
		double A = sensorPose.phi() + (scan.rightToLeft ? -0.5 : +0.5) * scan.aperture;
		const double AA = (scan.rightToLeft ? 1.0 : -1.0) * (scan.aperture / (nRays - 1));

		// Each thread must create its own rng:
		thread_local mrpt::random::CRandomGenerator rnd;

		for (size_t i = 0; i < nRays; i++, A += AA)
		{
			const b2Vec2 endPt =
				b2Vec2(sensorPt.x + cos(A) * maxRange, sensorPt.y + sin(A) * maxRange);

			callback.hit_ = false;
			world_->getBox2DWorld()->RayCast(&callback, sensorPt, endPt);
			scan.setScanRangeValidity(i, callback.hit_);

			float range = 0;
			if (callback.hit_)
			{
				// Hit:
				range = std::sqrt(
					mrpt::square(callback.point_.x - sensorPt.x) +
					mrpt::square(callback.point_.y - sensorPt.y));
				range += rnd.drawGaussian1D_normalized() * rangeStdNoise_;
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
	world_->getTimeLogger().leave("LaserScanner.scan.2.polygons");

	// Summarize all scans in one single scan:
	// ----------------------------------------
	world_->getTimeLogger().enter("LaserScanner.scan.3.merge");

	auto lastScan = CObservation2DRangeScan::Create(scan_model_);

	lastScan->timestamp = world_->get_simul_timestamp();
	lastScan->sensorLabel = name_;

	lastScan->resizeScanAndAssign(nRays, maxRange, false);

	for (const auto& scan : lstScans)
	{
		for (size_t i = 0; i < nRays; i++)
		{
			if (scan.getScanRangeValidity(i))
			{
				lastScan->setScanRange(
					i, std::min(lastScan->getScanRange(i), scan.getScanRange(i)));
				lastScan->setScanRangeValidity(i, true);
			}
		}
	}
	world_->getTimeLogger().leave("LaserScanner.scan.3.merge");

	{
		std::lock_guard<std::mutex> csl(last_scan_cs_);
		last_scan_ = std::move(lastScan);
		last_scan2gui_ = last_scan_;
	}

	// publish as generic Protobuf (mrpt serialized) object:
	SensorBase::reportNewObservation(last_scan_, context);

	// Publish custom 2d-lidar observation type too:
	SensorBase::reportNewObservation_lidar_2d(last_scan_, context);

	gui_uptodate_ = false;
}

void LaserScanner::freeOpenGLResources()
{
	// Free fbo:
	fbo_renderer_depth_.reset();
}

void LaserScanner::simulateOn3DScene(mrpt::opengl::COpenGLScene& world3DScene)
{
	using namespace mrpt;  // _deg

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);
		if (!has_to_render_.has_value()) return;
	}

	auto tleWhole = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.2Dlidar");

	auto tle1 = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.2Dlidar.acqGuiMtx");

	tle1.stop();

	if (glCustomVisual_) glCustomVisual_->setVisibility(false);

	// Start making a copy of the pattern observation:
	auto curObs = mrpt::obs::CObservation2DRangeScan::Create(scan_model_);

	const size_t nRays = scan_model_.getScanSize();
	const double maxRange = scan_model_.maxRange;

	curObs->timestamp = world_->get_simul_timestamp();
	curObs->sensorLabel = name_;

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

	if (!fbo_renderer_depth_)
	{
		mrpt::opengl::CFBORender::Parameters p;
		p.width = FBO_NCOLS;
		p.height = FBO_NROWS;
		p.create_EGL_context = world()->sensor_has_to_create_egl_context();

		fbo_renderer_depth_ = std::make_shared<mrpt::opengl::CFBORender>(p);
	}

	auto viewport = world3DScene.getViewport();

	auto& cam = fbo_renderer_depth_->getCamera(world3DScene);

	const auto fixedAxisConventionRot =
		mrpt::poses::CPose3D(0, 0, 0, -90.0_deg, 0.0_deg, -90.0_deg);

	const auto vehiclePose = mrpt::poses::CPose3D(vehicle_.getPose());

	// ----------------------------------------------------------
	// Decompose the 2D lidar FOV into "n" depth camera images,
	// of camModel_FOV each.
	// ----------------------------------------------------------
	const auto firstAngle = curObs->getScanAngle(0);  // wrt sensorPose
	const auto lastAngle = curObs->getScanAngle(curObs->getScanSize() - 1);
	const bool scanIsCW = (lastAngle > firstAngle);
	ASSERT_NEAR_(std::abs(lastAngle - firstAngle), curObs->aperture, 1e-3);

	const unsigned int numRenders = std::ceil((curObs->aperture / camModel_FOV) - 1e-3);
	const auto numRaysPerRender =
		mrpt::round(nRays * std::min<double>(1.0, (camModel_FOV / curObs->aperture)));

	ASSERT_(numRaysPerRender > 0);

	// Precomputed LUT of bearings to pixel coordinates:
	//                    cx - u
	//  tan(bearing) = --------------
	//                      fx
	//
	if (angleIdx2pixelIdx_.empty())
	{
		angleIdx2pixelIdx_.resize(numRaysPerRender);
		angleIdx2secant_.resize(numRaysPerRender);

		for (int i = 0; i < numRaysPerRender; i++)
		{
			const auto ang = (scanIsCW ? -1 : 1) *
							 (camModel_FOV * 0.5 - i * camModel_FOV / (numRaysPerRender - 1));

			const auto pixelIdx = mrpt::saturate_val<int>(
				mrpt::round(camModel.cx() - camModel.fx() * std::tan(ang)), 0, camModel.ncols - 1);

			angleIdx2pixelIdx_.at(i) = pixelIdx;
			angleIdx2secant_.at(i) = 1.0f / std::cos(ang);
		}
	}
	else
	{
		// sanity check:
		ASSERT_EQUAL_(angleIdx2pixelIdx_.size(), numRaysPerRender);
		ASSERT_EQUAL_(angleIdx2secant_.size(), numRaysPerRender);
	}

	// ----------------------------------------------------------
	// "DEPTH camera" to generate lidar readings:
	// ----------------------------------------------------------
	cam.set6DOFMode(true);
	cam.setProjectiveFromPinhole(camModel);

	// Disable rendering of shadows for this sensor:
#if MRPT_VERSION >= 0x270
	const bool wasShadowEnabled = viewport->isShadowCastingEnabled();
	viewport->enableShadowCasting(false);
#endif

	viewport->setViewportClipDistances(0.01, curObs->maxRange);
	mrpt::math::CMatrixFloat depthImage;

	// make owner's own body invisible?
	auto visVeh = dynamic_cast<VisualObject*>(&vehicle_);
	auto veh = dynamic_cast<VehicleBase*>(&vehicle_);
	bool formerVisVehState = true;
	if (ignore_parent_body_)
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
			firstAngle + (camModel_FOV / 2.0 + camModel_FOV * renderIdx) * (scanIsCW ? 1 : -1);

		const auto depthSensorPose =
			vehiclePose + curObs->sensorPose +
			mrpt::poses::CPose3D::FromYawPitchRoll(thisRenderMidAngle, 0.0, 0.0) +
			fixedAxisConventionRot;

		// Camera pose: vehicle + relativePoseOnVehicle:
		// Note: relativePoseOnVehicle should be (y,p,r)=(90deg,0,90deg) to make
		// the camera to look forward:
		cam.setPose(world()->applyWorldRenderOffset(depthSensorPose));

		auto tleRender =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.2Dlidar.renderSubScan");

		fbo_renderer_depth_->render_depth(world3DScene, depthImage);

		tleRender.stop();

		// Add random noise:
		if (rangeStdNoise_ > 0)
		{
			auto tleStore =
				mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.2Dlidar.noise");

			// Each thread must create its own rng:
			thread_local mrpt::random::CRandomGenerator rng;

			float* d = depthImage.data();
			const size_t N = depthImage.size();
			for (size_t i = 0; i < N; i++)
			{
				if (d[i] == 0) continue;  // it was an invalid ray return.

				const float dNoisy = d[i] + rng.drawGaussian1D(0, rangeStdNoise_);

				if (dNoisy < 0 || dNoisy > curObs->maxRange) continue;

				d[i] = dNoisy;
			}
		}

		auto tleStore =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.2Dlidar.storeObs");

		// Convert depth into range and store into scan observation:
		for (int i = 0; i < numRaysPerRender; i++)
		{
			const auto scanRayIdx = numRaysPerRender * renderIdx + i;
			// done with full scan range?
			if (scanRayIdx >= curObs->getScanSize()) break;

			const auto u = angleIdx2pixelIdx_.at(i);

			const float d = depthImage(0, u);
			const float range = d * angleIdx2secant_.at(i);

			if (range <= 0 || range >= curObs->maxRange) continue;	// invalid

			curObs->setScanRange(scanRayIdx, range);
			curObs->setScanRangeValidity(scanRayIdx, true);
		}
		tleStore.stop();
	}

	if (ignore_parent_body_)
	{
		if (visVeh) visVeh->customVisualVisible(formerVisVehState);
		if (veh) veh->chassisAndWheelsVisible(formerVisVehState);
	}

#if MRPT_VERSION >= 0x270
	viewport->enableShadowCasting(wasShadowEnabled);
#endif

	// Store generated obs:
	{
		auto tle3 =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.2Dlidar.acqObsMtx");

		std::lock_guard<std::mutex> csl(last_scan_cs_);
		last_scan_ = std::move(curObs);
		last_scan2gui_ = last_scan_;
	}

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);

		auto tlePub =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.2Dlidar.report");

		// publish as generic Protobuf (mrpt serialized) object:
		SensorBase::reportNewObservation(last_scan_, *has_to_render_);

		// Publish custom 2d-lidar observation type too:
		SensorBase::reportNewObservation_lidar_2d(last_scan_, *has_to_render_);

		tlePub.stop();

		if (glCustomVisual_) glCustomVisual_->setVisibility(true);

		gui_uptodate_ = false;

		has_to_render_.reset();
	}
}

void LaserScanner::registerOnServer(mvsim::Client& c)
{
	using namespace std::string_literals;

	SensorBase::registerOnServer(c);

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	// Topic:
	if (!publishTopic_.empty())
		c.advertiseTopic<mvsim_msgs::ObservationLidar2D>(publishTopic_ + "_scan"s);
#endif
}
