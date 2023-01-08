/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
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
	gui_uptodate_ = false;

	SensorBase::loadConfigFrom(root);
	SensorBase::make_sure_we_have_a_name("laser");

	// Other scalar params:
	TParameterDefinitions params;
	params["pose_3d"] = TParamEntry("%pose3d", &sensorPoseOnVeh_);
	params["range_std_noise"] = TParamEntry("%lf", &rangeStdNoise_);
	params["sensor_period"] = TParamEntry("%lf", &sensor_period_);
	params["max_range"] = TParamEntry("%f", &maxRange_);
	params["viz_pointSize"] = TParamEntry("%f", &viz_pointSize_);
	params["ignore_parent_body"] = TParamEntry("%bool", &ignore_parent_body_);

	params["vert_fov_degrees"] = TParamEntry("%lf_deg", &vertical_fov_);
	params["vert_nrays"] = TParamEntry("%i", &vertNumRays_);
	params["horz_nrays"] = TParamEntry("%i", &horzNumRays_);

	params["fbo_nrows"] = TParamEntry("%i", &fbo_nrows_);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues_);
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
	if (!glPoints_ && glVizSensors)
	{
		glPoints_ = mrpt::opengl::CPointCloudColoured::Create();
		glPoints_->setPointSize(viz_pointSize_);
		glPoints_->setLocalRepresentativePoint({0, 0, 0.10f});

		glVizSensors->insert(glPoints_);
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

		MRPT_TODO("render 3D lidar FOV");
#if 0
		auto fovScan = mrpt::opengl::CPlanarLaserScan::Create();
		gl_sensor_fov_->insert(fovScan);
#endif
		gl_sensor_fov_->setVisibility(false);
		viz->get().insert(gl_sensor_fov_);
		SensorBase::RegisterSensorFOVViz(gl_sensor_fov_);
	}

	if (!gui_uptodate_ && glVizSensors->isVisible())
	{
		{
			std::lock_guard<std::mutex> csl(last_scan_cs_);
			if (last_scan2gui_ && last_scan2gui_->pointcloud)
			{
				glPoints_->loadFromPointsMap(last_scan2gui_->pointcloud.get());
				gl_sensor_origin_corner_->setPose(last_scan2gui_->sensorPose);

				last_scan2gui_.reset();
			}
		}
		gui_uptodate_ = true;
	}

	const mrpt::poses::CPose3D p = vehicle_.getCPose3D() + sensorPoseOnVeh_;

	if (glPoints_) glPoints_->setPose(p);
	if (gl_sensor_fov_) gl_sensor_fov_->setPose(p);
	if (gl_sensor_origin_) gl_sensor_origin_->setPose(p);
	if (glCustomVisual_) glCustomVisual_->setPose(p);
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
}

void Lidar3D::freeOpenGLResources()
{
	// Free fbo:
	fbo_renderer_depth_.reset();
}

void Lidar3D::simulateOn3DScene(mrpt::opengl::COpenGLScene& world3DScene)
{
	using namespace mrpt;  // _deg

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);
		if (!has_to_render_.has_value()) return;
	}

	auto tleWhole = mrpt::system::CTimeLoggerEntry(
		world_->getTimeLogger(), "sensor.3Dlidar");

	auto tle1 = mrpt::system::CTimeLoggerEntry(
		world_->getTimeLogger(), "sensor.3Dlidar.acqGuiMtx");

	tle1.stop();

	// The sensor body must be made of transparent material! :-)
	if (glCustomVisual_) glCustomVisual_->setVisibility(false);

	// Create empty observation:
	auto curObs = mrpt::obs::CObservationPointCloud::Create();
	curObs->sensorPose = sensorPoseOnVeh_;
	curObs->timestamp = world_->get_simul_timestamp();
	curObs->sensorLabel = name_;

	auto curPtsPtr = mrpt::maps::CSimplePointsMap::Create();
	auto& curPts = *curPtsPtr;
	curObs->pointcloud = curPtsPtr;

	// Create FBO on first use, now that we are here at the GUI / OpenGL thread.
	constexpr double camModel_hFOV = 120.01_deg;
	const int FBO_NROWS = fbo_nrows_;
	// This FBO is for camModel_hFOV only:
	const int FBO_NCOLS = horzNumRays_;

	// worst vFOV case: at each sub-scan render corner:
	const double camModel_vFOV =
		std::min(179.0_deg, 2.05 * atan2(1.0, 1.0 / cos(camModel_hFOV * 0.5)));

	mrpt::img::TCamera camModel;
	camModel.ncols = FBO_NCOLS;
	camModel.nrows = FBO_NROWS;
	camModel.cx(camModel.ncols / 2.0);
	camModel.cy(camModel.nrows / 2.0);
	camModel.fx(camModel.cx() / tan(camModel_hFOV * 0.5));	// tan(FOV/2)=cx/fx
	camModel.fy(camModel.cy() / tan(camModel_vFOV * 0.5));

	if (!fbo_renderer_depth_)
	{
		mrpt::opengl::CFBORender::Parameters p;
		p.width = FBO_NCOLS;
		p.height = FBO_NROWS;
		p.create_EGL_context = world()->sensor_has_to_create_egl_context();

		fbo_renderer_depth_ = std::make_shared<mrpt::opengl::CFBORender>(p);
	}

	const size_t nCols = horzNumRays_;
	const size_t nRows = vertNumRays_;

	mrpt::math::CMatrixDouble rangeImage(nRows, nCols);
	rangeImage.setZero();  // 0=invalid (no lidar return)

	auto viewport = world3DScene.getViewport();

	auto& cam = fbo_renderer_depth_->getCamera(world3DScene);

	const auto fixedAxisConventionRot =
		mrpt::poses::CPose3D(0, 0, 0, -90.0_deg, 0.0_deg, -90.0_deg);

	const auto vehiclePose = mrpt::poses::CPose3D(vehicle_.getPose());

	// ----------------------------------------------------------
	// Decompose the horizontal lidar FOV into "n" depth images,
	// of camModel_hFOV each.
	// ----------------------------------------------------------
	ASSERT_GT_(horzNumRays_, 1);
	ASSERT_GT_(vertNumRays_, 1);

	constexpr bool scanIsCW = false;
	constexpr double aperture = 2 * M_PI;

	const double firstAngle = -aperture * 0.5;

	const unsigned int numRenders =
		std::ceil((aperture / camModel_hFOV) - 1e-3);
	const auto numHorzRaysPerRender = mrpt::round(
		horzNumRays_ * std::min<double>(1.0, (camModel_hFOV / aperture)));

	ASSERT_(numHorzRaysPerRender > 0);

	// Precomputed LUT of bearings to pixel coordinates:
	//                         cx - u
	//  tan(horzBearing) = --------------
	//                          fx
	//
	//                             cy - v
	//  tan(vertBearing) = -------------------------
	//                      fy / cos(horzBearing)
	//
	if (lut_.empty())
	{
		lut_.resize(numHorzRaysPerRender);

		for (int i = 0; i < numHorzRaysPerRender; i++)
		{
			lut_[i].column.resize(nRows);

			const double horzAng =
				(scanIsCW ? -1 : 1) *
				(camModel_hFOV * 0.5 -
				 i * camModel_hFOV / (numHorzRaysPerRender - 1));

			const double cosHorzAng = std::cos(horzAng);

			const auto pixel_u = mrpt::saturate_val<int>(
				mrpt::round(camModel.cx() - camModel.fx() * std::tan(horzAng)),
				0, camModel.ncols - 1);

			for (size_t j = 0; j < nRows; j++)
			{
				auto& entry = lut_[i].column[j];

				const auto vertAng =
					-vertical_fov_ * 0.5 + j * vertical_fov_ / (nRows - 1);

				const double cosVertAng = std::cos(vertAng);

				const auto pixel_v = mrpt::round(
					camModel.cy() -
					camModel.fy() * std::tan(vertAng) / cosHorzAng);

				// out of the simulated camera (should not happen?)
				if (pixel_v < 0 || pixel_v >= static_cast<int>(camModel.nrows))
					continue;

				entry.u = pixel_u;
				entry.v = pixel_v;
				entry.depth2range = 1.0f / (cosHorzAng * cosVertAng);
			}
		}
	}
	else
	{
		// check:
		ASSERT_EQUAL_(lut_.size(), numHorzRaysPerRender);
		ASSERT_EQUAL_(lut_.at(0).column.size(), nRows);
	}

	// ----------------------------------------------------------
	// "DEPTH camera" to generate lidar readings:
	// ----------------------------------------------------------
	cam.set6DOFMode(true);
	cam.setProjectiveFromPinhole(camModel);

	viewport->setViewportClipDistances(0.01, maxRange_);
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
			firstAngle + (camModel_hFOV / 2.0 + camModel_hFOV * renderIdx) *
							 (scanIsCW ? 1 : -1);

		const auto thisDepthSensorPoseWrtSensor =
			mrpt::poses::CPose3D::FromYawPitchRoll(
				thisRenderMidAngle, 0.0, 0.0) +
			fixedAxisConventionRot;

		const auto thisDepthSensorPoseOnVeh =
			curObs->sensorPose + thisDepthSensorPoseWrtSensor;

		const auto thisDepthSensorPose = vehiclePose + thisDepthSensorPoseOnVeh;

		// Camera pose: vehicle + relativePoseOnVehicle:
		// Note: relativePoseOnVehicle should be (y,p,r)=(90deg,0,90deg)
		// to make the camera to look forward:
		cam.setPose(thisDepthSensorPose);

		auto tleRender = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.3Dlidar.renderSubScan");

		fbo_renderer_depth_->render_depth(world3DScene, depthImage);

		tleRender.stop();

		// Add random noise:
		if (rangeStdNoise_ > 0)
		{
			// Each thread must create its own rng:
			thread_local mrpt::random::CRandomGenerator rng;
			thread_local std::vector<float> noiseSeq;
			thread_local size_t noiseIdx = 0;
			constexpr size_t noiseLen = 7823;  // prime
			if (noiseSeq.empty())
			{
				noiseSeq.reserve(noiseLen);
				for (size_t i = 0; i < noiseLen; i++)
					noiseSeq.push_back(rng.drawGaussian1D(0.0, rangeStdNoise_));
			}

			auto tleStore = mrpt::system::CTimeLoggerEntry(
				world_->getTimeLogger(), "sensor.3Dlidar.noise");

			float* d = depthImage.data();
			const size_t N = depthImage.size();
			for (size_t i = 0; i < N; i++)
			{
				if (d[i] == 0) continue;  // it was an invalid ray return.

				const float dNoisy = d[i] + noiseSeq[noiseIdx++];
				if (noiseIdx >= noiseLen) noiseIdx = 0;

				if (dNoisy < 0 || dNoisy > maxRange_) continue;

				d[i] = dNoisy;
			}
		}

		auto tleStore = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.3Dlidar.storeObs");

		// Convert depth into range and store into polar range images:
		for (int i = 0; i < numHorzRaysPerRender; i++)
		{
			const int iAbs = i + numHorzRaysPerRender * renderIdx;
			if (iAbs >= rangeImage.cols())
				continue;  // we don't need this image part

			for (unsigned int j = 0; j < nRows; j++)
			{
				// LUT entry:
				const auto& e = lut_.at(i).column.at(j);
				const auto u = e.u;
				const auto v = e.v;

				ASSERTDEB_LT_(u, depthImage.cols());
				ASSERTDEB_LT_(v, depthImage.rows());

				const float d = depthImage(v, u);
				const float range = d * e.depth2range;

				if (range <= 0 || range >= maxRange_) continue;	 // invalid

				ASSERTDEB_LT_(j, rangeImage.rows());
				ASSERTDEB_LT_(iAbs, rangeImage.cols());

				rangeImage(j, iAbs) = range;

				// add points:
				const mrpt::math::TPoint3D pt_wrt_cam = {
					d * (u - camModel.cx()) / camModel.fx(),
					d * (v - camModel.cy()) / camModel.fy(), d};
				curPts.insertPoint(
					thisDepthSensorPoseWrtSensor.composePoint(pt_wrt_cam));
			}
		}
		tleStore.stop();
	}

	if (ignore_parent_body_)
	{
		if (visVeh) visVeh->customVisualVisible(formerVisVehState);
		if (veh) veh->chassisAndWheelsVisible(formerVisVehState);
	}

	// Store generated obs:
	{
		auto tle3 = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.3Dlidar.acqObsMtx");

		std::lock_guard<std::mutex> csl(last_scan_cs_);
		last_scan_ = std::move(curObs);
		last_scan2gui_ = last_scan_;
	}

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);

		auto tlePub = mrpt::system::CTimeLoggerEntry(
			world_->getTimeLogger(), "sensor.3Dlidar.report");

		SensorBase::reportNewObservation(last_scan_, *has_to_render_);

		tlePub.stop();

		if (glCustomVisual_) glCustomVisual_->setVisibility(true);

		gui_uptodate_ = false;

		has_to_render_.reset();
	}
}
