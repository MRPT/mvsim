/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
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

//#include "rapidxml_print.hpp"

#if MRPT_VERSION >= 0x270
#include <mrpt/opengl/OpenGLDepth2LinearLUTs.h>
#endif

#if MRPT_VERSION >= 0x020b04  // >=2.11.4?
#define HAVE_POINTS_XYZIRT
#endif

#if defined(HAVE_POINTS_XYZIRT)
#include <mrpt/maps/CPointsMapXYZIRT.h>
#endif

#include "xml_utils.h"

using namespace mvsim;
using namespace rapidxml;

// TODO(jlbc): Also store obs as CObservationRotatingScan??

Lidar3D::Lidar3D(Simulable& parent, const rapidxml::xml_node<char>* root) : SensorBase(parent)
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
	params["min_range"] = TParamEntry("%f", &minRange_);
	params["max_range"] = TParamEntry("%f", &maxRange_);
	params["viz_pointSize"] = TParamEntry("%f", &viz_pointSize_);
	params["ignore_parent_body"] = TParamEntry("%bool", &ignore_parent_body_);

	params["vert_fov_degrees"] = TParamEntry("%lf", &vertical_fov_);
	params["vertical_ray_angles"] = TParamEntry("%s", &vertical_ray_angles_str_);

	params["vert_nrays"] = TParamEntry("%i", &vertNumRays_);
	params["horz_nrays"] = TParamEntry("%i", &horzNumRays_);
	params["horz_resolution_factor"] = TParamEntry("%lf", &horzResolutionFactor_);
	params["vert_resolution_factor"] = TParamEntry("%lf", &vertResolutionFactor_);

	params["max_vert_relative_depth_to_interpolatate"] =
		TParamEntry("%f", &maxDepthInterpolationStepVert_);
	params["max_horz_relative_depth_to_interpolatate"] =
		TParamEntry("%f", &maxDepthInterpolationStepHorz_);

	// Parse XML params:
	parse_xmlnode_children_as_param(*root, params, varValues_);
}

void Lidar3D::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	[[maybe_unused]] const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
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
				last_scan2gui_.reset();
			}
		}
		gui_uptodate_ = true;
	}

	const mrpt::poses::CPose3D p = vehicle_.getCPose3D() + sensorPoseOnVeh_;
	const auto pp = parent()->applyWorldRenderOffset(p);

	if (glPoints_) glPoints_->setPose(pp);
	if (gl_sensor_fov_) gl_sensor_fov_->setPose(pp);
	if (gl_sensor_origin_) gl_sensor_origin_->setPose(pp);
	if (glCustomVisual_) glCustomVisual_->setPose(pp);
}

void Lidar3D::simul_pre_timestep([[maybe_unused]] const TSimulContext& context) {}

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

	// Keep sensor global pose up-to-date:
	const auto& p = vehicle_.getPose();
	const auto globalSensorPose = p + sensorPoseOnVeh_.asTPose();
	Simulable::setPose(globalSensorPose, false /*do not notify*/);
}

void Lidar3D::notifySimulableSetPose(const mrpt::math::TPose3D& newPose)
{
	// The editor has moved the sensor in global coordinates.
	// Convert back to local:
	const auto& p = vehicle_.getPose();
	sensorPoseOnVeh_ = mrpt::poses::CPose3D(newPose - p);
}

void Lidar3D::freeOpenGLResources()
{
	// Free fbo:
	fbo_renderer_depth_.reset();
}

#if MRPT_VERSION >= 0x270
// Do the log->linear conversion ourselves for this sensor,
// since only a few depth points are actually used:
// (older mrpt versions already returned the linearized depth)
constexpr int DEPTH_LOG2LIN_BITS = 20;
using depth_log2lin_t = mrpt::opengl::OpenGLDepth2LinearLUTs<DEPTH_LOG2LIN_BITS>;
#endif

static float safeInterpolateRangeImage(
	const mrpt::math::CMatrixFloat& depthImage, const float maxDepthInterpolationStepVert,
	const float maxDepthInterpolationStepHorz, const int NCOLS, const int NROWS, float v, float u
#if MRPT_VERSION >= 0x270
	,
	const depth_log2lin_t::lut_t& depth_log2lin_lut
#endif
)
{
	const int u0 = static_cast<int>(u);
	const int v0 = static_cast<int>(v);
	const int u1 = std::min(u0 + 1, NCOLS - 1);
	const int v1 = std::min(v0 + 1, NROWS - 1);

	const float uw = u - u0;
	const float vw = v - v0;

	const float raw_d00 = depthImage(v0, u0);
	const float raw_d01 = depthImage(v1, u0);
	const float raw_d10 = depthImage(v0, u1);
	const float raw_d11 = depthImage(v1, u1);

	// Linearize:
#if MRPT_VERSION >= 0x270
	// Do the log->linear conversion ourselves for this sensor,
	// since only a few depth points are actually used:

	// map d in [-1.0f,+1.0f] ==> real depth values:
	const float d00 = depth_log2lin_lut[(raw_d00 + 1.0f) * (depth_log2lin_t::NUM_ENTRIES - 1) / 2];
	const float d01 = depth_log2lin_lut[(raw_d01 + 1.0f) * (depth_log2lin_t::NUM_ENTRIES - 1) / 2];
	const float d10 = depth_log2lin_lut[(raw_d10 + 1.0f) * (depth_log2lin_t::NUM_ENTRIES - 1) / 2];
	const float d11 = depth_log2lin_lut[(raw_d11 + 1.0f) * (depth_log2lin_t::NUM_ENTRIES - 1) / 2];
#else
	// "d" is already linear depth
	const float d00 = raw_d00;
	const float d01 = raw_d01;
	const float d10 = raw_d10;
	const float d11 = raw_d11;
#endif

	// max relative range difference in u and v directions:
	const float A_u = std::max(std::abs(d00 - d10), std::abs(d01 - d11));
	const float A_v = std::max(std::abs(d00 - d01), std::abs(d10 - d11));

	const auto maxStepU = maxDepthInterpolationStepHorz * d00;
	const auto maxStepV = maxDepthInterpolationStepVert * d00;

	if (A_v < maxStepV && A_u < maxStepU)
	{
		// smooth bilinear interpolation in (u,v):
		return d00 * (1.0f - uw) * (1.0f - vw) +  //
			   d01 * (1.0f - uw) * vw +	 //
			   d10 * uw * (1.0f - vw) +	 //
			   d11 * uw * vw;
	}
	else if (A_v < maxStepV)
	{
		// Linear interpolation in "v" only:
		// Pick closest "u":
		const float d0 = uw < 0.5f ? d00 : d10;
		const float d1 = uw < 0.5f ? d01 : d11;

		return d0 * (1.0f - vw) + d1 * vw;
	}
	else if (A_u < maxStepU)
	{
		// Linear interpolation in "u" only:

		// Pick closest "v":
		const float d0 = vw < 0.5f ? d00 : d01;
		const float d1 = vw < 0.5f ? d10 : d11;

		return d0 * (1.0f - uw) + d1 * uw;
	}
	else
	{
		// too many changes in depth, do not interpolate:
		return d00;
	}
}

void Lidar3D::simulateOn3DScene(mrpt::opengl::COpenGLScene& world3DScene)
{
	using namespace mrpt;  // _deg

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);
		if (!has_to_render_.has_value()) return;
	}

	auto tleWhole = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.3Dlidar");

	auto tle1 = mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.3Dlidar.acqGuiMtx");

	tle1.stop();

	// The sensor body must be made of transparent material! :-)
	if (glCustomVisual_) glCustomVisual_->setVisibility(false);

	// Create empty observation:
	auto curObs = mrpt::obs::CObservationPointCloud::Create();
	curObs->sensorPose = sensorPoseOnVeh_;
	curObs->timestamp = world_->get_simul_timestamp();
	curObs->sensorLabel = name_;

#if defined(HAVE_POINTS_XYZIRT)
	auto curPtsPtr = mrpt::maps::CPointsMapXYZIRT::Create();
#else
	auto curPtsPtr = mrpt::maps::CSimplePointsMap::Create();
#endif

	auto& curPts = *curPtsPtr;
	curObs->pointcloud = curPtsPtr;

	// Create FBO on first use, now that we are here at the GUI / OpenGL thread.
	constexpr double camModel_hFOV = 120.01_deg;

	// This FBO is for camModel_hFOV only:
	// Minimum horz resolution=360deg /120 deg
	const int FBO_NCOLS =
		mrpt::round(horzResolutionFactor_ * horzNumRays_ / (2 * M_PI / camModel_hFOV));

	mrpt::img::TCamera camModel;
	camModel.ncols = FBO_NCOLS;
	camModel.cx(camModel.ncols / 2.0);
	camModel.fx(camModel.cx() / tan(camModel_hFOV * 0.5));	// tan(FOV/2)=cx/fx

	// Build list of vertical angles, in increasing order (first negative, below
	// horizontal plane, final ones positive, above it):
	if (vertical_ray_angles_.empty())
	{
		if (vertical_ray_angles_str_.empty())
		{
			// even distribution:
			vertical_ray_angles_.resize(vertNumRays_);
			for (int i = 0; i < vertNumRays_; i++)
			{
				vertical_ray_angles_[i] = vertical_fov_ * (-0.5 + i * 1.0 / (vertNumRays_ - 1));
			}
		}
		else
		{
			// custom distribution:
			std::vector<std::string> vertAnglesStrs;
			mrpt::system::tokenize(vertical_ray_angles_str_, " \t\r\n", vertAnglesStrs);
			ASSERT_EQUAL_(vertAnglesStrs.size(), static_cast<size_t>(vertNumRays_));
			std::set<double> angs;
			for (const auto& s : vertAnglesStrs) angs.insert(std::stod(s));
			ASSERT_EQUAL_(angs.size(), static_cast<size_t>(vertNumRays_));
			for (const auto a : angs) vertical_ray_angles_.push_back(a);
		}

		// Pass to radians:
		for (double& a : vertical_ray_angles_) a = mrpt::DEG2RAD(a);
	}

	// worst vFOV case: at each sub-scan render corner:
	// (derivation in hand notes... to be passed to a paper)
	using mrpt::square;
	const double vertFOVMax = vertical_ray_angles_.back();
	const double vertFOVMin = std::abs(vertical_ray_angles_.front());

	const int FBO_NROWS_UP = vertResolutionFactor_ * tan(vertFOVMax) *
							 sqrt(square(camModel.fx()) + square(camModel.cx()));
	const int FBO_NROWS_DOWN = vertResolutionFactor_ * tan(vertFOVMin) *
							   sqrt(square(camModel.fx()) + square(camModel.cx()));

	const int FBO_NROWS = FBO_NROWS_DOWN + FBO_NROWS_UP + 1;
	camModel.nrows = FBO_NROWS;
	camModel.cy(FBO_NROWS_UP + 1);
	camModel.fy(camModel.fx());

	if (!fbo_renderer_depth_)
	{
		mrpt::opengl::CFBORender::Parameters p;
		p.width = FBO_NCOLS;
		p.height = FBO_NROWS;
		p.create_EGL_context = world()->sensor_has_to_create_egl_context();

#if MRPT_VERSION >= 0x270
		// Do the log->linear conversion ourselves for this sensor, since only a
		// few depth points are actually used.
		p.raw_depth = true;
#endif

		fbo_renderer_depth_ = std::make_shared<mrpt::opengl::CFBORender>(p);
	}

	const size_t nCols = horzNumRays_;
	const size_t nRows = vertNumRays_;

	mrpt::math::CMatrixDouble rangeImage(nRows, nCols);
	rangeImage.setZero();  // 0=invalid (no lidar return)

	auto viewport = world3DScene.getViewport();

	// Disable rendering of shadows for this sensor:
#if MRPT_VERSION >= 0x270
	const bool wasShadowEnabled = viewport->isShadowCastingEnabled();
	viewport->enableShadowCasting(false);
#endif

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

	const unsigned int numRenders = std::ceil((aperture / camModel_hFOV) - 1e-3);
	const auto numHorzRaysPerRender =
		mrpt::round(horzNumRays_ * std::min<double>(1.0, (camModel_hFOV / aperture)));

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
				(camModel_hFOV * 0.5 - i * camModel_hFOV / (numHorzRaysPerRender - 1));

			const double cosHorzAng = std::cos(horzAng);

			const auto pixel_u = mrpt::saturate_val<float>(
				camModel.cx() - camModel.fx() * std::tan(horzAng), 0, camModel.ncols - 1);

			for (size_t j = 0; j < nRows; j++)
			{
				auto& entry = lut_[i].column[j];

				const double vertAng = vertical_ray_angles_.at(j);
				const double cosVertAng = std::cos(vertAng);

				const auto pixel_v = camModel.cy() - camModel.fy() * std::tan(vertAng) / cosHorzAng;

				// out of the simulated camera (should not happen?)
				if (pixel_v < 0 || pixel_v >= static_cast<int>(camModel.nrows)) continue;

				entry.u = pixel_u;
				entry.v = pixel_v;
				entry.depth2range = 1.0f / (cosHorzAng * cosVertAng);
			}
		}
	}
	else
	{
		// check:
		ASSERT_EQUAL_(lut_.size(), static_cast<size_t>(numHorzRaysPerRender));
		ASSERT_EQUAL_(lut_.at(0).column.size(), nRows);
	}

	// ----------------------------------------------------------
	// "DEPTH camera" to generate lidar readings:
	// ----------------------------------------------------------
	cam.set6DOFMode(true);
	cam.setProjectiveFromPinhole(camModel);

	viewport->setViewportClipDistances(minRange_, maxRange_);
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

#if MRPT_VERSION >= 0x270
	// Do the log->linear conversion ourselves for this sensor,
	// since only a few depth points are actually used:
	auto& depth_log2lin = depth_log2lin_t::Instance();
	const auto& depth_log2lin_lut = depth_log2lin.lut_from_zn_zf(minRange_, maxRange_);
#endif

	for (size_t renderIdx = 0; renderIdx < numRenders; renderIdx++)
	{
		const double thisRenderMidAngle =
			firstAngle + (camModel_hFOV / 2.0 + camModel_hFOV * renderIdx) * (scanIsCW ? 1 : -1);

		const auto thisDepthSensorPoseWrtSensor =
			mrpt::poses::CPose3D::FromYawPitchRoll(thisRenderMidAngle, 0.0, 0.0) +
			fixedAxisConventionRot;

		const auto thisDepthSensorPoseOnVeh = curObs->sensorPose + thisDepthSensorPoseWrtSensor;

		const auto thisDepthSensorPose = vehiclePose + thisDepthSensorPoseOnVeh;

		// Camera pose: vehicle + relativePoseOnVehicle:
		// Note: relativePoseOnVehicle should be (y,p,r)=(90deg,0,90deg)
		// to make the camera to look forward:
		cam.setPose(world()->applyWorldRenderOffset(thisDepthSensorPose));

		auto tleRender =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.3Dlidar.renderSubScan");

		fbo_renderer_depth_->render_depth(world3DScene, depthImage);

		tleRender.stop();

		// Add random noise:
		// Each thread must create its own rng:
		thread_local mrpt::random::CRandomGenerator rng;
		thread_local std::vector<float> noiseSeq;
		thread_local size_t noiseIdx = 0;
		constexpr size_t noiseLen = 7823;  // prime
		if (rangeStdNoise_ > 0)
		{
			if (noiseSeq.empty())
			{
				noiseSeq.reserve(noiseLen);
				for (size_t i = 0; i < noiseLen; i++)
					noiseSeq.push_back(rng.drawGaussian1D(0.0, rangeStdNoise_));
			}
		}

		auto tleStore =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.3Dlidar.storeObs");

		// Convert depth into range and store into polar range images:
		for (int i = 0; i < numHorzRaysPerRender; i++)
		{
			const int iAbs = i + numHorzRaysPerRender * renderIdx;
			if (iAbs >= rangeImage.cols()) continue;  // we don't need this image part

			for (unsigned int j = 0; j < nRows; j++)
			{
				// LUT entry:
				const auto& e = lut_.at(i).column.at(j);
				const auto u = e.u;
				const auto v = e.v;

				ASSERTDEB_LT_(u, depthImage.cols());
				ASSERTDEB_LT_(v, depthImage.rows());

				// Depth:
				float d = safeInterpolateRangeImage(
					depthImage, maxDepthInterpolationStepVert_, maxDepthInterpolationStepHorz_,
					FBO_NCOLS, FBO_NROWS, v, u
#if MRPT_VERSION >= 0x270
					,
					depth_log2lin_lut
#endif
				);

				// Add noise:
				if (d != 0)	 // invalid range
				{
					const float dNoisy = d + noiseSeq[noiseIdx++];
					if (noiseIdx >= noiseLen) noiseIdx = 0;

					if (dNoisy < 0 || dNoisy > maxRange_) continue;

					d = dNoisy;
				}

				// un-project: depth -> range:
				const float range = d * e.depth2range;

				if (range <= 0 || range >= maxRange_) continue;	 // invalid

				ASSERTDEB_LT_(j, rangeImage.rows());
				ASSERTDEB_LT_(iAbs, rangeImage.cols());

				rangeImage(j, iAbs) = range;

				// add points:
				const mrpt::math::TPoint3D pt_wrt_cam = {
					d * (u - camModel.cx()) / camModel.fx(),
					d * (v - camModel.cy()) / camModel.fy(), d};
				curPts.insertPoint(thisDepthSensorPoseWrtSensor.composePoint(pt_wrt_cam));

#if defined(HAVE_POINTS_XYZIRT)
				// Add "ring" field:
				curPtsPtr->getPointsBufferRef_ring()->push_back(j);

				// Add "timestamp" field: all to zero since we are simulating an ideal "flash"
				// lidar:
				curPtsPtr->getPointsBufferRef_timestamp()->push_back(.0);
#endif
			}
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
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.3Dlidar.acqObsMtx");

		std::lock_guard<std::mutex> csl(last_scan_cs_);
		last_scan_ = std::move(curObs);
		last_scan2gui_ = last_scan_;
	}

	{
		auto lckHasTo = mrpt::lockHelper(has_to_render_mtx_);

		auto tlePub =
			mrpt::system::CTimeLoggerEntry(world_->getTimeLogger(), "sensor.3Dlidar.report");

		SensorBase::reportNewObservation(last_scan_, *has_to_render_);

		tlePub.stop();

		if (glCustomVisual_) glCustomVisual_->setVisibility(true);

		gui_uptodate_ = false;

		has_to_render_.reset();
	}
}
