/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "CameraSimulator.h"

#include <mrpt/core/exceptions.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>
#include <mrpt/version.h>
#include <mvsim/Block.h>
#include <mvsim/Sensors/CameraSensor.h>
#include <mvsim/Sensors/SensorBase.h>
#include <mvsim/World.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "TimeUtils.h"

using namespace mvsim_dataset_gen;

namespace
{
/** The depth camera frame looks along +X, its optical frame along +Z. */
mrpt::poses::CPose3D depthOpticalRot()
{
	return mrpt::poses::CPose3D::FromYawPitchRoll(-M_PI / 2, 0.0, -M_PI / 2);
}

/** The camera to give the MRPT renderer for images that follow `cam` in the
 * OpenCV / ROS camera_info convention (the center of pixel (u,v) at image
 * coordinates (u,v)): MRPT renders the center of pixel (u,v) at
 * (u+0.5,v+0.5), both in the pinhole projection and the lens distortion. */
mrpt::img::TCamera renderCamera(const mrpt::img::TCamera& cam)
{
	auto c = cam;
	c.cx(cam.cx() + 0.5);
	c.cy(cam.cy() + 0.5);
	return c;
}

/** Renderers return raw window depths in [0,1] (see linearizeDepth()). */
std::shared_ptr<mrpt::opengl::CFBORender> makeRenderer(
	const mrpt::img::TCamera& cam, bool createEglContext)
{
	mrpt::opengl::CFBORender::Parameters p;
	p.width = cam.ncols;
	p.height = cam.nrows;
	p.create_EGL_context = createEglContext;
	p.raw_depth = true;
	auto r = std::make_shared<mrpt::opengl::CFBORender>(p);
	r->setCamera(mrpt::viz::CCamera());
	return r;
}

/** Lens distortion and pixel noise, applied by the renderer (GPU). */
void setImageFormation(
	[[maybe_unused]] mrpt::opengl::CFBORender& r, const CameraImageModel& m,
	[[maybe_unused]] uint32_t noiseSeed)
{
#if MRPT_VERSION >= MIN_MRPT_VERSION_CAMERA_DISTORTION
	if (m.camera.distortion != mrpt::img::DistortionModel::none)
	{
		r.setLensDistortion(renderCamera(m.camera));
	}
	r.setRGBNoise(static_cast<float>(m.noiseStd), noiseSeed);
#else
	ASSERTMSG_(
		m.camera.distortion == mrpt::img::DistortionModel::none && m.noiseStd == 0,
		"Camera lens distortion and image noise require MRPT >= 3.6.0");
#endif
}

/** Points the renderer camera from the optical frame at `opticalPoseOnVehicle`
 * of the vehicle at time `t`. */
void placeCamera(
	mrpt::opengl::CFBORender& r, const CameraScene& scene, const TrajectorySource& traj,
	const mrpt::img::TCamera& camera, const mrpt::poses::CPose3D& opticalPoseOnVehicle, double t)
{
	auto& cam = r.getCameraOverride();
	cam.set6DOFMode(true);
	cam.setProjectiveFromPinhole(renderCamera(camera));
	cam.setPose(scene.world().applyWorldRenderOffset(traj.poseAt(t) + opticalPoseOnVehicle));
}

/** Raw window depths [0,1] of a perspective render with clip distances
 * (zn, zf) to depths along the optical axis [m], 0 for no return. In double
 * precision: the renderer's own conversion goes through a look-up table
 * whose quantization makes depths read short (about 4 mm at 14 m). */
void linearizeDepth(mrpt::math::CMatrixFloat& depth, double zn, double zf)
{
	for (int r = 0; r < depth.rows(); r++)
	{
		for (int c = 0; c < depth.cols(); c++)
		{
			float& d = depth(r, c);
			if (d >= 1.0f)
			{
				d = 0.0f;
				continue;
			}
			const double ndc = 2.0 * d - 1.0;
			d = static_cast<float>(2.0 * zn * zf / (zf + zn - ndc * (zf - zn)));
		}
	}
}

/** Renders an RGB image (with a rolling shutter, band by band). */
void renderRgb(
	mrpt::opengl::CFBORender& r, CameraScene& scene, const TrajectorySource& traj,
	const CameraImageModel& m, double t, mrpt::img::CImage& out)
{
	auto viewport = scene.scene().getViewport();
	viewport->setViewportClipDistances(m.clipMin, m.clipMax);
	// Fewer shadow cascades than a GUI view, as camera sensors:
	const mvsim::ViewportShadowSettingsGuard shadowGuard(*viewport);
	viewport->lightParameters().shadow_cascades =
		static_cast<uint8_t>(scene.world().sensor_shadow_cascades());

	const auto renderAt = [&](double tr, mrpt::img::CImage& img)
	{
		scene.setTime(tr);
		placeCamera(r, scene, traj, m.camera, m.poseOnVehicle, tr);
		r.render_RGB(scene.scene(), img);
	};

	if (m.rollingShutterLineTime <= 0)
	{
		renderAt(t, out);
		return;
	}
	// Rolling shutter: row `i` is exposed at t + i * lineTime. Each band of
	// rows comes from the pose at the time of its middle row:
	const int rows = static_cast<int>(m.camera.nrows);
	const int bands = std::clamp(m.rollingShutterBands, 1, rows);
	mrpt::img::CImage band;
	for (int k = 0; k < bands; k++)
	{
		const int row0 = k * rows / bands;
		const int row1 = (k + 1) * rows / bands;
		renderAt(t + 0.5 * (row0 + row1 - 1) * m.rollingShutterLineTime, band);
		if (k == 0)
		{
			out = band.makeDeepCopy();
			continue;
		}
		const size_t rowBytes = band.getWidth() * band.channels();
		for (int i = row0; i < row1; i++)
		{
			std::memcpy(out.ptrLine<uint8_t>(i), band.ptrLine<uint8_t>(i), rowBytes);
		}
	}
}

/** Writes `img` as a PNG file (in the background) and makes it an
 * externally stored image. */
void storePng(mrpt::img::CImage& img, const ImageFiles& files, const std::string& name)
{
	files.writer->write(img.makeDeepCopy(), files.imagesDir + "/" + name);
	img.setExternalStorage(name);
}
}  // namespace

// ---------------------------------------------------------------------------
CameraScene::CameraScene(mvsim::World& world, std::vector<MovingBlock> movingBlocks)
	: world_(world), movingBlocks_(std::move(movingBlocks))
{
	for (const auto& e : world.getListOfWorldElements())
	{
		e->guiUpdate(viz_, physical_);
	}
	for (const auto& nameBlock : world.getListOfBlocks())
	{
		nameBlock.second->guiUpdate(viz_, physical_);
	}
	world.applyLightOptionsTo(*physical_.getViewport());
}

void CameraScene::setTime(double tEpochSeconds)
{
	for (const auto& [block, traj] : movingBlocks_)
	{
		block->setPose(traj->poseAt(tEpochSeconds).asTPose());
		block->guiUpdate(viz_, physical_);
	}
}

ImageWriter::ImageWriter(size_t threads)
{
	try
	{
		for (size_t i = 0; i < std::max<size_t>(1, threads); i++)
		{
			threads_.emplace_back([this]() { worker(); });
		}
	}
	catch (...)
	{
		// The destructor does not run: join the threads already started.
		stopWorkers();
		throw;
	}
}

ImageWriter::~ImageWriter() { stopWorkers(); }

void ImageWriter::stopWorkers()
{
	{
		std::lock_guard<std::mutex> lck(mtx_);
		stop_ = true;
	}
	cv_.notify_all();
	for (auto& t : threads_)
	{
		t.join();
	}
	threads_.clear();
}

void ImageWriter::write(mrpt::img::CImage img, const std::string& path)
{
	std::unique_lock<std::mutex> lck(mtx_);
	// Bounded queue: at most a few images per worker wait to be written.
	cv_.wait(lck, [this]() { return queue_.size() < 4 * threads_.size() || !error_.empty(); });
	if (!error_.empty())
	{
		THROW_EXCEPTION(error_);
	}
	queue_.emplace_back(std::move(img), path);
	cv_.notify_all();
}

void ImageWriter::finish()
{
	std::unique_lock<std::mutex> lck(mtx_);
	cv_.wait(lck, [this]() { return (queue_.empty() && busy_ == 0) || !error_.empty(); });
	if (!error_.empty())
	{
		THROW_EXCEPTION(error_);
	}
}

void ImageWriter::worker()
{
	for (;;)
	{
		std::pair<mrpt::img::CImage, std::string> job;
		{
			std::unique_lock<std::mutex> lck(mtx_);
			cv_.wait(lck, [this]() { return stop_ || !queue_.empty(); });
			if (queue_.empty())
			{
				return;	 // stopping
			}
			job = std::move(queue_.front());
			queue_.pop_front();
			busy_++;
		}
		// Exceptions must not leave the thread (that would terminate the
		// program): they are reported by write() and finish().
		std::string error;
		try
		{
			if (!job.first.saveToFile(job.second))
			{
				error = "Cannot write image '" + job.second + "'";
			}
		}
		catch (const std::exception& e)
		{
			error = "Cannot write image '" + job.second + "': " + e.what();
		}
		{
			std::lock_guard<std::mutex> lck(mtx_);
			busy_--;
			if (!error.empty() && error_.empty())
			{
				error_ = error;
			}
		}
		cv_.notify_all();
	}
}

std::string ImageFiles::name(const std::string& suffixAndExt) const
{
	return label + mrpt::format("_%06zu", nextIndex) + suffixAndExt;
}

// ---------------------------------------------------------------------------
RgbCameraSim::RgbCameraSim(
	const CameraImageModel& model, const std::string& label, const std::string& imagesDir,
	std::shared_ptr<ImageWriter> writer, uint32_t noiseSeed, bool createEglContext)
	: model_(model), files_{imagesDir, label, std::move(writer), 0}
{
	renderer_ = makeRenderer(model_.camera, createEglContext);
	setImageFormation(*renderer_, model_, noiseSeed);
}

mrpt::obs::CObservationImage::Ptr RgbCameraSim::simulate(
	CameraScene& scene, const TrajectorySource& traj, double tEpochSeconds)
{
	auto obs = mrpt::obs::CObservationImage::Create();
	obs->timestamp = timeFromSeconds(tEpochSeconds);
	obs->sensorLabel = files_.label;
	obs->cameraPose = model_.poseOnVehicle;
	obs->cameraParams = model_.camera;
	mrpt::system::CTicTac tic;
	renderRgb(*renderer_, scene, traj, model_, tEpochSeconds, obs->image);
	timing_.render += tic.Tac();
	tic.Tic();
	storePng(obs->image, files_, files_.name(".png"));
	timing_.write += tic.Tac();
	timing_.frames++;
	files_.nextIndex++;
	return obs;
}

// ---------------------------------------------------------------------------
DepthCameraSim::DepthCameraSim(
	const mvsim::DepthCameraSensor& sensor, const std::string& imagesDir,
	std::shared_ptr<ImageWriter> writer, uint32_t noiseSeed, double noiseScale,
	bool createEglContext)
	: pattern_(sensor.sensorParams()),
	  files_{imagesDir, sensor.getName(), std::move(writer), 0},
	  rng_(noiseSeed)
{
	pattern_.sensorLabel = sensor.getName();
	const auto& opts = sensor.rgbImageOptions();

	rgb_.camera = pattern_.cameraParamsIntensity;
	rgb_.poseOnVehicle = pattern_.sensorPose + pattern_.relativePoseIntensityWRTDepth;
	rgb_.clipMin = sensor.rgbClipMin();
	rgb_.clipMax = sensor.rgbClipMax();
	rgb_.noiseStd = opts.imageNoiseStd * noiseScale;
	rgb_.rollingShutterLineTime = opts.rollingShutterLineTime;
	rgb_.rollingShutterBands = opts.rollingShutterBands;

	depth_.camera = pattern_.cameraParams;
	depth_.poseOnVehicle = pattern_.sensorPose + depthOpticalRot();
	depth_.clipMin = sensor.depthClipMin();
	depth_.clipMax = sensor.depthClipMax();
	depth_.unit = pattern_.rangeUnits;
	depth_.noiseStd = sensor.depthNoiseSigma() * noiseScale;
	if (noiseScale > 0)
	{
		depth_.errors = sensor.depthErrors();
	}
	// Range images are 16-bit: every depth in range must fit.
	ASSERTMSG_(
		!sensor.senseDepth() || depth_.clipMax / depth_.unit <= 65535.0,
		"'" + sensor.getName() +
			"': depth_clip_max / depth_resolution must be at most 65535 (16-bit depth images)");

	// One render gives both images if both cameras see the same view, as in
	// the interactive simulator:
	const auto& rc = rgb_.camera;
	const auto& dc = depth_.camera;
	const double relPoseError =
		(pattern_.relativePoseIntensityWRTDepth - depthOpticalRot()).asVectorVal().norm();
	singlePass_ = sensor.senseRgb() && sensor.senseDepth() && rc.ncols == dc.ncols &&
				  rc.nrows == dc.nrows && rc.intrinsicParams == dc.intrinsicParams &&
				  rc.distortion == mrpt::img::DistortionModel::none && relPoseError < 1e-6 &&
				  rgb_.rollingShutterLineTime <= 0 && rgb_.clipMin == depth_.clipMin &&
				  rgb_.clipMax >= depth_.clipMax;

	bool createContext = createEglContext;
	if (sensor.senseRgb())
	{
		rgbRenderer_ = makeRenderer(rgb_.camera, createContext);
		setImageFormation(*rgbRenderer_, rgb_, noiseSeed);
		createContext = false;
	}
	if (sensor.senseDepth() && !singlePass_)
	{
		depthRenderer_ = makeRenderer(depth_.camera, createContext);
	}
}

mrpt::obs::CObservation3DRangeScan::Ptr DepthCameraSim::simulate(
	CameraScene& scene, const TrajectorySource& traj, double tEpochSeconds)
{
	auto obs = mrpt::obs::CObservation3DRangeScan::Create(pattern_);
	obs->timestamp = timeFromSeconds(tEpochSeconds);
	obs->hasIntensityImage = false;
	obs->hasRangeImage = false;
	auto viewport = scene.scene().getViewport();
	mrpt::system::CTicTac tic;

	if (singlePass_)
	{
		viewport->setViewportClipDistances(rgb_.clipMin, rgb_.clipMax);
		const mvsim::ViewportShadowSettingsGuard shadowGuard(*viewport);
		viewport->lightParameters().shadow_cascades =
			static_cast<uint8_t>(scene.world().sensor_shadow_cascades());
		scene.setTime(tEpochSeconds);
		placeCamera(*rgbRenderer_, scene, traj, rgb_.camera, rgb_.poseOnVehicle, tEpochSeconds);
		rgbRenderer_->render_RGBD(scene.scene(), obs->intensityImage, depthImage_);
		linearizeDepth(depthImage_, rgb_.clipMin, rgb_.clipMax);
		obs->hasIntensityImage = true;
	}
	else
	{
		if (rgbRenderer_)
		{
			renderRgb(*rgbRenderer_, scene, traj, rgb_, tEpochSeconds, obs->intensityImage);
			obs->hasIntensityImage = true;
		}
		if (depthRenderer_)
		{
			viewport->setViewportClipDistances(depth_.clipMin, depth_.clipMax);
			// Shadows do not affect depth images:
			const mvsim::ViewportShadowSettingsGuard shadowGuard(*viewport);
			viewport->enableShadowCasting(false);
			scene.setTime(tEpochSeconds);
			placeCamera(
				*depthRenderer_, scene, traj, depth_.camera, depth_.poseOnVehicle, tEpochSeconds);
			depthRenderer_->render_depth(scene.scene(), depthImage_);
			linearizeDepth(depthImage_, depth_.clipMin, depth_.clipMax);
		}
	}

	if (singlePass_ || depthRenderer_)
	{
		// Depths beyond the depth camera range (possible with a single render,
		// clipped at the RGB range) are no return, also after the errors:
		if (depth_.errors.enabled())
		{
			depth_.errors.apply(depthImage_, depth_.camera, depth_.clipMax);
		}
		// Gaussian noise (one draw per pixel, so the stream does not depend on
		// the image contents), then range image units:
		std::normal_distribution<double> noise(0.0, 1.0);
		const int nRows = static_cast<int>(depthImage_.rows());
		const int nCols = static_cast<int>(depthImage_.cols());
		obs->rangeImage_setSize(nRows, nCols);
		const double maxUnits = std::min(65535.0, depth_.clipMax / depth_.unit);
		for (int r = 0; r < nRows; r++)
		{
			for (int c = 0; c < nCols; c++)
			{
				const double n = depth_.noiseStd > 0 ? depth_.noiseStd * noise(rng_) : 0.0;
				const double d = depthImage_(r, c);
				uint16_t v = 0;
				if (d > 0 && d <= depth_.clipMax)
				{
					const double units = std::round((d + n) / depth_.unit);
					// A valid return never becomes "no return" (0):
					v = static_cast<uint16_t>(std::clamp(units, 1.0, maxUnits));
				}
				obs->rangeImage(r, c) = v;
			}
		}
		obs->hasRangeImage = true;
		obs->range_is_depth = true;
	}

	timing_.render += tic.Tac();
	tic.Tic();
	if (obs->hasIntensityImage)
	{
		storePng(obs->intensityImage, files_, files_.name(".png"));
	}
	if (obs->hasRangeImage)
	{
		// MRPT's external range image format. With an explicit extension: MRPT
		// replaces any extension with .bin, and a label may contain dots.
		obs->rangeImage_convertToExternalStorage(files_.name("_depth.bin"), files_.imagesDir);
	}
	timing_.write += tic.Tac();
	timing_.frames++;
	files_.nextIndex++;
	return obs;
}
