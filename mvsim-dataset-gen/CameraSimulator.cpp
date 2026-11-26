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
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

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
	const int bands = m.rollingShutterBands;
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

/** Image files are named after the sensor, so its name must be a single
 * file name component. */
void checkLabelIsFileName(const std::string& label)
{
	ASSERTMSG_(
		!label.empty() && label != "." && label != ".." &&
			label.find_first_of("/\\") == std::string::npos,
		"Camera sensor name '" + label +
			"' cannot name its image files: it must not contain '/' or '\\', nor be '.' or "
			"'..'");
}

/** At most one band per image row: the band count that is rendered. */
void limitRollingShutterBands(CameraImageModel& m)
{
	m.rollingShutterBands = std::min(m.rollingShutterBands, static_cast<int>(m.camera.nrows));
}

/** Writes `img` as a PNG file (in the background) and makes it an
 * externally stored image. */
void storePng(mrpt::img::CImage& img, const ImageFiles& files, const std::string& name)
{
	files.writer->write(img.makeDeepCopy(), files.imagesDir + "/" + name);
	img.setExternalStorage(name);
}

/** Kannala-Brandt lens: the distorted angle theta_d of a ray at `theta` [rad]
 * from the optical axis, theta (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 +
 * k4 theta^8), and its derivative. */
double kbThetaD(const mrpt::img::TCamera& c, double theta)
{
	const double t2 = theta * theta;
	return theta * (1.0 + t2 * (c.k1() + t2 * (c.k2() + t2 * (c.k3() + t2 * c.k4()))));
}

double kbThetaDDerivative(const mrpt::img::TCamera& c, double theta)
{
	const double t2 = theta * theta;
	return 1.0 + t2 * (3 * c.k1() + t2 * (5 * c.k2() + t2 * (7 * c.k3() + t2 * 9 * c.k4())));
}

/** The largest ray angle [rad] of the lens: where its model stops being
 * monotonic, at most pi, and at most half the image circle, if any. */
double kbMaxTheta(const mrpt::img::TCamera& c, double imageCircleFovDeg)
{
	constexpr int kSteps = 3600;
	double maxTheta = M_PI;
	for (int i = 1; i <= kSteps; i++)
	{
		const double theta = M_PI * i / kSteps;
		if (kbThetaDDerivative(c, theta) <= 0)
		{
			maxTheta = M_PI * (i - 1) / kSteps;
			break;
		}
	}
	if (imageCircleFovDeg > 0)
	{
		maxTheta = std::min(maxTheta, 0.5 * mrpt::DEG2RAD(imageCircleFovDeg));
	}
	return maxTheta;
}

/** The six faces of a cube around the optical center: the rotation of each
 * face optical frame (columns: its +X, +Y, +Z axes in the camera optical
 * frame): front (+Z), right (+X), left (-X), down (+Y), up (-Y), back (-Z). */
std::array<mrpt::math::CMatrixDouble33, 6> cubeFaceRotations()
{
	const auto make = [](const std::array<double, 9>& rowMajor)
	{
		mrpt::math::CMatrixDouble33 R;
		for (int r = 0; r < 3; r++)
		{
			for (int c = 0; c < 3; c++)
			{
				R(r, c) = rowMajor[3 * r + c];
			}
		}
		return R;
	};
	return {
		make({1, 0, 0, 0, 1, 0, 0, 0, 1}),	// front
		make({0, 0, 1, 0, 1, 0, -1, 0, 0}),	 // right: x=-Z, y=+Y, z=+X
		make({0, 0, -1, 0, 1, 0, 1, 0, 0}),	 // left: x=+Z, y=+Y, z=-X
		make({1, 0, 0, 0, 0, 1, 0, -1, 0}),	 // down: x=+X, y=-Z, z=+Y
		make({1, 0, 0, 0, 0, -1, 0, 1, 0}),	 // up: x=+X, y=+Z, z=-Y
		make({-1, 0, 0, 0, 1, 0, 0, 0, -1})	 // back: x=-X, y=+Y, z=-Z
	};
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
bool CubeFisheyeRenderer::pixelRay(
	const mrpt::img::TCamera& cam, double maxThetaRad, double u, double v,
	mrpt::math::TPoint3D& ray)
{
	const double xd = (u - cam.cx()) / cam.fx();
	const double yd = (v - cam.cy()) / cam.fy();
	const double rd = std::hypot(xd, yd);
	if (rd < 1e-12)
	{
		ray = {0.0, 0.0, 1.0};
		return true;
	}
	// theta_d grows monotonically up to maxThetaRad: beyond it, nothing.
	if (rd > kbThetaD(cam, maxThetaRad))
	{
		return false;
	}
	// Bisection, to double precision:
	double lo = 0;
	double hi = maxThetaRad;
	for (int i = 0; i < 64 && hi - lo > 1e-15; i++)
	{
		const double mid = 0.5 * (lo + hi);
		(kbThetaD(cam, mid) < rd ? lo : hi) = mid;
	}
	const double theta = 0.5 * (lo + hi);
	const double s = std::sin(theta) / rd;
	ray = {xd * s, yd * s, std::cos(theta)};
	return true;
}

CubeFisheyeRenderer::CubeFisheyeRenderer(
	const CameraImageModel& model, bool createEglContext, uint32_t noiseSeed)
	: model_(model), rng_(noiseSeed)
{
	const auto& cam = model_.camera;
	ASSERT_(cam.distortion == mrpt::img::DistortionModel::kannala_brandt);
	const int W = static_cast<int>(cam.ncols);
	const int H = static_cast<int>(cam.nrows);
	ASSERT_(W > 0 && H > 0);
	const double maxTheta = kbMaxTheta(cam, model_.imageCircleFovDeg);

	// Faces sample about 1.5 times finer than the image center:
	const double F = 1.5 * std::max(cam.fx(), cam.fy());
	const auto rotations = cubeFaceRotations();

	struct Box
	{
		double u0 = std::numeric_limits<double>::max();
		double u1 = -std::numeric_limits<double>::max();
		double v0 = std::numeric_limits<double>::max();
		double v1 = -std::numeric_limits<double>::max();
		bool used = false;
	};
	std::array<Box, 6> boxes;
	lut_.assign(static_cast<size_t>(W) * static_cast<size_t>(H), Sample());
	for (int v = 0; v < H; v++)
	{
		for (int u = 0; u < W; u++)
		{
			mrpt::math::TPoint3D d;
			if (!pixelRay(cam, maxTheta, u, v, d))
			{
				continue;
			}
			// The face the ray leaves the cube through:
			int best = 0;
			double bestZ = -2;
			for (int k = 0; k < 6; k++)
			{
				const auto& R = rotations[k];
				const double z = R(0, 2) * d.x + R(1, 2) * d.y + R(2, 2) * d.z;
				if (z > bestZ)
				{
					bestZ = z;
					best = k;
				}
			}
			const auto& R = rotations[best];
			const double x = R(0, 0) * d.x + R(1, 0) * d.y + R(2, 0) * d.z;
			const double y = R(0, 1) * d.x + R(1, 1) * d.y + R(2, 1) * d.z;
			const double fu = F * x / bestZ;
			const double fv = F * y / bestZ;
			auto& b = boxes[best];
			b.u0 = std::min(b.u0, fu);
			b.u1 = std::max(b.u1, fu);
			b.v0 = std::min(b.v0, fv);
			b.v1 = std::max(b.v1, fv);
			b.used = true;
			auto& s = lut_[static_cast<size_t>(v) * W + u];
			s.face = static_cast<int8_t>(best);
			s.x = static_cast<float>(fu);  // face coordinates, for now
			s.y = static_cast<float>(fv);
		}
	}

	// One renderer per face seen, cropped to what the image samples (with
	// one pixel more on each side for the bilinear interpolation):
	std::array<int8_t, 6> faceIndex;
	faceIndex.fill(-1);
	std::array<int, 6> i0s{};
	std::array<int, 6> j0s{};
	bool createContext = createEglContext;
	for (int k = 0; k < 6; k++)
	{
		const auto& b = boxes[k];
		if (!b.used)
		{
			continue;
		}
		const int i0 = static_cast<int>(std::floor(b.u0)) - 1;
		const int i1 = static_cast<int>(std::floor(b.u1)) + 2;
		const int j0 = static_cast<int>(std::floor(b.v0)) - 1;
		const int j1 = static_cast<int>(std::floor(b.v1)) + 2;
		Face f;
		f.rotation = mrpt::poses::CPose3D(rotations[k], mrpt::math::CVectorFixedDouble<3>());
		f.camera.ncols = static_cast<uint32_t>(i1 - i0 + 1);
		f.camera.nrows = static_cast<uint32_t>(j1 - j0 + 1);
		f.camera.fx(F);
		f.camera.fy(F);
		f.camera.cx(-i0);
		f.camera.cy(-j0);
		f.camera.distortion = mrpt::img::DistortionModel::none;
		f.renderer = makeRenderer(f.camera, createContext);
		createContext = false;
		faceIndex[k] = static_cast<int8_t>(faces_.size());
		i0s[k] = i0;
		j0s[k] = j0;
		faces_.push_back(std::move(f));
	}
	for (auto& s : lut_)
	{
		if (s.face < 0)
		{
			continue;
		}
		const int k = s.face;
		s.face = faceIndex[k];
		s.x -= static_cast<float>(i0s[k]);
		s.y -= static_cast<float>(j0s[k]);
	}
}

size_t CubeFisheyeRenderer::facePixels() const
{
	size_t n = 0;
	for (const auto& f : faces_)
	{
		n += static_cast<size_t>(f.camera.ncols) * f.camera.nrows;
	}
	return n;
}

void CubeFisheyeRenderer::render(
	CameraScene& scene, const TrajectorySource& traj, double tEpochSeconds, mrpt::img::CImage& out)
{
	auto viewport = scene.scene().getViewport();
	viewport->setViewportClipDistances(model_.clipMin, model_.clipMax);
	const mvsim::ViewportShadowSettingsGuard shadowGuard(*viewport);
	viewport->lightParameters().shadow_cascades =
		static_cast<uint8_t>(scene.world().sensor_shadow_cascades());
	scene.setTime(tEpochSeconds);
	for (auto& f : faces_)
	{
		placeCamera(
			*f.renderer, scene, traj, f.camera, model_.poseOnVehicle + f.rotation, tEpochSeconds);
		f.renderer->render_RGB(scene.scene(), f.image);
		ASSERT_(
			f.image.getWidth() == f.camera.ncols && f.image.getHeight() == f.camera.nrows &&
			f.image.channels() == 3);
	}

	// Bilinear resampling, then noise (one draw per pixel and channel, so the
	// noise stream does not depend on the image contents):
	const int W = static_cast<int>(model_.camera.ncols);
	const int H = static_cast<int>(model_.camera.nrows);
	out = mrpt::img::CImage(W, H, mrpt::img::CH_RGB);
	std::normal_distribution<double> noise(0.0, 1.0);
	const double noiseStd = model_.noiseStd;
	for (int v = 0; v < H; v++)
	{
		uint8_t* row = out.ptrLine<uint8_t>(v);
		for (int u = 0; u < W; u++)
		{
			const auto& s = lut_[static_cast<size_t>(v) * W + u];
			std::array<double, 3> rgb = {0, 0, 0};
			if (s.face >= 0)
			{
				const auto& img = faces_[s.face].image;
				const int x0 = static_cast<int>(s.x);
				const int y0 = static_cast<int>(s.y);
				const double ax = s.x - x0;
				const double ay = s.y - y0;
				const uint8_t* r0 = img.ptrLine<uint8_t>(y0) + 3 * x0;
				const uint8_t* r1 = img.ptrLine<uint8_t>(y0 + 1) + 3 * x0;
				for (int c = 0; c < 3; c++)
				{
					rgb[c] = (1 - ay) * ((1 - ax) * r0[c] + ax * r0[c + 3]) +
							 ay * ((1 - ax) * r1[c] + ax * r1[c + 3]);
				}
			}
			for (int c = 0; c < 3; c++)
			{
				const double n = noiseStd > 0 ? noiseStd * noise(rng_) : 0.0;
				row[3 * u + c] =
					static_cast<uint8_t>(std::clamp(std::round(rgb[c] + n), 0.0, 255.0));
			}
		}
	}
}

// ---------------------------------------------------------------------------
RgbCameraSim::RgbCameraSim(
	const CameraImageModel& model, const std::string& label, const std::string& imagesDir,
	std::shared_ptr<ImageWriter> writer, uint32_t noiseSeed, bool createEglContext)
	: model_(model), files_{imagesDir, label, std::move(writer), 0}
{
	checkLabelIsFileName(label);
	limitRollingShutterBands(model_);
	if (model_.camera.distortion == mrpt::img::DistortionModel::kannala_brandt)
	{
		ASSERTMSG_(
			model_.rollingShutterLineTime <= 0,
			"Camera '" + label +
				"': the rolling shutter is not supported with the kannala_brandt lens model");
		fisheye_ = std::make_unique<CubeFisheyeRenderer>(model_, createEglContext, noiseSeed);
		return;
	}
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
	if (fisheye_)
	{
		fisheye_->render(scene, traj, tEpochSeconds, obs->image);
	}
	else
	{
		renderRgb(*renderer_, scene, traj, model_, tEpochSeconds, obs->image);
	}
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
	checkLabelIsFileName(sensor.getName());
	pattern_.sensorLabel = sensor.getName();
	const auto& opts = sensor.rgbImageOptions();

	rgb_.camera = pattern_.cameraParamsIntensity;
	rgb_.poseOnVehicle = pattern_.sensorPose + pattern_.relativePoseIntensityWRTDepth;
	rgb_.clipMin = sensor.rgbClipMin();
	rgb_.clipMax = sensor.rgbClipMax();
	rgb_.noiseStd = opts.imageNoiseStd * noiseScale;
	rgb_.rollingShutterLineTime = opts.rollingShutterLineTime;
	rgb_.rollingShutterBands = opts.rollingShutterBands;
	limitRollingShutterBands(rgb_);
	ASSERTMSG_(
		opts.imageCircleFovDeg == 0,
		"'" + sensor.getName() + "': image circles are only simulated for camera sensors");

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
	// Range images are 16-bit: every depth in range must fit, and the range
	// must hold at least one unit.
	const double rangeUnits = depth_.clipMax / depth_.unit;
	ASSERTMSG_(
		!sensor.senseDepth() || (rangeUnits >= 1.0 && rangeUnits <= 65535.0),
		"'" + sensor.getName() +
			"': depth_clip_max / depth_resolution must be between 1 and 65535 (16-bit depth "
			"images)");

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
