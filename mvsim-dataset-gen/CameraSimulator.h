/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/CImage.h>
#include <mrpt/img/TCamera.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/viz/Scene.h>
#include <mvsim/Sensors/DepthCameraSensor.h>

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "TrajectorySource.h"

namespace mvsim
{
class World;
class Block;
class CameraSensor;
}  // namespace mvsim

namespace mvsim_dataset_gen
{
/** The world as cameras see it: its world elements and blocks (never the
 * vehicle carrying the sensors), lit as the world XML says. Blocks that move
 * during the run follow their own trajectories. */
class CameraScene
{
   public:
	using MovingBlock =
		std::pair<std::shared_ptr<mvsim::Block>, std::shared_ptr<const TrajectorySource>>;

	CameraScene(mvsim::World& world, std::vector<MovingBlock> movingBlocks);

	/** Places the moving blocks at their poses at `tEpochSeconds`. */
	void setTime(double tEpochSeconds);

	mrpt::viz::Scene& scene() { return physical_; }
	const mvsim::World& world() const { return world_; }

   private:
	mvsim::World& world_;
	mrpt::viz::Scene viz_;	//!< Visual-only objects, not rendered
	mrpt::viz::Scene physical_;	 //!< What sensors see
	std::vector<MovingBlock> movingBlocks_;
};

/** How an image is formed: intrinsics (with lens distortion), the pose of
 * the optical frame (+Z forward, +X right, +Y down) on the vehicle, clip
 * distances, pixel noise and rolling shutter. */
struct CameraImageModel
{
	mrpt::img::TCamera camera;
	mrpt::poses::CPose3D poseOnVehicle;
	float clipMin = 0.01f;
	float clipMax = 1e4f;
	double noiseStd = 0;  //!< [intensity levels, 0-255]
	/** Time between rows [s], 0: global shutter. The image is rendered as
	 * `rollingShutterBands` bands of rows (at most one per row), each from the
	 * pose at the time of its middle row. */
	double rollingShutterLineTime = 0;
	int rollingShutterBands = 8;
	/** Fisheye (kannala_brandt) cameras: field of view of the image circle
	 * [deg], black beyond it; 0: none. */
	double imageCircleFovDeg = 0;
};

/** Renders fisheye (kannala_brandt) images of any field of view, also 180
 * degrees and wider: the scene is rendered as up to six 90-degree pinhole
 * views (the faces of a cube centered on the optical center, each cropped to
 * the part of it the image sees), which are then resampled (bilinear) into
 * the fisheye image. Pixel noise is added after resampling. */
class CubeFisheyeRenderer
{
   public:
	CubeFisheyeRenderer(const CameraImageModel& model, bool createEglContext, uint32_t noiseSeed);

	/** Renders the image of the camera on the vehicle following `traj` at
	 * `tEpochSeconds`. */
	void render(
		CameraScene& scene, const TrajectorySource& traj, double tEpochSeconds,
		mrpt::img::CImage& out);

	/** Faces rendered per image, and their total pixel count. */
	size_t faceCount() const { return faces_.size(); }
	size_t facePixels() const;

	/** The ray (unit vector in the optical frame: +Z forward, +X right, +Y
	 * down) of the pixel at image coordinates (u, v) (OpenCV convention), if
	 * the lens sees anything there. */
	static bool pixelRay(
		const mrpt::img::TCamera& cam, double maxThetaRad, double u, double v,
		mrpt::math::TPoint3D& ray);

   private:
	struct Face
	{
		mrpt::poses::CPose3D rotation;	//!< Of the face optical frame
		mrpt::img::TCamera camera;	//!< Cropped pinhole view
		std::shared_ptr<mrpt::opengl::CFBORender> renderer;
		mrpt::img::CImage image;
	};
	/** Where each output pixel comes from: a face (-1: black) and the image
	 * coordinates in it. */
	struct Sample
	{
		int8_t face = -1;
		float x = 0;
		float y = 0;
	};

	CameraImageModel model_;
	std::vector<Face> faces_;
	std::vector<Sample> lut_;
	std::mt19937 rng_;
};

/** What the depth images of a depth camera contain. */
struct DepthImageModel
{
	mrpt::img::TCamera camera;
	mrpt::poses::CPose3D poseOnVehicle;	 //!< Of the optical frame
	float clipMin = 0.1f;
	float clipMax = 15.0f;	//!< Also the maximum range
	double unit = 1e-3;	 //!< Range image unit [m]
	double noiseStd = 0;  //!< Gaussian depth noise [m]
	mvsim::DepthErrorModel errors;	//!< Systematic errors
};

/** Wall clock time spent per camera [s]. */
struct CameraTiming
{
	size_t frames = 0;
	double render = 0;	//!< Rendering, and depth conversion
	double write = 0;  //!< Image files
};

/** Encodes and writes image files in worker threads (PNG encoding takes
 * longer than rendering). The queue is bounded, so memory stays bounded. */
class ImageWriter
{
   public:
	explicit ImageWriter(size_t threads);
	~ImageWriter();
	ImageWriter(const ImageWriter&) = delete;
	ImageWriter& operator=(const ImageWriter&) = delete;

	/** Queues a write of `img` (which must not share its pixels) to `path`. */
	void write(mrpt::img::CImage img, const std::string& path);

	/** Waits for every queued write. Throws if any of them failed. */
	void finish();

   private:
	void worker();
	void stopWorkers();

	std::vector<std::thread> threads_;
	std::mutex mtx_;
	std::condition_variable cv_;
	std::deque<std::pair<mrpt::img::CImage, std::string>> queue_;
	size_t busy_ = 0;
	bool stop_ = false;
	std::string error_;
};

/** Where and how image files are written: `<imagesDir>/<label>_<index>...`,
 * referenced from the observations by their file name only (relative to
 * the images directory of the rawlog). */
struct ImageFiles
{
	std::string imagesDir;
	std::string label;
	std::shared_ptr<ImageWriter> writer;
	size_t nextIndex = 0;

	/** The file name of the next frame, with this suffix and extension. */
	std::string name(const std::string& suffixAndExt) const;
};

/** Renders the images of an RGB camera ("camera" sensor). */
class RgbCameraSim
{
   public:
	RgbCameraSim(
		const CameraImageModel& model, const std::string& label, const std::string& imagesDir,
		std::shared_ptr<ImageWriter> writer, uint32_t noiseSeed, bool createEglContext);

	/** The observation of the frame sampled at `tEpochSeconds` (the time
	 * of the first row with a rolling shutter), with the vehicle following
	 * `traj`, and its image written to a PNG file. */
	mrpt::obs::CObservationImage::Ptr simulate(
		CameraScene& scene, const TrajectorySource& traj, double tEpochSeconds);

	const CameraImageModel& model() const { return model_; }
	const CameraTiming& timing() const { return timing_; }
	/** The cube face renderer of a fisheye camera, or null. */
	const CubeFisheyeRenderer* fisheye() const { return fisheye_.get(); }

   private:
	CameraImageModel model_;
	CameraTiming timing_;
	ImageFiles files_;
	std::shared_ptr<mrpt::opengl::CFBORender> renderer_;
	std::unique_ptr<CubeFisheyeRenderer> fisheye_;	//!< kannala_brandt cameras
};

/** Renders the images of a depth camera ("rgbd_camera" sensor): RGB and/or
 * depth, as its options say. */
class DepthCameraSim
{
   public:
	/** `noiseScale` scales the depth and image noise; 0 also disables the
	 * systematic depth errors. */
	DepthCameraSim(
		const mvsim::DepthCameraSensor& sensor, const std::string& imagesDir,
		std::shared_ptr<ImageWriter> writer, uint32_t noiseSeed, double noiseScale,
		bool createEglContext);

	/** The observation of the frame sampled at `tEpochSeconds`, with its
	 * images written to files: the RGB image as PNG, the range image in
	 * MRPT's external format (a compressed binary matrix of `uint16` depths
	 * in `rangeUnits`). */
	mrpt::obs::CObservation3DRangeScan::Ptr simulate(
		CameraScene& scene, const TrajectorySource& traj, double tEpochSeconds);

	bool hasRgb() const { return static_cast<bool>(rgbRenderer_) || singlePass_; }
	bool hasDepth() const { return static_cast<bool>(depthRenderer_) || singlePass_; }
	const CameraImageModel& rgbModel() const { return rgb_; }
	const DepthImageModel& depthModel() const { return depth_; }
	const CameraTiming& timing() const { return timing_; }

   private:
	CameraTiming timing_;
	mrpt::obs::CObservation3DRangeScan pattern_;
	CameraImageModel rgb_;
	DepthImageModel depth_;
	bool singlePass_ = false;  //!< Both images from one render
	ImageFiles files_;
	std::shared_ptr<mrpt::opengl::CFBORender> rgbRenderer_;
	std::shared_ptr<mrpt::opengl::CFBORender> depthRenderer_;
	std::mt19937 rng_;
	mrpt::math::CMatrixFloat depthImage_;
};

}  // namespace mvsim_dataset_gen
