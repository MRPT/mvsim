/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TCamera.h>
#include <mrpt/poses/CPose3D.h>
#include <mvsim/Sensors/DepthCameraSensor.h>
#include <mvsim/Sensors/ImuNoiseModel.h>
#include <mvsim/World.h>
#include <mvsim/raytracer/Lidar3DModel.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "LidarSimulator.h"

namespace mvsim
{
class GNSS;
}

namespace mvsim_dataset_gen
{
/** What a dataset's ground-truth YAML file records about one LiDAR. */
struct LidarGroundTruth
{
	std::string label;
	mrpt::poses::CPose3D pose;	//!< On the vehicle
	double clockOffset = 0;	 //!< Stamp minus sampling time [s]
	std::shared_ptr<const mvsim::rt::Lidar3DModel> model;
	/** The options in effect, with the drawn per-beam errors */
	std::shared_ptr<const LidarSimOptions> opts;
	/** Std. dev. of the per-beam errors in effect [deg, deg, m] */
	double beamElevationStdDeg = 0;
	double beamAzimuthStdDeg = 0;
	double beamRangeBiasStd = 0;
	int calibrationSeed = 0;
};

struct ImuGroundTruth
{
	std::string label;
	mrpt::poses::CPose3D pose;	//!< On the vehicle
	double clockOffset = 0;	 //!< [s]
	double period = 0;	//!< [s]
	mvsim::ImuNoiseModel::ChannelParams gyroscope;	//!< In effect
	mvsim::ImuNoiseModel::ChannelParams accelerometer;	//!< In effect
	std::string biasFile;  //!< The bias time series, if written
};

struct GnssGroundTruth
{
	std::string label;
	const mvsim::GNSS* sensor = nullptr;
	double clockOffset = 0;	 //!< [s]
	double noiseScale = 1;	//!< Of the position noise and of the event offsets
};

struct OdometryGroundTruth
{
	std::string label;
	double clockOffset = 0;	 //!< [s]
	double period = 0;	//!< [s]
	double transNoiseRel = 0;
	double rotNoiseRel = 0;
};

/** One image stream of a camera (RGB or depth). */
struct CameraImageGroundTruth
{
	mrpt::img::TCamera camera;	//!< Intrinsics and lens distortion
	mrpt::poses::CPose3D pose;	//!< Of the optical frame on the vehicle
	double clipMin = 0;
	double clipMax = 0;
	double noiseStd = 0;  //!< RGB: [intensity levels]; depth: [m]
	double rollingShutterLineTime = 0;	//!< [s], RGB only
	int rollingShutterBands = 0;
	double imageCircleFovDeg = 0;  //!< [deg], fisheye lenses; 0: none
	std::string files;	//!< Path pattern, relative to the dataset directory
	bool isDepth = false;
	double depthUnit = 0;  //!< [m]
	mvsim::DepthErrorModel depthErrors;
};

struct CameraGroundTruth
{
	std::string label;
	double clockOffset = 0;	 //!< [s]
	double period = 0;	//!< [s]
	std::optional<CameraImageGroundTruth> rgb;
	std::optional<CameraImageGroundTruth> depth;
};

/** Everything the ground-truth YAML file of a dataset records. */
struct DatasetGroundTruth
{
	std::string worldFile;
	std::string rawlogFile;
	std::string vehicleGtFile;
	std::string vehicle;
	std::string baseFrame;
	std::string shutter;
	double tStart = 0;	//!< [s], trajectory clock
	double tEnd = 0;  //!< [s], trajectory clock
	unsigned seed = 0;
	bool noiseless = false;
	double noiseScale = 1;
	const mvsim::World::GeoreferenceOptions* georef = nullptr;
	std::vector<LidarGroundTruth> lidars;
	std::vector<ImuGroundTruth> imus;
	std::vector<GnssGroundTruth> gnss;
	std::vector<OdometryGroundTruth> odometry;
	std::vector<CameraGroundTruth> cameras;
};

/** Writes `gt` as YAML to `path` (see docs/mvsim-dataset-gen.rst for the
 * format). Throws on I/O errors. */
void writeGroundTruthYaml(const std::string& path, const DatasetGroundTruth& gt);

}  // namespace mvsim_dataset_gen
