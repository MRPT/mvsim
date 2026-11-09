/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose3D.h>
#include <mvsim/raytracer/Lidar3DModel.h>
#include <mvsim/raytracer/RayScene.h>

#include <random>
#include <string>
#include <vector>

#include "TrajectorySource.h"

namespace mvsim_dataset_gen
{
enum class ShutterMode
{
	/** One pose for the whole sweep; every point's `t` field is 0. */
	Global,
	/** Realistic rotating LiDAR: each azimuth column fires at its own time,
	 * from the vehicle pose interpolated at that instant; `t` is the
	 * column's firing offset from the sweep start, in [0, sensor period). */
	Rolling
};

struct LidarSimOptions
{
	ShutterMode shutter = ShutterMode::Rolling;

	/** Std. dev. of additive Gaussian noise on each range reading [m]. 0 =
	 * noiseless (the ideal ground-truth dataset). */
	double rangeStdNoise = 0.0;

	/** Whether to compute and store the "intensity" field (a purely
	 * geometric Lambertian cos(incidence)/r^2 stand-in: the world XML
	 * carries no material model). */
	bool generateIntensity = false;

	/** \name Realistic error model (see mvsim::Lidar3D::ErrorModel); all
	 * zero or empty: disabled.
	 * @{ */
	double rangeStdNoisePerMeter = 0.0;
	double beamDivergenceDeg = 0.0;
	double maxIncidenceDeg = 0.0;
	double mixedPixelProbability = 0.0;
	/** Per-ring calibration errors: elevation and azimuth offsets [rad] and
	 * range bias [m], filled by drawCalibrationErrors(). */
	std::vector<double> beamElevOffset;
	std::vector<double> beamAzimOffset;
	std::vector<double> beamRangeBias;
	/** @} */

	bool hasRealisticErrors() const
	{
		return rangeStdNoisePerMeter > 0 || beamDivergenceDeg > 0 || maxIncidenceDeg > 0 ||
			   mixedPixelProbability > 0 || !beamRangeBias.empty();
	}
};

/** Draws the per-ring calibration errors of `opts` (left empty if every std
 * is zero), reproducibly from `seed`. */
void drawCalibrationErrors(
	LidarSimOptions& opts, int nRings, double elevationStdDeg, double azimuthStdDeg,
	double rangeBiasStd, int seed);

/** Simulates one full 360-degree sweep of a 3D LiDAR via exact ray casting
 * against `scene`, with the sensor rigidly mounted at `sensorPoseOnVehicle`
 * on a vehicle whose ground-truth pose follows `traj`.
 *
 * Each point is expressed in the sensor frame at the instant its column
 * fired, as a real rotating LiDAR driver outputs it: in
 * `ShutterMode::Rolling` the cloud is motion-skewed, and consumers can use
 * the per-point `t` field (relative to the observation timestamp, i.e. the
 * sweep start) to de-skew it. In `ShutterMode::Global`, every point's `t`
 * is 0 and the whole sweep uses the sweep-start pose.
 *
 * Columns are ray-cast in parallel with TBB (`RayScene::castRay()` is
 * const/thread-safe), each with its own RNG stream seeded up front from
 * `rng`, and merged into the output in column order afterwards -- so the
 * result depends only on `rng`'s state on entry, never on thread scheduling.
 */
mrpt::obs::CObservationPointCloud::Ptr simulateLidarSweep(
	const mvsim::rt::RayScene& scene, const mvsim::rt::Lidar3DModel& model,
	const TrajectorySource& traj, const mrpt::poses::CPose3D& sensorPoseOnVehicle,
	double sweepStartEpochSeconds, const std::string& sensorLabel, const LidarSimOptions& opts,
	std::mt19937& rng);

}  // namespace mvsim_dataset_gen
