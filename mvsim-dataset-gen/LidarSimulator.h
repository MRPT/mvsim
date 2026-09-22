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

#include "TrajectorySource.h"

namespace mvsim_dataset_gen
{
enum class ShutterMode
{
	Global,	 //!< One pose for the whole sweep; every point's `t` field is 0.
	Rolling	 //!< Pose re-interpolated per column; `t` is the column's firing offset.
};

struct LidarSimOptions
{
	ShutterMode shutter = ShutterMode::Global;

	/** Std. dev. of additive Gaussian noise on each range reading [m]. 0 =
	 * noiseless (the ideal ground-truth dataset). */
	double rangeStdNoise = 0.0;

	/** Whether to compute and store the "intensity" field (a purely
	 * geometric Lambertian cos(incidence)/r^2 stand-in: the world XML
	 * carries no material model). */
	bool generateIntensity = false;
};

/** Simulates one full 360-degree sweep of a 3D LiDAR via exact ray casting
 * against `scene`, with the sensor rigidly mounted at `sensorPoseOnVehicle`
 * on a vehicle whose ground-truth pose follows `traj`.
 *
 * Points are expressed in the sensor frame *at the sweep-start pose*
 * (the usual convention for de-skewing consumers): `t` tells them how much
 * ego-motion to undo. In `ShutterMode::Global`, every point's `t` is 0.
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
