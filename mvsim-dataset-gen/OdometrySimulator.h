/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPose2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose2D.h>

#include <random>

namespace mvsim_dataset_gen
{
/** Integrates planar (x,y,yaw) wheel odometry by dead-reckoning the
 * body-frame pose increment between two consecutive ground-truth poses
 * (the XY/yaw projection of the prescribed trajectory), optionally
 * corrupting each increment with Gaussian noise proportional to the motion
 * magnitude -- the classic dead-reckoning drift model. With zero noise,
 * this is exact dead reckoning of the true motion (odometry == ground
 * truth, up to the planar projection).
 *
 * There is no per-vehicle `<odometry>` sensor tag in mvsim's world XML (the
 * interactive simulator derives odometry from the stepped Box2D wheel
 * state instead); this tool synthesizes it directly from the prescribed
 * trajectory, with its own CLI-configured noise parameters.
 *
 * `runningOdom` is the caller-owned, running cumulative pose: pass the same
 * instance across calls to accumulate.
 */
mrpt::obs::CObservationOdometry::Ptr simulateOdometryStep(
	const mrpt::math::TPose2D& truePoseA, const mrpt::math::TPose2D& truePoseB,
	double tEpochSecondsB, double transNoiseRel, double rotNoiseRel,
	mrpt::poses::CPose2D& runningOdom, std::mt19937& rng);

}  // namespace mvsim_dataset_gen
