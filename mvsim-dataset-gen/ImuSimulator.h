/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mvsim/Sensors/ImuNoiseModel.h>

#include <random>
#include <string>

#include "TrajectorySource.h"

namespace mvsim
{
class World;
}

namespace mvsim_dataset_gen
{
/** Generates one IMU sample by analytically differentiating the prescribed
 * ground-truth trajectory (central finite differences), then optionally
 * corrupting it with mvsim's own ImuNoiseModel (Forster 2016: white noise +
 * bias random walk).
 *
 * Angular velocity is the SO(3) logarithm of the relative rotation between
 * `t-diffStep` and `t+diffStep`, divided by `2*diffStep`: exact to
 * second order in `diffStep`, and well-defined even for large rotations
 * (unlike a naive Euler-angle finite difference). Proper (specific)
 * acceleration is the finite-difference second derivative of position,
 * minus gravity, expressed in the body frame at `t` -- so a stationary
 * sensor reads +9.81 m/s^2 on its local Z axis, as a real accelerometer
 * does.
 */
/** `sampleDt` is the time since the *previous* IMU sample [s] (used to scale
 * the noise model's bias random walk); `diffStep` is the (typically much
 * smaller) finite-difference epsilon used to compute the true derivative,
 * defaulting to a tenth of the sensor period. */
mrpt::obs::CObservationIMU::Ptr simulateImuSample(
	const mvsim::World& world, const TrajectorySource& traj,
	const mrpt::poses::CPose3D& sensorPoseOnVehicle, double tEpochSeconds, double diffStep,
	double sampleDt, const std::string& sensorLabel, bool measureOrientation,
	double orientationStdNoise, mvsim::ImuNoiseModel& noiseModel, std::mt19937& rng);

}  // namespace mvsim_dataset_gen
