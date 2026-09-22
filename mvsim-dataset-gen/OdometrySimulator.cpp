/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "OdometrySimulator.h"

#include <cmath>

using namespace mvsim_dataset_gen;

mrpt::obs::CObservationOdometry::Ptr mvsim_dataset_gen::simulateOdometryStep(
	const mrpt::math::TPose2D& truePoseA, const mrpt::math::TPose2D& truePoseB,
	double tEpochSecondsB, double transNoiseRel, double rotNoiseRel,
	mrpt::poses::CPose2D& runningOdom, std::mt19937& rng)
{
	const mrpt::poses::CPose2D delta =
		mrpt::poses::CPose2D(truePoseB) - mrpt::poses::CPose2D(truePoseA);

	double dx = delta.x();
	double dy = delta.y();
	double dphi = delta.phi();

	const double dTrans = std::hypot(dx, dy);
	const double motionMag = dTrans + std::abs(dphi);

	if (transNoiseRel > 0 && motionMag > 0)
	{
		std::normal_distribution<double> transNoise(0.0, transNoiseRel * dTrans);
		dx += transNoise(rng);
		dy += transNoise(rng);
	}
	if (rotNoiseRel > 0 && motionMag > 0)
	{
		std::normal_distribution<double> rotNoise(0.0, rotNoiseRel * motionMag);
		dphi += rotNoise(rng);
	}

	runningOdom = runningOdom + mrpt::poses::CPose2D(dx, dy, dphi);

	auto obs = mrpt::obs::CObservationOdometry::Create();
	obs->timestamp = mrpt::Clock::fromDouble(tEpochSecondsB);
	obs->sensorLabel = "odom";
	obs->odometry = runningOdom;
	return obs;
}
