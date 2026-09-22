/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "LidarSimulator.h"

#include <mrpt/maps/CGenericPointsMap.h>

#include <algorithm>
#include <cmath>

using namespace mvsim_dataset_gen;
using mrpt::math::TPoint3D;
using mrpt::poses::CPose3D;

namespace
{
/** Rotates (no translation) a local-frame direction vector by `pose`'s
 * rotation. */
TPoint3D rotateOnly(const CPose3D& pose, const TPoint3D& localDir)
{
	const TPoint3D withOrigin = pose.composePoint(localDir);
	const TPoint3D origin = pose.composePoint(TPoint3D(0, 0, 0));
	return withOrigin - origin;
}

}  // namespace

mrpt::obs::CObservationPointCloud::Ptr mvsim_dataset_gen::simulateLidarSweep(
	const mvsim::rt::RayScene& scene, const mvsim::rt::Lidar3DModel& model,
	const TrajectorySource& traj, const CPose3D& sensorPoseOnVehicle, double sweepStartEpochSeconds,
	const std::string& sensorLabel, const LidarSimOptions& opts, std::mt19937& rng)
{
	auto pts = mrpt::maps::CGenericPointsMap::Create();
	pts->registerField_float("t");
	pts->registerField_uint16("ring");
	if (opts.generateIntensity)
	{
		pts->registerField_float("intensity");
	}

	const CPose3D vehiclePoseAtStart = traj.poseAt(sweepStartEpochSeconds);
	const CPose3D sensorPoseAtStart = vehiclePoseAtStart + sensorPoseOnVehicle;

	std::normal_distribution<double> noiseDist(0.0, std::max(0.0, opts.rangeStdNoise));

	const int nRings = model.vertRays();
	const int nCols = model.horzRays();

	// Rolling shutter: cache one sensor world-pose per column (computed
	// once, reused across all rings in that column).
	CPose3D sensorPoseThisColumn = sensorPoseAtStart;
	int cachedCol = -1;

	for (int col = 0; col < nCols; col++)
	{
		if (opts.shutter == ShutterMode::Rolling)
		{
			if (col != cachedCol)
			{
				const double tCol = sweepStartEpochSeconds + model.columnFireTime(col);
				const CPose3D vehiclePoseAtCol = traj.poseAt(tCol);
				sensorPoseThisColumn = vehiclePoseAtCol + sensorPoseOnVehicle;
				cachedCol = col;
			}
		}
		else
		{
			sensorPoseThisColumn = sensorPoseAtStart;
		}

		const float tField = opts.shutter == ShutterMode::Rolling
								 ? static_cast<float>(model.columnFireTime(col))
								 : 0.0f;

		for (int ring = 0; ring < nRings; ring++)
		{
			const TPoint3D localDir = model.rayDirection(ring, col);

			mvsim::rt::Ray ray;
			ray.org = sensorPoseThisColumn.composePoint(TPoint3D(0, 0, 0));
			ray.dir = rotateOnly(sensorPoseThisColumn, localDir);
			ray.tMin = std::max(1e-6, static_cast<double>(model.params().minRange));
			ray.tMax = model.params().maxRange;

			const auto hit = scene.castRay(ray);
			if (!hit)
			{
				continue;
			}

			double range = hit->t;
			if (opts.rangeStdNoise > 0)
			{
				range += noiseDist(rng);
			}
			if (range < model.params().minRange || range > model.params().maxRange)
			{
				continue;
			}

			const TPoint3D worldPt = ray.at(range);
			// Express the point in the sensor frame *at the sweep-start
			// pose*, regardless of shutter mode (the de-skewing convention:
			// `t` tells a consumer how much motion to undo).
			const TPoint3D localPt = sensorPoseAtStart.inverseComposePoint(worldPt);

			pts->insertPointFast(
				static_cast<float>(localPt.x), static_cast<float>(localPt.y),
				static_cast<float>(localPt.z));
			pts->insertPointField_float("t", tField);
			pts->insertPointField_uint16("ring", static_cast<uint16_t>(ring));

			if (opts.generateIntensity)
			{
				// Purely geometric Lambertian cos(incidence)/r^2 stand-in;
				// mvsim's world XML carries no material model.
				const double cosIncidence = std::max(
					0.0, -(ray.dir.x * hit->normal.x + ray.dir.y * hit->normal.y +
						   ray.dir.z * hit->normal.z));
				const double falloff = 1.0 / std::max(1.0, range * range);
				const float intensity =
					static_cast<float>(std::clamp(cosIncidence * falloff * 4.0, 0.0, 1.0));
				pts->insertPointField_float("intensity", intensity);
			}
		}
	}

	auto obs = mrpt::obs::CObservationPointCloud::Create();
	obs->pointcloud = pts;
	obs->sensorLabel = sensorLabel;
	obs->sensorPose = sensorPoseAtStart;
	obs->timestamp = mrpt::Clock::fromDouble(sweepStartEpochSeconds);
	return obs;
}
