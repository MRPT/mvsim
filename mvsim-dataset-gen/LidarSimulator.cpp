/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "LidarSimulator.h"

#include <mrpt/maps/CGenericPointsMap.h>
#ifdef MVSIM_HAS_TBB
#include <tbb/parallel_for.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

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

/** One ray hit, buffered per-column so columns can be ray-cast in parallel
 * while still being merged into the output point cloud in a fixed,
 * seed-reproducible column order. */
struct ColumnPoint
{
	float x = 0;
	float y = 0;
	float z = 0;
	float t = 0;
	uint16_t ring = 0;
	float intensity = 0;
};

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

	const int nRings = model.vertRays();
	const int nCols = model.horzRays();

	// One RNG seed per column, drawn sequentially from `rng` up front: the
	// noise stream is then fully determined by `rng`'s state on entry,
	// regardless of how columns get scheduled across threads below.
	std::vector<std::uint32_t> colSeeds(nCols);
	for (int col = 0; col < nCols; col++)
	{
		colSeeds[col] = static_cast<std::uint32_t>(rng());
	}

	// Each column is cast independently (RayScene::castRay() is const and
	// thread-safe), buffered here, then merged into `pts` below in column
	// order so the resulting point cloud does not depend on thread
	// scheduling either.
	std::vector<std::vector<ColumnPoint>> perColumn(nCols);

	auto processColumn = [&](int col)
	{
		std::mt19937 colRng(colSeeds[col]);
		std::normal_distribution<double> noiseDist(0.0, std::max(0.0, opts.rangeStdNoise));

		CPose3D sensorPoseThisColumn = sensorPoseAtStart;
		if (opts.shutter == ShutterMode::Rolling)
		{
			const double tCol = sweepStartEpochSeconds + model.columnFireTime(col);
			const CPose3D vehiclePoseAtCol = traj.poseAt(tCol);
			sensorPoseThisColumn = vehiclePoseAtCol + sensorPoseOnVehicle;
		}

		const float tField = opts.shutter == ShutterMode::Rolling
								 ? static_cast<float>(model.columnFireTime(col))
								 : 0.0f;

		std::vector<ColumnPoint>& out = perColumn[col];
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
				range += noiseDist(colRng);
			}
			if (range < model.params().minRange || range > model.params().maxRange)
			{
				continue;
			}

			const TPoint3D worldPt = ray.at(range);
			// Express the point in the sensor frame *at the sweep-start
			// pose*, regardless of shutter mode (the de-skewing
			// convention: `t` tells a consumer how much motion to undo).
			const TPoint3D localPt = sensorPoseAtStart.inverseComposePoint(worldPt);

			ColumnPoint cp;
			cp.x = static_cast<float>(localPt.x);
			cp.y = static_cast<float>(localPt.y);
			cp.z = static_cast<float>(localPt.z);
			cp.t = tField;
			cp.ring = static_cast<uint16_t>(ring);

			if (opts.generateIntensity)
			{
				// Purely geometric Lambertian cos(incidence)/r^2
				// stand-in; mvsim's world XML carries no material model.
				const double cosIncidence = std::max(
					0.0, -(ray.dir.x * hit->normal.x + ray.dir.y * hit->normal.y +
						   ray.dir.z * hit->normal.z));
				const double falloff = 1.0 / std::max(1.0, range * range);
				cp.intensity =
					static_cast<float>(std::clamp(cosIncidence * falloff * 4.0, 0.0, 1.0));
			}

			out.push_back(cp);
		}
	};

#ifdef MVSIM_HAS_TBB
	tbb::parallel_for(0, nCols, processColumn);
#else
	for (int col = 0; col < nCols; col++)
	{
		processColumn(col);
	}
#endif

	for (const auto& colPts : perColumn)
	{
		for (const auto& cp : colPts)
		{
			pts->insertPointFast(cp.x, cp.y, cp.z);
			pts->insertPointField_float("t", cp.t);
			pts->insertPointField_uint16("ring", cp.ring);
			if (opts.generateIntensity)
			{
				pts->insertPointField_float("intensity", cp.intensity);
			}
		}
	}

	auto obs = mrpt::obs::CObservationPointCloud::Create();
	obs->pointcloud = pts;
	obs->sensorLabel = sensorLabel;
	// "sensorPose" field means the base_link => lidar transformation:
	obs->sensorPose = sensorPoseOnVehicle;
	obs->timestamp = mrpt::Clock::fromDouble(sweepStartEpochSeconds);
	return obs;
}
