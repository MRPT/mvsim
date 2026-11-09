/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "LidarSimulator.h"

#include <mrpt/core/bits_math.h>
#include <mrpt/maps/CGenericPointsMap.h>

#include "TimeUtils.h"
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

/** Direction with its elevation and azimuth shifted by the given angles
 * [rad]. */
TPoint3D perturbDirection(const TPoint3D& dir, double dElevation, double dAzimuth)
{
	const double horz = std::sqrt(dir.x * dir.x + dir.y * dir.y);
	const double el = std::atan2(dir.z, horz) + dElevation;
	const double az = std::atan2(dir.y, dir.x) + dAzimuth;
	const double n = dir.norm();
	return {n * std::cos(el) * std::cos(az), n * std::cos(el) * std::sin(az), n * std::sin(el)};
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

/** What the mixed-return pass needs of each hit (kept only if enabled):
 * range, nominal unit direction (sensor frame), the surface hit point and
 * normal (world frame), and the Lambertian cosine of the intensity model. */
struct MixedReturnInfo
{
	float range = 0;
	TPoint3D dir;
	TPoint3D hitWorld;
	TPoint3D normalWorld;
	float lambertCos = 0;
};

/** The geometric intensity stand-in: Lambertian cos(incidence)/r^2 (the
 * world XML carries no material model). */
float geometricIntensity(double lambertCos, double range)
{
	const double falloff = 1.0 / std::max(1.0, range * range);
	return static_cast<float>(std::clamp(lambertCos * falloff * 4.0, 0.0, 1.0));
}

/** Mixed returns: where neighbor rays hit different surfaces (one lies off
 * the other's tangent plane), the beam footprint straddles the edge and,
 * with the given probability, the return is a range between both. */
void addMixedReturns(
	std::vector<std::vector<ColumnPoint>>& perColumn,
	const std::vector<std::vector<MixedReturnInfo>>& info, int nRings, double probability,
	float minRange, float maxRange, bool withIntensity, std::mt19937& rng)
{
	const int nCols = static_cast<int>(perColumn.size());
	// (column, ring) => index in perColumn[column], or -1 if no return:
	std::vector<int> idx(static_cast<size_t>(nCols) * nRings, -1);
	for (int c = 0; c < nCols; c++)
	{
		for (size_t k = 0; k < perColumn[c].size(); k++)
		{
			idx[static_cast<size_t>(c) * nRings + perColumn[c][k].ring] = static_cast<int>(k);
		}
	}
	const auto at = [&](int c, int r) -> const MixedReturnInfo*
	{
		if (r < 0 || r >= nRings)
		{
			return nullptr;
		}
		c = (c + nCols) % nCols;
		const int k = idx[static_cast<size_t>(c) * nRings + r];
		return k < 0 ? nullptr : &info[c][k];
	};

	std::uniform_real_distribution<double> uni(0.0, 1.0);
	std::vector<std::vector<float>> newRange(nCols);
	for (int c = 0; c < nCols; c++)
	{
		newRange[c].resize(perColumn[c].size());
		for (size_t k = 0; k < perColumn[c].size(); k++)
		{
			const MixedReturnInfo& p = info[c][k];
			const int ring = perColumn[c][k].ring;
			newRange[c][k] = p.range;

			// The neighbor farthest off this point's tangent plane:
			const MixedReturnInfo* other = nullptr;
			double worst = std::max(0.05, 0.02 * p.range);
			for (const auto* q :
				 {at(c - 1, ring), at(c + 1, ring), at(c, ring - 1), at(c, ring + 1)})
			{
				if (!q)
				{
					continue;
				}
				const TPoint3D d = q->hitWorld - p.hitWorld;
				const double off =
					std::abs(d.x * p.normalWorld.x + d.y * p.normalWorld.y + d.z * p.normalWorld.z);
				if (off > worst)
				{
					worst = off;
					other = q;
				}
			}
			if (other && uni(rng) < probability)
			{
				const float r = p.range + static_cast<float>(uni(rng)) * (other->range - p.range);
				if (r > minRange && r < maxRange)
				{
					newRange[c][k] = r;
				}
			}
		}
	}
	for (int c = 0; c < nCols; c++)
	{
		for (size_t k = 0; k < perColumn[c].size(); k++)
		{
			const float r = newRange[c][k];
			if (r == info[c][k].range)
			{
				continue;
			}
			ColumnPoint& p = perColumn[c][k];
			const TPoint3D& dir = info[c][k].dir;
			p.x = static_cast<float>(dir.x * r);
			p.y = static_cast<float>(dir.y * r);
			p.z = static_cast<float>(dir.z * r);
			if (withIntensity)
			{
				p.intensity = geometricIntensity(info[c][k].lambertCos, r);
			}
		}
	}
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
	const bool mixedReturns = opts.mixedPixelProbability > 0;
	std::vector<std::vector<MixedReturnInfo>> mixedInfo(mixedReturns ? nCols : 0);

	// Drawn only with the error model enabled, so that the default noise
	// stream is unchanged:
	const bool realistic = opts.hasRealisticErrors();
	const bool calibrated = !opts.beamRangeBias.empty();
	std::uint32_t mixedSeed = 0;
	if (mixedReturns)
	{
		mixedSeed = static_cast<std::uint32_t>(rng());
	}
	const double cosMaxIncidence =
		opts.maxIncidenceDeg > 0 ? std::cos(mrpt::DEG2RAD(opts.maxIncidenceDeg)) : -1.0;
	const double beamDivergence = mrpt::DEG2RAD(opts.beamDivergenceDeg);

	auto processColumn = [&](int col)
	{
		std::mt19937 colRng(colSeeds[col]);
		std::normal_distribution<double> noiseDist(0.0, std::max(0.0, opts.rangeStdNoise));
		std::normal_distribution<double> unitNoise(0.0, 1.0);
		std::uniform_real_distribution<double> unitUniform(-0.5, 0.5);

		const bool rolling = opts.shutter == ShutterMode::Rolling;
		const double tColRel = rolling ? model.columnFireTime(col) : 0.0;
		const CPose3D sensorPoseThisColumn =
			rolling ? traj.poseAt(sweepStartEpochSeconds + tColRel) + sensorPoseOnVehicle
					: sensorPoseAtStart;
		const auto tField = static_cast<float>(tColRel);

		std::vector<ColumnPoint>& out = perColumn[col];
		for (int ring = 0; ring < nRings; ring++)
		{
			const TPoint3D localDir = model.rayDirection(ring, col);

			mvsim::rt::Ray ray;
			ray.org = sensorPoseThisColumn.composePoint(TPoint3D(0, 0, 0));
			// With calibration errors, the ray samples the scene along a
			// perturbed direction, but the point is reported along the
			// nominal one:
			const TPoint3D sampleDir = calibrated ? perturbDirection(
														localDir, opts.beamElevOffset.at(ring),
														opts.beamAzimOffset.at(ring))
												  : localDir;
			ray.dir = rotateOnly(sensorPoseThisColumn, sampleDir);
			ray.tMin = std::max(1e-6, static_cast<double>(model.params().minRange));
			ray.tMax = model.params().maxRange;

			const auto hit = scene.castRay(ray);
			if (!hit)
			{
				continue;
			}

			double range = hit->t;
			if (realistic)
			{
				const double dirNorm = ray.dir.norm();
				const double cosIncidence =
					dirNorm > 0 ? std::abs(
									  ray.dir.x * hit->normal.x + ray.dir.y * hit->normal.y +
									  ray.dir.z * hit->normal.z) /
									  dirNorm
								: 1.0;
				if (cosIncidence < cosMaxIncidence)
				{
					continue;  // too oblique: no return
				}
				// Gaussian noise, constant plus proportional to range:
				const double sigma = std::sqrt(
					mrpt::square(opts.rangeStdNoise) +
					mrpt::square(opts.rangeStdNoisePerMeter * range));
				const double footprint = range * beamDivergence;
				range += sigma * unitNoise(colRng);
				// The beam footprint on an oblique surface spreads the return
				// uniformly over a range interval:
				if (footprint > 0)
				{
					const double c = std::clamp(cosIncidence, 0.1, 1.0);
					const double tanIncidence = std::sqrt(1.0 - c * c) / c;
					range += footprint * tanIncidence * unitUniform(colRng);
				}
				if (calibrated)
				{
					range += opts.beamRangeBias.at(ring);
				}
			}
			else if (opts.rangeStdNoise > 0)
			{
				range += noiseDist(colRng);
			}
			if (range < model.params().minRange || range > model.params().maxRange)
			{
				continue;
			}

			// Point in the sensor frame at this column's firing pose
			// (not de-skewed, like real LiDAR drivers output it):
			const TPoint3D localPt = localDir * range;

			ColumnPoint cp;
			cp.x = static_cast<float>(localPt.x);
			cp.y = static_cast<float>(localPt.y);
			cp.z = static_cast<float>(localPt.z);
			cp.t = tField;
			cp.ring = static_cast<uint16_t>(ring);
			// Lambertian cosine for the geometric intensity stand-in:
			const double lambertCos = std::max(
				0.0, -(ray.dir.x * hit->normal.x + ray.dir.y * hit->normal.y +
					   ray.dir.z * hit->normal.z));
			if (opts.generateIntensity)
			{
				cp.intensity = geometricIntensity(lambertCos, range);
			}
			if (mixedReturns)
			{
				const double n = localDir.norm();
				MixedReturnInfo mi;
				mi.range = static_cast<float>(range);
				mi.dir = n > 0 ? localDir * (1.0 / n) : localDir;
				mi.hitWorld = ray.at(hit->t);
				mi.normalWorld = hit->normal;
				mi.lambertCos = static_cast<float>(lambertCos);
				mixedInfo[col].push_back(mi);
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

	if (mixedReturns)
	{
		std::mt19937 mixedRng(mixedSeed);
		addMixedReturns(
			perColumn, mixedInfo, nRings, opts.mixedPixelProbability, model.params().minRange,
			model.params().maxRange, opts.generateIntensity, mixedRng);
	}

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
	obs->timestamp = timeFromSeconds(sweepStartEpochSeconds);
	return obs;
}

void mvsim_dataset_gen::drawCalibrationErrors(
	LidarSimOptions& opts, int nRings, double elevationStdDeg, double azimuthStdDeg,
	double rangeBiasStd, int seed)
{
	opts.beamElevOffset.clear();
	opts.beamAzimOffset.clear();
	opts.beamRangeBias.clear();
	if (elevationStdDeg <= 0 && azimuthStdDeg <= 0 && rangeBiasStd <= 0)
	{
		return;
	}
	std::mt19937 rng(static_cast<std::uint32_t>(seed));
	std::normal_distribution<double> unit(0.0, 1.0);
	for (int r = 0; r < nRings; r++)
	{
		opts.beamElevOffset.push_back(mrpt::DEG2RAD(elevationStdDeg) * unit(rng));
		opts.beamAzimOffset.push_back(mrpt::DEG2RAD(azimuthStdDeg) * unit(rng));
		opts.beamRangeBias.push_back(rangeBiasStd * unit(rng));
	}
}
