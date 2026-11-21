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
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>
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
	uint16_t label = 0;
	uint16_t instance = 0;
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

/** A moving object as all the rays of one column see it: they share one
 * firing time and one origin. */
struct PosedObject
{
	const MovingObject* object = nullptr;
	CPose3D pose;  //!< Of the object frame in the world, at the firing time
	TPoint3D rayOrgLocal;  //!< Ray origin, in the object frame
	TPoint3D toBoundCenter;	 //!< From the ray origin to the bounding sphere center (world frame)
	double distSqrToBoundCenter = 0;
};

/** The moving objects at the given time, for rays from `rayOrg` (world). */
std::vector<PosedObject> poseObjects(
	const std::vector<MovingObject>& objects, double tEpochSeconds, const TPoint3D& rayOrg)
{
	std::vector<PosedObject> out;
	out.reserve(objects.size());
	for (const auto& obj : objects)
	{
		PosedObject po;
		po.object = &obj;
		// Objects sharing one trajectory are stored together:
		if (!out.empty() && out.back().object->trajectory == obj.trajectory)
		{
			po.pose = out.back().pose;
		}
		else
		{
			po.pose = obj.trajectory->poseAt(tEpochSeconds);
		}
		po.rayOrgLocal = po.pose.inverseComposePoint(rayOrg);
		po.toBoundCenter = po.pose.composePoint(obj.boundCenter) - rayOrg;
		po.distSqrToBoundCenter = po.toBoundCenter.sqrNorm();
		out.push_back(po);
	}
	return out;
}

/** Keeps in `hit` (world frame) the nearest of itself and the hits of `ray`
 * on the moving objects, and in `hitScene` the scene of its primitive. */
void castRayOnObjects(
	const std::vector<PosedObject>& objects, const mvsim::rt::Ray& ray,
	std::optional<mvsim::rt::Hit>& hit, const mvsim::rt::RayScene*& hitScene)
{
	for (const auto& po : objects)
	{
		// Quick rejection (`ray.dir` is a unit vector): the ray line misses
		// the bounding sphere, or the sphere is out of the range interval
		// still of interest.
		const double tMax = hit ? hit->t : ray.tMax;
		const double radius = po.object->boundRadius;
		const TPoint3D& m = po.toBoundCenter;
		const double along = m.x * ray.dir.x + m.y * ray.dir.y + m.z * ray.dir.z;
		if (po.distSqrToBoundCenter - along * along > radius * radius ||
			along + radius < ray.tMin || along - radius > tMax)
		{
			continue;
		}
		mvsim::rt::Ray localRay;
		localRay.org = po.rayOrgLocal;
		localRay.dir = po.pose.inverseRotateVector(ray.dir);
		localRay.tMin = ray.tMin;
		localRay.tMax = tMax;
		auto localHit = po.object->scene.castRay(localRay);
		if (!localHit)
		{
			continue;
		}
		localHit->normal = po.pose.rotateVector(localHit->normal);
		hit = localHit;
		hitScene = &po.object->scene;
	}
}

/** Number of sub-rays of the beam model: the beam axis and six rays around
 * it. */
constexpr int kBeamRays = 7;

/** Share of the beam energy of the axis sub-ray and of each of the six
 * others. A Gaussian beam whose full width at half maximum is the
 * divergence has half the axis intensity on the cone of the six. */
constexpr double kAxisWeight = 0.25;
constexpr double kRingWeight = 0.125;

/** Smallest incidence cosine used for the footprint spread: beyond about 84
 * deg, the spread stays that of 84 deg. */
constexpr double kMinFootprintCos = 0.1;

/** Beam model: largest incidence angle [deg] whose footprint spread grows
 * the range noise; real sensors measured beyond it are not noisier. */
constexpr double kBeamMaxSpreadIncidenceDeg = 80.0;

/** Beam model: largest footprint spread [m] that adds to the range noise. A
 * longer echo shifts the detected range rather than spreading it further:
 * real sensors measured at any incidence are not noisier than this spread
 * gives (about 4 cm). */
constexpr double kBeamMaxSpread = 0.15;

/** Beam model: two sub-ray hits lie on the same surface if each is within
 * this distance [m] of the other's tangent plane. */
constexpr double kSameSurfaceTolerance = 0.05;

/** Geometry of the beam model's sub-rays, the same for every ray of a sweep:
 * the cosine and sine of half the divergence, and the direction of each of
 * the six outer sub-rays around the axis. */
struct BeamPattern
{
	double cosHalf = 1;
	double sinHalf = 0;
	std::array<double, kBeamRays - 1> cosPhi{};
	std::array<double, kBeamRays - 1> sinPhi{};
};

BeamPattern beamPattern(double divergence)
{
	BeamPattern bp;
	bp.cosHalf = std::cos(0.5 * divergence);
	bp.sinHalf = std::sin(0.5 * divergence);
	for (int k = 0; k < kBeamRays - 1; k++)
	{
		// Two of them in the vertical plane of the axis:
		const double phi = mrpt::DEG2RAD(90.0 + 60.0 * k);
		bp.cosPhi[k] = std::cos(phi);
		bp.sinPhi[k] = std::sin(phi);
	}
	return bp;
}

/** Sub-ray directions about the unit vector `dir`: the axis first, then six
 * rays on the cone of half the divergence. */
std::array<TPoint3D, kBeamRays> beamDirections(const TPoint3D& dir, const BeamPattern& bp)
{
	// A horizontal unit vector normal to the axis, and the one normal to both:
	TPoint3D h(-dir.y, dir.x, 0);
	const double hn = h.norm();
	h = hn > 1e-9 ? h * (1.0 / hn) : TPoint3D(0, 1, 0);
	const TPoint3D v(
		dir.y * h.z - dir.z * h.y, dir.z * h.x - dir.x * h.z, dir.x * h.y - dir.y * h.x);
	std::array<TPoint3D, kBeamRays> out;
	out[0] = dir;
	for (int k = 1; k < kBeamRays; k++)
	{
		out[k] = dir * bp.cosHalf + (h * bp.cosPhi[k - 1] + v * bp.sinPhi[k - 1]) * bp.sinHalf;
	}
	return out;
}

/** One sub-ray hit of the beam model. */
struct BeamSample
{
	double range = 0;
	double cosIncidence = 0;
	/** Received power: the sub-ray's share of the beam, times
	 * cos(incidence) / r^2. */
	double power = 0;
	double lambertCos = 0;
	TPoint3D hitWorld;
	TPoint3D normalWorld;
	uint16_t label = 0;
	uint16_t instance = 0;
};

BeamSample beamSample(
	const mvsim::rt::Ray& ray, const mvsim::rt::Hit& hit, const mvsim::rt::RayScene& hitScene,
	double weight)
{
	const double dn =
		ray.dir.x * hit.normal.x + ray.dir.y * hit.normal.y + ray.dir.z * hit.normal.z;
	BeamSample s;
	s.range = hit.t;
	s.cosIncidence = std::abs(dn);
	s.power = weight * std::max(s.cosIncidence, 1e-9) / std::max(hit.t * hit.t, 1e-12);
	s.lambertCos = std::max(0.0, -dn);
	s.hitWorld = ray.at(hit.t);
	s.normalWorld = hit.normal;
	const auto& prim = hitScene.primitive(hit.primIndex);
	s.label = prim.semanticClass;
	s.instance = prim.instanceId;
	return s;
}

/** Whether two sub-ray hits lie on the same surface: each within
 * kSameSurfaceTolerance of the other's tangent plane. */
bool sameSurface(const BeamSample& a, const BeamSample& b)
{
	const TPoint3D d = b.hitWorld - a.hitWorld;
	const double offA =
		std::abs(d.x * a.normalWorld.x + d.y * a.normalWorld.y + d.z * a.normalWorld.z);
	const double offB =
		std::abs(d.x * b.normalWorld.x + d.y * b.normalWorld.y + d.z * b.normalWorld.z);
	return offA <= kSameSurfaceTolerance && offB <= kSameSurfaceTolerance;
}

/** An echo of the beam model: a run of sub-ray hits, in range order, each
 * on the surface of the previous one or closer to it than the echo
 * separation. */
struct Echo
{
	double range = 0;  //!< Power-weighted mean range
	double minRange = 0;
	double maxRange = 0;
	double power = 0;  //!< Sum of the power of its hits
	size_t strongest = 0;  //!< Index of its hit of largest power
	bool mixed = false;	 //!< It joins hits on different surfaces
};

/** The echo of largest power among the hits in `samples` (sorted by range
 * on return, which must not be empty). Hits on one surface form one echo,
 * however long; hits on different surfaces closer than `separation` along
 * the beam merge into one too, as the pulses they return overlap. */
Echo strongestEcho(std::vector<BeamSample>& samples, double separation)
{
	std::stable_sort(
		samples.begin(), samples.end(),
		[](const BeamSample& a, const BeamSample& b) { return a.range < b.range; });
	Echo best;
	size_t first = 0;
	bool mixed = false;
	for (size_t i = 1; i <= samples.size(); i++)
	{
		if (i < samples.size())
		{
			const bool same = sameSurface(samples[i - 1], samples[i]);
			if (same || samples[i].range - samples[i - 1].range < separation)
			{
				mixed = mixed || !same;
				continue;
			}
		}
		// Hits [first, i) form one echo:
		Echo e;
		e.strongest = first;
		e.mixed = mixed;
		e.minRange = samples[first].range;
		e.maxRange = samples[i - 1].range;
		double weightedRange = 0;
		for (size_t j = first; j < i; j++)
		{
			e.power += samples[j].power;
			weightedRange += samples[j].power * samples[j].range;
			if (samples[j].power > samples[e.strongest].power)
			{
				e.strongest = j;
			}
		}
		e.range = weightedRange / e.power;
		if (e.power > best.power)
		{
			best = e;
		}
		first = i;
		mixed = false;
	}
	return best;
}

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
	float minRange, float maxRange, bool withIntensity, const std::optional<uint16_t>& mixedLabel,
	std::mt19937& rng)
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
			if (mixedLabel)
			{
				p.label = *mixedLabel;
				p.instance = 0;
			}
		}
	}
}

}  // namespace

mrpt::obs::CObservationPointCloud::Ptr mvsim_dataset_gen::simulateLidarSweep(
	const mvsim::rt::RayScene& scene, const std::vector<MovingObject>& movingObjects,
	const mvsim::rt::Lidar3DModel& model, const TrajectorySource& traj,
	const CPose3D& sensorPoseOnVehicle, double sweepStartEpochSeconds,
	const std::string& sensorLabel, const LidarSimOptions& opts, std::mt19937& rng)
{
	auto pts = mrpt::maps::CGenericPointsMap::Create();
	pts->registerField_float("t");
	pts->registerField_uint16("ring");
	if (opts.generateIntensity)
	{
		pts->registerField_float("intensity");
	}
	if (opts.generateLabels)
	{
		pts->registerField_uint16("label");
		pts->registerField_uint16("instance");
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
	const double beamDivergence = mrpt::DEG2RAD(opts.beamDivergenceDeg);
	// The beam model replaces the mixed returns between neighbor rays:
	const bool beamModel = opts.echoSeparation > 0 && beamDivergence > 0;
	const bool mixedReturns = opts.mixedPixelProbability > 0 && !beamModel;
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
	// Received power of a Lambertian surface at normal incidence and the
	// detection range:
	const double minPower = opts.detectionRange > 0 ? 1.0 / mrpt::square(opts.detectionRange) : 0.0;
	const BeamPattern beam = beamPattern(beamDivergence);
	const double maxSpreadPerMeter =
		beamDivergence * std::tan(mrpt::DEG2RAD(kBeamMaxSpreadIncidenceDeg));

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
		const TPoint3D rayOrg = sensorPoseThisColumn.composePoint(TPoint3D(0, 0, 0));
		// The moving objects, at this column's firing time:
		const std::vector<PosedObject> posedObjects =
			poseObjects(movingObjects, sweepStartEpochSeconds + tColRel, rayOrg);

		std::vector<ColumnPoint>& out = perColumn[col];
		std::vector<BeamSample> samples;
		samples.reserve(kBeamRays);
		for (int ring = 0; ring < nRings; ring++)
		{
			const TPoint3D localDir = model.rayDirection(ring, col);

			mvsim::rt::Ray ray;
			ray.org = rayOrg;
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

			auto hit = scene.castRay(ray);
			const mvsim::rt::RayScene* hitScene = &scene;
			castRayOnObjects(posedObjects, ray, hit, hitScene);
			if (!hit)
			{
				continue;
			}

			double range = hit->t;
			// Lambertian cosine for the geometric intensity stand-in, and the
			// surface the return comes from:
			double lambertCos = std::max(
				0.0, -(ray.dir.x * hit->normal.x + ray.dir.y * hit->normal.y +
					   ray.dir.z * hit->normal.z));
			const auto& hitPrim = hitScene->primitive(hit->primIndex);
			uint16_t label = hitPrim.semanticClass;
			uint16_t instance = hitPrim.instanceId;
			if (realistic && beamModel)
			{
				// The beam samples its footprint with several sub-rays; the
				// strongest echo among their hits is the return. Hits beyond the
				// incidence cutoff return nothing:
				samples.clear();
				const auto keep = [&](const BeamSample& bs)
				{
					if (bs.cosIncidence >= cosMaxIncidence)
					{
						samples.push_back(bs);
					}
				};
				keep(beamSample(ray, *hit, *hitScene, kAxisWeight));
				const TPoint3D axis = sampleDir * (1.0 / sampleDir.norm());
				const auto dirs = beamDirections(axis, beam);
				for (int k = 1; k < kBeamRays; k++)
				{
					mvsim::rt::Ray sub = ray;
					sub.dir = rotateOnly(sensorPoseThisColumn, dirs[k]);
					auto subHit = scene.castRay(sub);
					const mvsim::rt::RayScene* subScene = &scene;
					castRayOnObjects(posedObjects, sub, subHit, subScene);
					if (subHit)
					{
						keep(beamSample(sub, *subHit, *subScene, kRingWeight));
					}
				}
				if (samples.empty())
				{
					continue;  // too oblique: no return
				}
				const Echo echo = strongestEcho(samples, opts.echoSeparation);
				if (echo.power < minPower)
				{
					continue;  // too weak to be detected
				}
				const BeamSample& s = samples[echo.strongest];
				lambertCos = s.lambertCos;
				label = s.label;
				instance = s.instance;
				if (echo.mixed && opts.mixedReturnLabel)
				{
					label = *opts.mixedReturnLabel;
					instance = 0;
				}
				// The echo's spread, up to that of a plane seen at
				// kBeamMaxSpreadIncidenceDeg and kBeamMaxSpread, around its mean
				// range but within its range interval; then Gaussian noise,
				// constant plus proportional to range:
				const double spread = std::min(
					{echo.maxRange - echo.minRange, maxSpreadPerMeter * echo.range,
					 kBeamMaxSpread});
				const double lowest = echo.minRange + 0.5 * spread;
				const double highest = std::max(lowest, echo.maxRange - 0.5 * spread);
				const double center = std::clamp(echo.range, lowest, highest);
				const double sigma = std::sqrt(
					mrpt::square(opts.rangeStdNoise) +
					mrpt::square(opts.rangeStdNoisePerMeter * echo.range));
				range = center;
				range += sigma * unitNoise(colRng);
				range += spread * unitUniform(colRng);
				if (calibrated)
				{
					range += opts.beamRangeBias.at(ring);
				}
			}
			else if (realistic)
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
				if (cosIncidence / std::max(range * range, 1e-12) < minPower)
				{
					continue;  // too weak to be detected
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
					const double c = std::clamp(cosIncidence, kMinFootprintCos, 1.0);
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
			if (opts.generateIntensity)
			{
				cp.intensity = geometricIntensity(lambertCos, range);
			}
			if (opts.generateLabels)
			{
				cp.label = label;
				cp.instance = instance;
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
			model.params().maxRange, opts.generateIntensity, opts.mixedReturnLabel, mixedRng);
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
			if (opts.generateLabels)
			{
				pts->insertPointField_uint16("label", cp.label);
				pts->insertPointField_uint16("instance", cp.instance);
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

MovingObject::MovingObject(
	mvsim::rt::RayScene localScene, std::shared_ptr<const TrajectorySource> traj)
	: scene(std::move(localScene)), trajectory(std::move(traj))
{
	mvsim::rt::AABB box;
	for (size_t i = 0; i < scene.primitiveCount(); i++)
	{
		box.grow(mvsim::rt::primitiveAABB(scene.primitive(i)));
	}
	if (box.isValid())
	{
		boundCenter = box.centroid();
		// Slightly enlarged, so that rounding never rejects a grazing hit:
		boundRadius = 0.5 * (box.max - box.min).norm() * (1.0 + 1e-9) + 1e-9;
	}
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
