/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mvsim/PoseTrajectoryFollower.h>

#include <algorithm>
#include <cmath>

using namespace mvsim;

void PoseTrajectoryFollower::setWaypoints(std::vector<Waypoint> waypoints)
{
	if (waypoints.size() < 2)
	{
		THROW_EXCEPTION_FMT(
			"[PoseTrajectoryFollower] At least 2 waypoints are required, got %zu",
			waypoints.size());
	}

	std::sort(
		waypoints.begin(), waypoints.end(),
		[](const Waypoint& a, const Waypoint& b) { return a.t < b.t; });

	for (size_t i = 1; i < waypoints.size(); i++)
	{
		if (waypoints[i].t <= waypoints[i - 1].t)
		{
			THROW_EXCEPTION_FMT(
				"[PoseTrajectoryFollower] Waypoint times must be strictly "
				"increasing, found repeated/unsorted time t=%f",
				waypoints[i].t);
		}
	}

	waypoints_ = std::move(waypoints);

	cumLen_.assign(waypoints_.size(), 0.0);
	for (size_t i = 1; i < waypoints_.size(); i++)
	{
		cumLen_[i] = cumLen_[i - 1] + (waypoints_[i].xy - waypoints_[i - 1].xy).norm();
	}
}

bool PoseTrajectoryFollower::finished(double simTime) const
{
	if (loop_ || empty())
	{
		return false;
	}
	return simTime >= endTime();
}

double PoseTrajectoryFollower::wrapTime(double simTime) const
{
	const double t0 = startTime();
	const double t1 = endTime();

	if (!loop_)
	{
		return std::clamp(simTime, t0, t1);
	}

	const double period = t1 - t0;
	if (period <= 0)
	{
		return t0;
	}

	double t = std::fmod(simTime - t0, period);
	if (t < 0)
	{
		t += period;
	}
	return t0 + t;
}

std::pair<size_t, double> PoseTrajectoryFollower::segmentForTime(double t) const
{
	size_t i = 0;
	for (; i + 2 < waypoints_.size(); i++)
	{
		if (t <= waypoints_[i + 1].t)
		{
			break;
		}
	}

	const double t0 = waypoints_[i].t;
	const double t1 = waypoints_[i + 1].t;
	const double frac = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;

	return {i, std::clamp(frac, 0.0, 1.0)};
}

PoseTrajectoryFollower::RefPoint PoseTrajectoryFollower::referenceAtTime(double simTime) const
{
	const auto [i, frac] = segmentForTime(wrapTime(simTime));

	const auto& wp0 = waypoints_[i];
	const auto& wp1 = waypoints_[i + 1];

	RefPoint rp;
	rp.xy = wp0.xy + (wp1.xy - wp0.xy) * frac;
	rp.arcLength = cumLen_[i] + frac * (cumLen_[i + 1] - cumLen_[i]);

	const double dt = wp1.t - wp0.t;
	rp.speed = (dt > 0) ? ((cumLen_[i + 1] - cumLen_[i]) / dt) : 0.0;

	return rp;
}

mrpt::math::TPose2D PoseTrajectoryFollower::referencePose(double simTime) const
{
	if (empty())
	{
		return {};
	}

	// Resolve simTime to its actual (clamped/wrapped) reference time once, so
	// the forward/backward probes below step from *that* point rather than
	// from the raw (possibly far out-of-range) simTime -- otherwise, for a
	// non-looping trajectory queried well past its end, both simTime+dtProbe
	// and simTime-dtProbe would clamp right back to the same terminal time.
	const double tRef = wrapTime(simTime);
	const auto rp = referenceAtTime(tRef);

	// Estimate heading from a small time step ahead. At the very last
	// waypoint of a non-looping trajectory, wrapTime() clamps tRef+dtProbe
	// right back to the same point, so the forward probe collapses to zero
	// displacement; fall back to a backward probe in that case so the final
	// heading still reflects the incoming direction of travel instead of 0.
	const double dtProbe = 1e-3;
	const auto rpAhead = referenceAtTime(tRef + dtProbe);

	auto d = rpAhead.xy - rp.xy;
	if (d.norm() <= 1e-9)
	{
		const auto rpBehind = referenceAtTime(tRef - dtProbe);
		d = rp.xy - rpBehind.xy;
	}

	double yaw = 0;
	if (d.norm() > 1e-9)
	{
		yaw = std::atan2(d.y, d.x);
	}

	return {rp.xy.x, rp.xy.y, yaw};
}

mrpt::math::TPoint2D PoseTrajectoryFollower::positionAtArcLength(double s) const
{
	const double len = totalLength();
	if (len <= 0)
	{
		return waypoints_.front().xy;
	}

	if (loop_)
	{
		s = std::fmod(s, len);
		if (s < 0)
		{
			s += len;
		}
	}
	else
	{
		s = std::clamp(s, 0.0, len);
	}

	size_t i = 0;
	for (; i + 2 < waypoints_.size(); i++)
	{
		if (s <= cumLen_[i + 1])
		{
			break;
		}
	}

	const double segLen = cumLen_[i + 1] - cumLen_[i];
	const double frac = (segLen > 0) ? ((s - cumLen_[i]) / segLen) : 0.0;

	return waypoints_[i].xy +
		   (waypoints_[i + 1].xy - waypoints_[i].xy) * std::clamp(frac, 0.0, 1.0);
}

mrpt::math::TTwist2D PoseTrajectoryFollower::computeTwist(
	double simTime, const mrpt::math::TPose2D& curPose) const
{
	mrpt::math::TTwist2D out{0, 0, 0};

	if (empty() || finished(simTime))
	{
		return out;
	}

	const auto ref = referenceAtTime(simTime);

	if (ref.speed <= 0)
	{
		// Paused segment (two waypoints at the same location): hold position.
		return out;
	}

	// Anchor the lookahead search on the *time-reference* arc length rather
	// than on the closest point of the whole path to the vehicle: for
	// non-convex polylines (e.g. a square loop) a pure closest-point search
	// can jump to an unrelated, unreached segment as soon as the vehicle
	// drifts towards the inside of the shape, which would derail tracking
	// entirely. Anchoring on schedule time is monotonic and immune to that,
	// at the cost of relying on the omega clamp below to keep curvature
	// bounded while the vehicle temporarily lags behind schedule (e.g.
	// while slowing down for a sharp corner).
	const auto lookXY = positionAtArcLength(ref.arcLength + lookAheadDistance_);

	// Transform the lookahead point into the vehicle's local frame:
	const double dx = lookXY.x - curPose.x;
	const double dy = lookXY.y - curPose.y;
	const double c = std::cos(curPose.phi);
	const double s = std::sin(curPose.phi);
	const double lx = c * dx + s * dy;
	const double ly = -s * dx + c * dy;

	const double Ld2 = lx * lx + ly * ly;
	if (Ld2 < 1e-6)
	{
		out.vx = ref.speed;
		return out;
	}

	// Classic pure-pursuit curvature: kappa = 2*y / L^2
	const double curvature = 2.0 * ly / Ld2;

	out.vx = ref.speed;
	out.omega = ref.speed * curvature;

	// Bound the angular speed: real (non-holonomic) robots cannot spin
	// arbitrarily fast. Slow down -- rather than just capping omega -- so
	// that the *curvature* (hence the executed turn radius) stays the one
	// pure-pursuit actually asked for; capping omega alone while keeping
	// vx fixed would widen the turn radius and make the vehicle undershoot
	// sharp corners instead of rounding them tightly.
	if (maxAngularSpeed_ > 0 && std::abs(out.omega) > maxAngularSpeed_)
	{
		const double scale = maxAngularSpeed_ / std::abs(out.omega);
		out.vx *= scale;
		out.omega = std::copysign(maxAngularSpeed_, out.omega);
	}

	return out;
}
