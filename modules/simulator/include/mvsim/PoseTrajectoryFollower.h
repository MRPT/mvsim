/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>

#include <utility>
#include <vector>

namespace mvsim
{
/** Evaluates a closed-form, time-parameterized 2D trajectory given as a
 * polyline of `(t, x, y)` waypoints, and drives a non-holonomic vehicle
 * along it using a pure-pursuit strategy: the reference *speed* comes from
 * the user-given time schedule (feedforward: distance/time between
 * consecutive waypoints), while the *heading* is servoed towards a
 * lookahead point found by walking along the path geometry starting at the
 * current time-reference position. This makes the tracking closed-loop
 * (small deviations self-correct every timestep) while still respecting
 * the (vx, omega) non-holonomic motion model shared by all vehicle types
 * in mvsim.
 *
 * This class has no dependency on World/Box2D or any other mvsim
 * simulation machinery, so it can be exercised in isolation
 * (see tests/test_pose_trajectory_follower.cpp).
 *
 * \ingroup mvsim_simulator_module
 */
class PoseTrajectoryFollower
{
   public:
	/** A single `(time, position)` sample of the reference trajectory. */
	struct Waypoint
	{
		double t = 0;  //!< Time since the start of the trajectory [s]
		mrpt::math::TPoint2D xy{0, 0};	//!< Target position [m]

		Waypoint() = default;
		Waypoint(double time, double x, double y) : t(time), xy(x, y) {}
	};

	PoseTrajectoryFollower() = default;

	/** Defines the trajectory to follow. Waypoints are sorted by time
	 * internally, so they need not be given in order.
	 * \exception std::exception If fewer than 2 waypoints are given, or if
	 * two waypoints share the exact same timestamp.
	 */
	void setWaypoints(std::vector<Waypoint> waypoints);

	void setLoop(bool loop) { loop_ = loop; }
	bool loop() const { return loop_; }

	/** Lookahead distance for the pure-pursuit heading control [m] */
	void setLookAheadDistance(double d) { lookAheadDistance_ = d; }
	double lookAheadDistance() const { return lookAheadDistance_; }

	/** Maximum |omega| the controller will ever command [rad/s]. Bounds the
	 * otherwise unbounded pure-pursuit curvature response near sharp
	 * corners (curvature grows as 1/lookAheadDistance as the lookahead
	 * point approaches perpendicular to the vehicle heading). Set to 0 to
	 * disable the clamp. */
	void setMaxAngularSpeed(double w) { maxAngularSpeed_ = w; }
	double maxAngularSpeed() const { return maxAngularSpeed_; }

	/** true if setWaypoints() has not been called with >=2 waypoints yet */
	bool empty() const { return waypoints_.size() < 2; }

	/** Read-only access to the defined waypoints (e.g. for GUI
	 * visualization purposes). */
	const std::vector<Waypoint>& waypoints() const { return waypoints_; }

	/** Time of the first waypoint [s]. Requires !empty() */
	double startTime() const { return waypoints_.front().t; }

	/** Time of the last waypoint [s]. Requires !empty() */
	double endTime() const { return waypoints_.back().t; }

	/** true once a non-looping trajectory has been fully driven (i.e.
	 * `simTime` is past the last waypoint's time). Always false if
	 * loop()==true or empty(). */
	bool finished(double simTime) const;

	/** Reference pose (position and path-tangent heading) at the given
	 * time, following the path geometry exactly, with no pure-pursuit
	 * smoothing. Mainly useful to place a vehicle at its initial pose, or
	 * for testing. Returns the origin if empty(). */
	mrpt::math::TPose2D referencePose(double simTime) const;

	/** Core algorithm: given the current absolute simulation time and the
	 * vehicle's actual current pose, returns the local-frame twist command
	 * (vx, omega; vy always 0) that tracks the trajectory. Returns a zero
	 * twist if empty(), if the (non-looping) trajectory already
	 * finished(), or while paused at a segment with zero feedforward
	 * speed (i.e. two waypoints sharing the same x,y). */
	mrpt::math::TTwist2D computeTwist(double simTime, const mrpt::math::TPose2D& curPose) const;

   private:
	std::vector<Waypoint> waypoints_;
	std::vector<double> cumLen_;  //!< Cumulative path length up to waypoint[i]
	bool loop_ = true;
	double lookAheadDistance_ = 0.5;  //!< [m]
	double maxAngularSpeed_ = 2.0;	//!< [rad/s], 0=unlimited

	double totalLength() const { return cumLen_.empty() ? 0.0 : cumLen_.back(); }

	/** Maps a (possibly out-of-range) time to a valid reference time within
	 * [startTime(), endTime()], wrapping around if loop_==true, or
	 * clamping to the endpoints otherwise. */
	double wrapTime(double simTime) const;

	/** Returns the segment index `i` (0-based) such that
	 * `waypoints_[i].t <= t <= waypoints_[i+1].t`, plus the `[0,1]`
	 * interpolation fraction within that segment. `t` must already be
	 * within `[startTime(), endTime()]`. */
	std::pair<size_t, double> segmentForTime(double t) const;

	/** Position, arc-length-since-start, and feedforward speed of the
	 * reference trajectory at the given time. */
	struct RefPoint
	{
		mrpt::math::TPoint2D xy;
		double arcLength = 0;
		double speed = 0;
	};
	RefPoint referenceAtTime(double simTime) const;

	/** Position of the path at the given arc length, wrapping around if
	 * loop_==true, or clamped to the endpoints otherwise. */
	mrpt::math::TPoint2D positionAtArcLength(double s) const;
};

}  // namespace mvsim
