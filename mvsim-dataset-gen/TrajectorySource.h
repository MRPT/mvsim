/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <string>

namespace mvsim_dataset_gen
{
/** A prescribed, ground-truth SE(3) trajectory: the vehicle pose *is* this
 * trajectory at every instant, with no controller or dynamics involved.
 *
 * v1 supports `.tum` files only (full SE(3): timestamp, x, y, z, qx, qy,
 * qz, qw, one epoch-seconds-timestamped pose per line). 2D `(x,y,phi)`
 * input with terrain-following is left as future work (see the design
 * doc).
 */
class TrajectorySource
{
   public:
	/** Loads a TUM-format trajectory file. Throws on failure. */
	void loadTum(const std::string& path);

	/** Timestamp (epoch seconds) of the first/last waypoint. */
	double startTime() const;
	double endTime() const;

	/** Interpolated pose at `tEpochSeconds`, clamped to
	 * [startTime(), endTime()] (a query outside that range returns the
	 * nearest endpoint's pose rather than extrapolating). */
	mrpt::poses::CPose3D poseAt(double tEpochSeconds) const;

	void setInterpolationMethod(mrpt::poses::TInterpolatorMethod m);

   private:
	mrpt::poses::CPose3DInterpolator path_;
};

}  // namespace mvsim_dataset_gen
