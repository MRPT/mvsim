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

namespace mvsim
{
class World;
}

namespace mvsim_dataset_gen
{
/** A prescribed, ground-truth SE(3) trajectory: the vehicle pose *is* this
 * trajectory at every instant, with no controller or dynamics involved.
 *
 * Two input forms:
 *  - `.tum`: full SE(3) (timestamp, x, y, z, qx, qy, qz, qw), one
 *    epoch-seconds-timestamped pose per line. See loadTum().
 *  - 2D `(t, x, y)` waypoints, either the `<waypoint t="" x="" y=""/>` XML
 *    block used by the files under `definitions/trajectories/`, or a plain
 *    3-column text file. Yaw comes from the path tangent; z/roll/pitch are
 *    derived by querying a World's terrain. See load2DWithTerrain().
 */
class TrajectorySource
{
   public:
	/** Loads a TUM-format trajectory file. Throws on failure. */
	void loadTum(const std::string& path);

	/** Loads a 2D `(t, x, y)` trajectory (XML `<waypoint>` block or plain
	 * "t x y" text, auto-detected) and lifts it to 6-DoF: yaw from the
	 * path tangent (central difference; one-sided at the endpoints), and
	 * z/roll/pitch from a plane fit through 4 probe points -- offset by
	 * +/-footprintLx/2 and +/-footprintLy/2 in the body frame -- queried
	 * against `world`'s terrain (`World::getElevationsAt()`, which covers
	 * both world elements and blocks; the *lowest* elevation at each probe
	 * is taken, so a block floating above the ground near the path never
	 * fools the fit into thinking the terrain is higher there). The plane
	 * fit is a small-angle approximation, adequate for gently rolling
	 * terrain, not for cliffs or stairs. Throws on failure. */
	void load2DWithTerrain(
		const std::string& path, const mvsim::World& world, double footprintLx = 0.6,
		double footprintLy = 0.4);

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
