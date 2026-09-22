/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint3D.h>

#include <vector>

namespace mvsim::rt
{
/** Ray-generation model for a rotating 3D LiDAR, mirroring the parameters
 * of mvsim's `Lidar3D` sensor (see `definitions/*.sensor.xml`) so the same
 * world/vehicle files describe both the interactive simulator and this
 * offline ray tracer.
 *
 * Ring ordering matches `Lidar3D::loadConfigFrom()`: rings are stored in
 * *increasing* elevation order (ring 0 is the lowest/most negative angle),
 * whether they come from an explicit `vertical_ray_angles` list (which is
 * sorted ascending, regardless of the order given in XML) or from an even
 * `vert_fov_degrees` split.
 */
class Lidar3DModel
{
   public:
	struct Params
	{
		/** Number of vertical rings. Ignored if `verticalRayAnglesDeg` is
		 * non-empty (its size is used instead). */
		int vertNumRays = 16;

		/** Symmetric vertical FOV [deg], used only if
		 * `verticalRayAnglesDeg` is empty. */
		double vertFovDegrees = 30.0;

		/** Explicit per-ring elevation angles [deg], as in
		 * `<vertical_ray_angles>`. Order given here does not matter: rings
		 * are re-sorted ascending, matching Lidar3D's own behavior. Leave
		 * empty to use `vertFovDegrees` + `vertNumRays` instead. */
		std::vector<double> verticalRayAnglesDeg;

		/** Number of azimuth columns in a full 360-degree sweep. */
		int horzNumRays = 1800;

		/** Duration of one full sweep [s]. */
		double sensorPeriod = 0.1;

		double minRange = 0.01;
		double maxRange = 80.0;
	};

	explicit Lidar3DModel(Params params);

	const Params& params() const { return params_; }

	int vertRays() const { return static_cast<int>(ringElevationsRad_.size()); }
	int horzRays() const { return params_.horzNumRays; }

	/** Elevation angle of `ring` [rad], `ring` in `[0, vertRays())`. */
	double ringElevationRad(int ring) const { return ringElevationsRad_.at(ring); }

	/** Azimuth of `col` [rad], in `[-pi, pi)`; `col` in `[0, horzRays())`. */
	double columnAzimuthRad(int col) const;

	/** Time at which `col` fires, relative to the sweep start [s]. Every
	 * ring within a column is treated as firing simultaneously (see the
	 * design doc for the intra-column refinement left as future work). */
	double columnFireTime(int col) const;

	/** Unit direction vector, in the sensor frame, for ray `(ring, col)`. */
	mrpt::math::TPoint3D rayDirection(int ring, int col) const;

   private:
	Params params_;
	std::vector<double> ringElevationsRad_;	 //!< Ascending order.
};

}  // namespace mvsim::rt
