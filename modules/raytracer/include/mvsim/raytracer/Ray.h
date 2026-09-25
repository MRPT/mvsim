/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint3D.h>

#include <cstdint>
#include <limits>

namespace mvsim::rt
{
/** A ray for exact geometric ray casting. `dir` must be unit-norm. */
struct Ray
{
	mrpt::math::TPoint3D org{0, 0, 0};
	mrpt::math::TVector3D dir{1, 0, 0};

	double tMin = 1e-6;
	double tMax = std::numeric_limits<double>::max();

	/** Point at parameter `t` along the ray. */
	mrpt::math::TPoint3D at(double t) const
	{
		return {org.x + t * dir.x, org.y + t * dir.y, org.z + t * dir.z};
	}
};

/** Result of a successful ray-primitive (or ray-scene) intersection. */
struct Hit
{
	double t = 0;  //!< Range along the ray [m]
	mrpt::math::TVector3D normal{0, 0, 1};	//!< Unit, outward-facing
	uint32_t primIndex = 0;	 //!< Index into RayScene's primitive list
};

}  // namespace mvsim::rt
