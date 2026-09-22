/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/geometry.h>

// Small internal vector-algebra helpers shared across the raytracer's
// primitive intersection code. Not part of the public API.

namespace mvsim::rt::detail
{
inline double dot(const mrpt::math::TPoint3D& a, const mrpt::math::TPoint3D& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline mrpt::math::TPoint3D cross(const mrpt::math::TPoint3D& a, const mrpt::math::TPoint3D& b)
{
	return mrpt::math::crossProduct3D(a, b);
}

inline mrpt::math::TPoint3D normalized(const mrpt::math::TPoint3D& v)
{
	const double n = v.norm();
	if (n < 1e-12)
	{
		return {0, 0, 0};
	}
	return {v.x / n, v.y / n, v.z / n};
}

}  // namespace mvsim::rt::detail
