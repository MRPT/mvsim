/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>

#include <cstdint>
#include <vector>

namespace mvsim
{
/** A "2.5" shape: a 2D polygon + a [zMin,zMax] height range */
struct Shape2p5
{
	Shape2p5() = default;

	static Shape2p5 CreateConvexHullFromPoints(
		const std::vector<mrpt::math::TPoint3Df>& pts);

	double volume() const;

	void mergeWith(const Shape2p5& s);
	void mergeWith(const std::vector<mrpt::math::TPoint3Df>& pts);

	mrpt::math::TPolygon2D contour;
	float zMin = 0, zMax = 0;
};

}  // namespace mvsim
