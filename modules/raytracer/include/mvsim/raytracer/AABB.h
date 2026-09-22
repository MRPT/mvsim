/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mvsim/raytracer/Ray.h>

#include <algorithm>
#include <limits>

namespace mvsim::rt
{
/** Axis-aligned bounding box, used both for per-primitive bounds and for BVH
 * node bounds. */
struct AABB
{
	mrpt::math::TPoint3D min{
		std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
		std::numeric_limits<double>::max()};
	mrpt::math::TPoint3D max{
		std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
		std::numeric_limits<double>::lowest()};

	bool isValid() const { return min.x <= max.x && min.y <= max.y && min.z <= max.z; }

	void grow(const mrpt::math::TPoint3D& p)
	{
		min.x = std::min(min.x, p.x);
		min.y = std::min(min.y, p.y);
		min.z = std::min(min.z, p.z);
		max.x = std::max(max.x, p.x);
		max.y = std::max(max.y, p.y);
		max.z = std::max(max.z, p.z);
	}

	void grow(const AABB& other)
	{
		grow(other.min);
		grow(other.max);
	}

	mrpt::math::TPoint3D centroid() const
	{
		return {(min.x + max.x) * 0.5, (min.y + max.y) * 0.5, (min.z + max.z) * 0.5};
	}

	/** Index of the widest axis: 0=x, 1=y, 2=z */
	int longestAxis() const
	{
		const double dx = max.x - min.x;
		const double dy = max.y - min.y;
		const double dz = max.z - min.z;
		if (dx >= dy && dx >= dz)
		{
			return 0;
		}
		if (dy >= dz)
		{
			return 1;
		}
		return 2;
	}

	/** Standard slab test. Returns true and updates [tNear,tFar] with the
	 * intersection interval (clipped to the ray's own [tMin,tMax]) if the ray
	 * intersects this box. */
	bool intersect(const Ray& ray, double& tNear, double& tFar) const
	{
		tNear = ray.tMin;
		tFar = ray.tMax;

		const double* rayOrg = &ray.org.x;
		const double* rayDir = &ray.dir.x;
		const double* boxMin = &min.x;
		const double* boxMax = &max.x;

		for (int axis = 0; axis < 3; axis++)
		{
			if (std::abs(rayDir[axis]) < 1e-12)
			{
				if (rayOrg[axis] < boxMin[axis] || rayOrg[axis] > boxMax[axis])
				{
					return false;
				}
				continue;
			}

			const double invD = 1.0 / rayDir[axis];
			double t0 = (boxMin[axis] - rayOrg[axis]) * invD;
			double t1 = (boxMax[axis] - rayOrg[axis]) * invD;
			if (t0 > t1)
			{
				std::swap(t0, t1);
			}

			tNear = std::max(tNear, t0);
			tFar = std::min(tFar, t1);
			if (tNear > tFar)
			{
				return false;
			}
		}
		return true;
	}
};

}  // namespace mvsim::rt
