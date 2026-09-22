/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/raytracer/Primitive.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include "VecOps.h"

using namespace mvsim::rt;
using mrpt::math::TPoint2D;
using mrpt::math::TPoint3D;

namespace
{
constexpr double kEps = 1e-9;

TPoint3D rotateForward(const mrpt::poses::CPose3D& pose, const TPoint3D& v)
{
	const auto& r = pose.getRotationMatrix();
	return {
		r(0, 0) * v.x + r(0, 1) * v.y + r(0, 2) * v.z,
		r(1, 0) * v.x + r(1, 1) * v.y + r(1, 2) * v.z,
		r(2, 0) * v.x + r(2, 1) * v.y + r(2, 2) * v.z};
}

/** Rotates a *direction* (no translation) by the inverse of `pose`'s rotation. */
TPoint3D rotateInverse(const mrpt::poses::CPose3D& pose, const TPoint3D& v)
{
	const auto& r = pose.getRotationMatrix();
	return {
		r(0, 0) * v.x + r(1, 0) * v.y + r(2, 0) * v.z,
		r(0, 1) * v.x + r(1, 1) * v.y + r(2, 1) * v.z,
		r(0, 2) * v.x + r(1, 2) * v.y + r(2, 2) * v.z};
}

bool intersectTriangleRaw(
	const TPoint3D& v0, const TPoint3D& v1, const TPoint3D& v2, const Ray& ray, Hit& out)
{
	const TPoint3D e1 = v1 - v0;
	const TPoint3D e2 = v2 - v0;
	const TPoint3D pvec = mvsim::rt::detail::cross(ray.dir, e2);
	const double det = mvsim::rt::detail::dot(e1, pvec);
	if (std::abs(det) < 1e-14)
	{
		return false;  // Ray parallel to the triangle's plane
	}
	const double invDet = 1.0 / det;
	const TPoint3D tvec = ray.org - v0;
	const double u = mvsim::rt::detail::dot(tvec, pvec) * invDet;
	if (u < -kEps || u > 1 + kEps)
	{
		return false;
	}
	const TPoint3D qvec = mvsim::rt::detail::cross(tvec, e1);
	const double v = mvsim::rt::detail::dot(ray.dir, qvec) * invDet;
	if (v < -kEps || u + v > 1 + kEps)
	{
		return false;
	}
	const double t = mvsim::rt::detail::dot(e2, qvec) * invDet;
	if (t < ray.tMin || t > ray.tMax)
	{
		return false;
	}
	out.t = t;
	TPoint3D n = mvsim::rt::detail::normalized(mvsim::rt::detail::cross(e1, e2));
	if (det < 0)
	{
		n = -n;	 // Hit the back face: flip so the normal faces the ray
	}
	out.normal = n;
	return true;
}

bool intersectPlane(const Plane& pl, const Ray& ray, Hit& out)
{
	const double denom = mvsim::rt::detail::dot(ray.dir, pl.normal);
	if (std::abs(denom) < 1e-12)
	{
		return false;
	}
	const double t = mvsim::rt::detail::dot(pl.center - ray.org, pl.normal) / denom;
	if (t < ray.tMin || t > ray.tMax)
	{
		return false;
	}
	const TPoint3D p = ray.at(t);
	const TPoint3D d = p - pl.center;
	const double u = mvsim::rt::detail::dot(d, pl.uAxis);
	const double v = mvsim::rt::detail::dot(d, pl.vAxis);
	if (std::abs(u) > pl.halfU || std::abs(v) > pl.halfV)
	{
		return false;
	}
	out.t = t;
	out.normal = denom < 0 ? pl.normal : -pl.normal;
	return true;
}

bool intersectSphere(const Sphere& sp, const Ray& ray, Hit& out)
{
	const TPoint3D oc = ray.org - sp.center;
	const double b = mvsim::rt::detail::dot(oc, ray.dir);
	const double c = mvsim::rt::detail::dot(oc, oc) - sp.radius * sp.radius;
	const double disc = b * b - c;
	if (disc < 0)
	{
		return false;
	}
	const double sq = std::sqrt(disc);
	double t = -b - sq;
	if (t < ray.tMin)
	{
		t = -b + sq;
	}
	if (t < ray.tMin || t > ray.tMax)
	{
		return false;
	}
	out.t = t;
	out.normal = mvsim::rt::detail::normalized(ray.at(t) - sp.center);
	return true;
}

bool intersectCylinder(const Cylinder& cyl, const Ray& ray, Hit& out)
{
	const TPoint3D org = cyl.pose.inverseComposePoint(ray.org);
	const TPoint3D dir = rotateInverse(cyl.pose, ray.dir);

	double bestT = ray.tMax;
	bool found = false;
	TPoint3D bestNormalLocal{0, 0, 1};

	// Side (curved) surface: solve the quadratic in the cylinder's local XY.
	const double a = dir.x * dir.x + dir.y * dir.y;
	if (a > 1e-14)
	{
		const double b = 2.0 * (org.x * dir.x + org.y * dir.y);
		const double c = org.x * org.x + org.y * org.y - cyl.radius * cyl.radius;
		const double disc = b * b - 4 * a * c;
		if (disc >= 0)
		{
			const double sq = std::sqrt(disc);
			for (const double t : {(-b - sq) / (2 * a), (-b + sq) / (2 * a)})
			{
				if (t < ray.tMin || t > bestT)
				{
					continue;
				}
				const double z = org.z + t * dir.z;
				if (z < 0 || z > cyl.length)
				{
					continue;
				}
				bestT = t;
				found = true;
				const double px = org.x + t * dir.x;
				const double py = org.y + t * dir.y;
				bestNormalLocal = {px / cyl.radius, py / cyl.radius, 0};
			}
		}
	}

	// Caps.
	if ((cyl.cappedBottom || cyl.cappedTop) && std::abs(dir.z) > 1e-14)
	{
		if (cyl.cappedBottom)
		{
			const double t = (0.0 - org.z) / dir.z;
			if (t >= ray.tMin && t < bestT)
			{
				const double px = org.x + t * dir.x;
				const double py = org.y + t * dir.y;
				if (px * px + py * py <= cyl.radius * cyl.radius)
				{
					bestT = t;
					found = true;
					bestNormalLocal = {0, 0, -1};
				}
			}
		}
		if (cyl.cappedTop)
		{
			const double t = (cyl.length - org.z) / dir.z;
			if (t >= ray.tMin && t < bestT)
			{
				const double px = org.x + t * dir.x;
				const double py = org.y + t * dir.y;
				if (px * px + py * py <= cyl.radius * cyl.radius)
				{
					bestT = t;
					found = true;
					bestNormalLocal = {0, 0, 1};
				}
			}
		}
	}

	if (!found || bestT > ray.tMax)
	{
		return false;
	}
	out.t = bestT;
	out.normal = rotateForward(cyl.pose, bestNormalLocal);
	return true;
}

/** Outward-facing 2D normal of edge `p0->p1`, oriented away from `centroid`
 * regardless of the contour's winding order. */
TPoint2D outwardEdgeNormal2D(const TPoint2D& p0, const TPoint2D& p1, const TPoint2D& centroid)
{
	const double ex = p1.x - p0.x;
	const double ey = p1.y - p0.y;
	const double len = std::sqrt(ex * ex + ey * ey);
	if (len < 1e-12)
	{
		return {0, 0};
	}
	TPoint2D n{ey / len, -ex / len};
	const TPoint2D mid{(p0.x + p1.x) * 0.5, (p0.y + p1.y) * 0.5};
	const double d = n.x * (mid.x - centroid.x) + n.y * (mid.y - centroid.y);
	if (d < 0)
	{
		n.x = -n.x;
		n.y = -n.y;
	}
	return n;
}

bool intersectPrism(const Prism& prism, const Ray& ray, Hit& out)
{
	const auto& contour = prism.contour;
	if (contour.size() < 3)
	{
		return false;
	}

	// 1) Clip against the [zMin,zMax] slab.
	double tSlabLo = ray.tMin;
	double tSlabHi = ray.tMax;
	bool zBounded = true;
	if (std::abs(ray.dir.z) < 1e-12)
	{
		if (ray.org.z < prism.zMin || ray.org.z > prism.zMax)
		{
			return false;
		}
		zBounded = false;  // Ray stays within the slab for its whole length.
	}
	else
	{
		double t0 = (prism.zMin - ray.org.z) / ray.dir.z;
		double t1 = (prism.zMax - ray.org.z) / ray.dir.z;
		if (t0 > t1)
		{
			std::swap(t0, t1);
		}
		tSlabLo = std::max(t0, ray.tMin);
		tSlabHi = std::min(t1, ray.tMax);
	}
	if (tSlabLo > tSlabHi)
	{
		return false;
	}

	// 2) Breakpoints: slab entry/exit, plus every polygon-edge crossing of
	// the ray's 2D (XY) projection, each tagged with the normal it would
	// contribute if it turns out to be the entry point.
	struct Breakpoint
	{
		double t;
		TPoint3D normal;
	};
	std::vector<Breakpoint> bps;
	bps.push_back(
		{tSlabLo, zBounded ? TPoint3D{0, 0, ray.dir.z > 0 ? -1.0 : 1.0} : TPoint3D{0, 0, 0}});
	bps.push_back(
		{tSlabHi, zBounded ? TPoint3D{0, 0, ray.dir.z > 0 ? 1.0 : -1.0} : TPoint3D{0, 0, 0}});

	const double ox = ray.org.x, oy = ray.org.y, dx = ray.dir.x, dy = ray.dir.y;
	const bool rayVerticalXY = (std::abs(dx) < 1e-12 && std::abs(dy) < 1e-12);

	TPoint2D centroid{0, 0};
	for (const auto& p : contour)
	{
		centroid.x += p.x;
		centroid.y += p.y;
	}
	centroid.x /= static_cast<double>(contour.size());
	centroid.y /= static_cast<double>(contour.size());

	if (!rayVerticalXY)
	{
		const size_t n = contour.size();
		for (size_t i = 0; i < n; i++)
		{
			const auto& p0 = contour[i];
			const auto& p1 = contour[(i + 1) % n];
			const double ex = p1.x - p0.x;
			const double ey = p1.y - p0.y;
			const double denom = dx * ey - dy * ex;
			if (std::abs(denom) < 1e-12)
			{
				continue;  // Parallel to this edge.
			}
			const double t = ((p0.x - ox) * ey - (p0.y - oy) * ex) / denom;
			const double s =
				std::abs(ex) > std::abs(ey) ? (ox + t * dx - p0.x) / ex : (oy + t * dy - p0.y) / ey;
			if (s < -1e-9 || s > 1 + 1e-9)
			{
				continue;
			}
			if (t < tSlabLo || t > tSlabHi)
			{
				continue;
			}
			const TPoint2D n2 = outwardEdgeNormal2D(p0, p1, centroid);
			bps.push_back({t, {n2.x, n2.y, 0}});
		}
	}

	std::sort(
		bps.begin(), bps.end(), [](const Breakpoint& a, const Breakpoint& b) { return a.t < b.t; });

	// 3) Walk sub-intervals; the first one that lies inside the solid gives
	// the entry hit.
	for (size_t i = 0; i + 1 < bps.size(); i++)
	{
		const double tA = bps[i].t;
		const double tB = bps[i + 1].t;
		if (tB - tA < 1e-9)
		{
			continue;  // Degenerate (coincident breakpoints).
		}
		const double tMid = 0.5 * (tA + tB);
		const TPoint2D pt2d{ox + tMid * dx, oy + tMid * dy};
		if (!contour.contains(pt2d))
		{
			continue;
		}
		const double tHit = std::max(tA, ray.tMin);
		if (tHit > ray.tMax)
		{
			return false;
		}
		out.t = tHit;
		out.normal = (bps[i].normal.x == 0 && bps[i].normal.y == 0 && bps[i].normal.z == 0)
						 ? -ray.dir	 // Degenerate: ray origin starts inside the solid.
						 : bps[i].normal;
		return true;
	}
	return false;
}

bool intersectHeightField(const HeightField& hf, const Ray& ray, Hit& out)
{
	const auto rows = hf.z.rows();
	const auto cols = hf.z.cols();
	if (rows < 2 || cols < 2)
	{
		return false;
	}
	const double maxX = hf.minX + (cols - 1) * hf.resolution;
	const double maxY = hf.minY + (rows - 1) * hf.resolution;
	const double zMin = hf.z.minCoeff();
	const double zMax = hf.z.maxCoeff();

	AABB box;
	box.grow({hf.minX, hf.minY, zMin});
	box.grow({maxX, maxY, zMax});

	double tNear, tFar;
	if (!box.intersect(ray, tNear, tFar))
	{
		return false;
	}
	tNear = std::max(tNear, ray.tMin);
	tFar = std::min(tFar, ray.tMax);
	if (tNear > tFar)
	{
		return false;
	}

	const double startX = ray.org.x + tNear * ray.dir.x;
	const double startY = ray.org.y + tNear * ray.dir.y;
	int col = static_cast<int>(std::floor((startX - hf.minX) / hf.resolution));
	int row = static_cast<int>(std::floor((startY - hf.minY) / hf.resolution));
	col = std::clamp(col, 0, static_cast<int>(cols) - 2);
	row = std::clamp(row, 0, static_cast<int>(rows) - 2);

	const int stepCol = ray.dir.x > 1e-12 ? 1 : (ray.dir.x < -1e-12 ? -1 : 0);
	const int stepRow = ray.dir.y > 1e-12 ? 1 : (ray.dir.y < -1e-12 ? -1 : 0);

	const double tDeltaX =
		stepCol != 0 ? hf.resolution / std::abs(ray.dir.x) : std::numeric_limits<double>::max();
	const double tDeltaY =
		stepRow != 0 ? hf.resolution / std::abs(ray.dir.y) : std::numeric_limits<double>::max();

	auto nextBoundaryT = [&](int idx, int step, double minCoord, double org, double dir) -> double
	{
		if (step == 0)
		{
			return std::numeric_limits<double>::max();
		}
		const double bound = minCoord + (idx + (step > 0 ? 1 : 0)) * hf.resolution;
		return (bound - org) / dir;
	};

	double tMaxX = nextBoundaryT(col, stepCol, hf.minX, ray.org.x, ray.dir.x);
	double tMaxY = nextBoundaryT(row, stepRow, hf.minY, ray.org.y, ray.dir.y);

	double t = tNear;
	while (t <= tFar + 1e-9 && col >= 0 && col < static_cast<int>(cols) - 1 && row >= 0 &&
		   row < static_cast<int>(rows) - 1)
	{
		const TPoint3D p00{
			hf.minX + col * hf.resolution, hf.minY + row * hf.resolution, hf.z(row, col)};
		const TPoint3D p10{
			hf.minX + (col + 1) * hf.resolution, hf.minY + row * hf.resolution, hf.z(row, col + 1)};
		const TPoint3D p01{
			hf.minX + col * hf.resolution, hf.minY + (row + 1) * hf.resolution, hf.z(row + 1, col)};
		const TPoint3D p11{
			hf.minX + (col + 1) * hf.resolution, hf.minY + (row + 1) * hf.resolution,
			hf.z(row + 1, col + 1)};

		Hit h1, h2;
		const bool got1 = intersectTriangleRaw(p00, p10, p11, ray, h1);
		const bool got2 = intersectTriangleRaw(p00, p11, p01, ray, h2);
		if (got1 && (!got2 || h1.t <= h2.t))
		{
			out = h1;
			return true;
		}
		if (got2)
		{
			out = h2;
			return true;
		}

		if (tMaxX < tMaxY)
		{
			t = tMaxX;
			tMaxX += tDeltaX;
			col += stepCol;
		}
		else
		{
			t = tMaxY;
			tMaxY += tDeltaY;
			row += stepRow;
		}
	}
	return false;
}

}  // namespace

AABB mvsim::rt::primitiveAABB(const Primitive& prim)
{
	return std::visit(
		[](auto&& g) -> AABB
		{
			using T = std::decay_t<decltype(g)>;
			AABB box;
			if constexpr (std::is_same_v<T, Plane>)
			{
				for (const int su : {-1, 1})
				{
					for (const int sv : {-1, 1})
					{
						box.grow(g.center + (su * g.halfU) * g.uAxis + (sv * g.halfV) * g.vAxis);
					}
				}
			}
			else if constexpr (std::is_same_v<T, Prism>)
			{
				for (const auto& p : g.contour)
				{
					box.grow({p.x, p.y, g.zMin});
					box.grow({p.x, p.y, g.zMax});
				}
			}
			else if constexpr (std::is_same_v<T, Cylinder>)
			{
				for (int i = 0; i < 8; i++)
				{
					const double lx = (i & 1) ? g.radius : -g.radius;
					const double ly = (i & 2) ? g.radius : -g.radius;
					const double lz = (i & 4) ? g.length : 0.0;
					box.grow(g.pose.composePoint(TPoint3D{lx, ly, lz}));
				}
			}
			else if constexpr (std::is_same_v<T, Sphere>)
			{
				box.grow(g.center - TPoint3D{g.radius, g.radius, g.radius});
				box.grow(g.center + TPoint3D{g.radius, g.radius, g.radius});
			}
			else if constexpr (std::is_same_v<T, Triangle>)
			{
				box.grow(g.v0);
				box.grow(g.v1);
				box.grow(g.v2);
			}
			else if constexpr (std::is_same_v<T, HeightField>)
			{
				const double maxX = g.minX + (g.z.cols() - 1) * g.resolution;
				const double maxY = g.minY + (g.z.rows() - 1) * g.resolution;
				box.grow({g.minX, g.minY, g.z.minCoeff()});
				box.grow({maxX, maxY, g.z.maxCoeff()});
			}
			return box;
		},
		prim.geometry);
}

bool mvsim::rt::intersectPrimitive(const Primitive& prim, const Ray& ray, Hit& outHit)
{
	return std::visit(
		[&](auto&& g) -> bool
		{
			using T = std::decay_t<decltype(g)>;
			if constexpr (std::is_same_v<T, Plane>)
			{
				return intersectPlane(g, ray, outHit);
			}
			else if constexpr (std::is_same_v<T, Prism>)
			{
				return intersectPrism(g, ray, outHit);
			}
			else if constexpr (std::is_same_v<T, Cylinder>)
			{
				return intersectCylinder(g, ray, outHit);
			}
			else if constexpr (std::is_same_v<T, Sphere>)
			{
				return intersectSphere(g, ray, outHit);
			}
			else if constexpr (std::is_same_v<T, Triangle>)
			{
				return intersectTriangleRaw(g.v0, g.v1, g.v2, ray, outHit);
			}
			else if constexpr (std::is_same_v<T, HeightField>)
			{
				return intersectHeightField(g, ray, outHit);
			}
			return false;
		},
		prim.geometry);
}
