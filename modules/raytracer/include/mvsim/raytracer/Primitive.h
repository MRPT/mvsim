/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mvsim/raytracer/AABB.h>
#include <mvsim/raytracer/Ray.h>

#include <cstdint>
#include <string>
#include <variant>

namespace mvsim::rt
{
/** A finite rectangle lying in a plane, e.g. a `<horizontal_plane>` world
 * element. `center` +/- `halfU`*`uAxis` +/- `halfV`*`vAxis` defines its
 * extent; `normal`, `uAxis`, `vAxis` must be mutually orthonormal. */
struct Plane
{
	mrpt::math::TPoint3D center{0, 0, 0};
	mrpt::math::TVector3D normal{0, 0, 1};
	mrpt::math::TVector3D uAxis{1, 0, 0};
	mrpt::math::TVector3D vAxis{0, 1, 0};
	double halfU = 1.0;
	double halfV = 1.0;
};

/** A 2D polygon (in world XY, i.e. already accounting for any yaw) extruded
 * along world Z in `[zMin, zMax]`. Covers mvsim `<block>` shapes and
 * `<vertical_plane>` wall segments. The polygon is assumed simple
 * (non-self-intersecting), and may be non-convex. */
struct Prism
{
	mrpt::math::TPolygon2D contour;
	double zMin = 0.0;
	double zMax = 1.0;
};

/** An exact circular cylinder: the local frame's +Z axis is the cylinder
 * axis, extending from local Z=0 (the base) to local Z=`length`. */
struct Cylinder
{
	mrpt::poses::CPose3D pose;
	double radius = 1.0;
	double length = 1.0;
	bool cappedBottom = true;
	bool cappedTop = true;
};

struct Sphere
{
	mrpt::math::TPoint3D center{0, 0, 0};
	double radius = 1.0;
};

/** Fallback primitive for anything not analytically representable
 * (`<geometry type="ramp"|"semi_cylinder_bump">`, `<shape_from_visual/>`). */
struct Triangle
{
	mrpt::math::TPoint3D v0, v1, v2;
};

/** A regular-grid heightfield, from `<element class="elevation_map">`.
 * `z(row,col)` gives the elevation at world point
 * `(minX + col*resolution, minY + row*resolution)`. */
struct HeightField
{
	mrpt::math::CMatrixDouble z;
	double minX = 0.0;
	double minY = 0.0;
	double resolution = 1.0;
};

using PrimitiveGeometry = std::variant<Plane, Prism, Cylinder, Sphere, Triangle, HeightField>;

/** A scene primitive plus the bookkeeping needed to trace back a hit to the
 * mvsim world entity that generated it. */
struct Primitive
{
	PrimitiveGeometry geometry;

	/** Name of the source world entity (block, world element, ...), for
	 * diagnostics and `--dump-scene`. */
	std::string objectName;
};

/** Axis-aligned bounding box enclosing `prim`. Infinite/unbounded planes are
 * not supported: `Plane` here is always a finite rectangle. */
AABB primitiveAABB(const Primitive& prim);

/** Exact ray-primitive intersection. On success, fills `outHit.t` and
 * `outHit.normal` (outward-facing) and returns true. `outHit.primIndex` is
 * left untouched (the caller, typically RayScene, fills it in). Only hits
 * with `ray.tMin <= t <= ray.tMax` are reported. */
bool intersectPrimitive(const Primitive& prim, const Ray& ray, Hit& outHit);

}  // namespace mvsim::rt
