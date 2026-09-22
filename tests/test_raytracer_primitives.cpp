/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/math/TPoint3D.h>
#include <mvsim/raytracer/Primitive.h>

#include <cmath>
#include <cstdio>

#include "test_utils.h"

int g_failures = 0;

using namespace mvsim::rt;
using mrpt::math::TPoint2D;
using mrpt::math::TPoint3D;

namespace
{
Ray makeRay(const TPoint3D& org, TPoint3D dir)
{
	const double n = dir.norm();
	dir.x /= n;
	dir.y /= n;
	dir.z /= n;
	Ray r;
	r.org = org;
	r.dir = dir;
	return r;
}

// ---------------------------------------------------------------
void test_plane()
{
	Plane pl;
	pl.center = {0, 0, 1.0};
	pl.normal = {0, 0, 1};
	pl.uAxis = {1, 0, 0};
	pl.vAxis = {0, 1, 0};
	pl.halfU = 5.0;
	pl.halfV = 5.0;
	Primitive prim{pl, "floor"};

	// Straight down onto the plane from above:
	{
		const Ray r = makeRay({1, 2, 10}, {0, 0, -1});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
		EXPECT_NEAR(h.t, 9.0, 1e-9);
		EXPECT_NEAR(h.normal.z, 1.0, 1e-9);
	}
	// Ray missing the finite extent:
	{
		const Ray r = makeRay({100, 2, 10}, {0, 0, -1});
		Hit h;
		EXPECT_FALSE(intersectPrimitive(prim, r, h));
	}
	// Ray parallel to the plane never hits:
	{
		const Ray r = makeRay({0, 0, 5}, {1, 0, 0});
		Hit h;
		EXPECT_FALSE(intersectPrimitive(prim, r, h));
	}
	// Hitting from below: normal must still face the ray.
	{
		const Ray r = makeRay({0, 0, -10}, {0, 0, 1});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
		EXPECT_NEAR(h.normal.z, -1.0, 1e-9);
	}
}

// ---------------------------------------------------------------
void test_sphere()
{
	Sphere sp;
	sp.center = {2, 0, 0};
	sp.radius = 1.0;
	Primitive prim{sp, "ball"};

	const Ray r = makeRay({0, 0, 0}, {1, 0, 0});
	Hit h;
	EXPECT_TRUE(intersectPrimitive(prim, r, h));
	EXPECT_NEAR(h.t, 1.0, 1e-9);  // Hits the near side, at x=1
	EXPECT_NEAR(h.normal.x, -1.0, 1e-9);

	// Ray missing entirely:
	const Ray rMiss = makeRay({0, 5, 0}, {1, 0, 0});
	Hit h2;
	EXPECT_FALSE(intersectPrimitive(prim, rMiss, h2));
}

// ---------------------------------------------------------------
void test_box_prism()
{
	// A 2x2 box footprint centered at origin, from z=0 to z=3.
	Prism box;
	box.contour = mrpt::math::TPolygon2D(
		{TPoint2D(-1, -1), TPoint2D(1, -1), TPoint2D(1, 1), TPoint2D(-1, 1)});
	box.zMin = 0.0;
	box.zMax = 3.0;
	Primitive prim{box, "box"};

	// Straight through the top face, from above:
	{
		const Ray r = makeRay({0, 0, 10}, {0, 0, -1});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
		EXPECT_NEAR(h.t, 7.0, 1e-9);  // 10 -> z=3
		EXPECT_NEAR(h.normal.z, 1.0, 1e-9);
	}
	// Through a side wall, mid-height:
	{
		const Ray r = makeRay({-10, 0, 1.5}, {1, 0, 0});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
		EXPECT_NEAR(h.t, 9.0, 1e-9);  // -10 -> x=-1
		EXPECT_NEAR(h.normal.x, -1.0, 1e-6);
		EXPECT_NEAR(h.normal.z, 0.0, 1e-9);
	}
	// Missing the box in XY entirely:
	{
		const Ray r = makeRay({-10, 5, 1.5}, {1, 0, 0});
		Hit h;
		EXPECT_FALSE(intersectPrimitive(prim, r, h));
	}
	// Missing due to z (passes over the top):
	{
		const Ray r = makeRay({-10, 0, 10}, {1, 0, 0});
		Hit h;
		EXPECT_FALSE(intersectPrimitive(prim, r, h));
	}
	// Diagonal ray through a corner region, must hit the nearer wall/top consistently:
	{
		const Ray r = makeRay({-10, -10, 1.0}, {1, 1, 0});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
		// Should enter around x=y=-1
		const TPoint3D p = r.at(h.t);
		EXPECT_NEAR(p.x, -1.0, 1e-6);
		EXPECT_NEAR(p.y, -1.0, 1e-6);
	}
}

// ---------------------------------------------------------------
void test_lshape_prism_nonconvex()
{
	// An "L" shaped footprint (non-convex), z in [0,1].
	// Full square [0,4]x[0,4] minus the [2,4]x[2,4] quadrant.
	Prism lshape;
	lshape.contour = mrpt::math::TPolygon2D(
		{TPoint2D(0, 0), TPoint2D(4, 0), TPoint2D(4, 2), TPoint2D(2, 2), TPoint2D(2, 4),
		 TPoint2D(0, 4)});
	lshape.zMin = 0.0;
	lshape.zMax = 1.0;
	Primitive prim{lshape, "lshape"};

	// Ray through the notch (removed quadrant) must miss:
	{
		const Ray r = makeRay({3, 3, 0.5}, {0, 0, 1});
		Hit h;
		EXPECT_FALSE(intersectPrimitive(prim, r, h));
	}
	// Ray through the solid part of the L must hit the top:
	{
		const Ray r = makeRay({1, 1, 10}, {0, 0, -1});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
		EXPECT_NEAR(h.t, 9.0, 1e-9);
	}
	{
		const Ray r = makeRay({1, 3, 10}, {0, 0, -1});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
		EXPECT_NEAR(h.t, 9.0, 1e-9);
	}
}

// ---------------------------------------------------------------
void test_cylinder_exact_curved_surface()
{
	Cylinder cyl;
	cyl.pose = mrpt::poses::CPose3D::Identity();  // axis along world +Z, base at origin
	cyl.radius = 0.5;
	cyl.length = 2.0;
	Primitive prim{cyl, "cyl"};

	// Ray through the side, horizontally, at mid-height: exact radius, no faceting.
	{
		const Ray r = makeRay({-10, 0, 1.0}, {1, 0, 0});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
		EXPECT_NEAR(h.t, 9.5, 1e-9);  // -10 -> x=-0.5
		EXPECT_NEAR(h.normal.x, -1.0, 1e-9);
	}
	// Grazing ray at exactly radius+epsilon must miss; at radius-epsilon must hit,
	// verifying the surface is a true circle (not an N-gon facet).
	{
		const double y = 0.5 - 1e-4;
		const Ray r = makeRay({-10, y, 1.0}, {1, 0, 0});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
	}
	{
		const double y = 0.5 + 1e-3;
		const Ray r = makeRay({-10, y, 1.0}, {1, 0, 0});
		Hit h;
		EXPECT_FALSE(intersectPrimitive(prim, r, h));
	}
	// Straight down onto the top cap:
	{
		const Ray r = makeRay({0, 0, 10}, {0, 0, -1});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
		EXPECT_NEAR(h.t, 8.0, 1e-9);  // 10 -> z=2
		EXPECT_NEAR(h.normal.z, 1.0, 1e-9);
	}
	// Straight down the axis (degenerate side-quadratic): only caps apply.
	{
		const Ray r = makeRay({0, 0, 10}, {0, 0, -1});
		Hit h;
		EXPECT_TRUE(intersectPrimitive(prim, r, h));
	}
	// Missing the cylinder radius entirely:
	{
		const Ray r = makeRay({-10, 5, 1.0}, {1, 0, 0});
		Hit h;
		EXPECT_FALSE(intersectPrimitive(prim, r, h));
	}
}

// ---------------------------------------------------------------
void test_cylinder_rotated_pose()
{
	// Cylinder lying on its side: axis along world +X, base at (1,0,0).
	Cylinder cyl;
	cyl.pose = mrpt::poses::CPose3D::FromYawPitchRoll(0, M_PI / 2, 0);
	cyl.pose.x(1.0);
	cyl.radius = 0.3;
	cyl.length = 2.0;  // extends from x=1 to x=3
	Primitive prim{cyl, "cyl2"};

	const Ray r = makeRay({2, -10, 0}, {0, 1, 0});
	Hit h;
	EXPECT_TRUE(intersectPrimitive(prim, r, h));
	EXPECT_NEAR(h.t, 9.7, 1e-6);  // -10 -> y=-0.3
}

// ---------------------------------------------------------------
void test_triangle()
{
	Triangle tri;
	tri.v0 = {0, 0, 0};
	tri.v1 = {2, 0, 0};
	tri.v2 = {0, 2, 0};
	Primitive prim{tri, "tri"};

	const Ray r = makeRay({0.3, 0.3, 5}, {0, 0, -1});
	Hit h;
	EXPECT_TRUE(intersectPrimitive(prim, r, h));
	EXPECT_NEAR(h.t, 5.0, 1e-9);

	// Outside the triangle:
	const Ray rMiss = makeRay({1.9, 1.9, 5}, {0, 0, -1});
	Hit h2;
	EXPECT_FALSE(intersectPrimitive(prim, rMiss, h2));
}

// ---------------------------------------------------------------
void test_heightfield_flat_matches_plane()
{
	// A flat 5x5 grid at z=2, resolution 1, spanning [0,4]x[0,4].
	HeightField hf;
	hf.z.setSize(5, 5);
	hf.z.fill(2.0);
	hf.minX = 0.0;
	hf.minY = 0.0;
	hf.resolution = 1.0;
	Primitive prim{hf, "terrain"};

	const Ray r = makeRay({2, 2, 10}, {0, 0, -1});
	Hit h;
	EXPECT_TRUE(intersectPrimitive(prim, r, h));
	EXPECT_NEAR(h.t, 8.0, 1e-6);
	EXPECT_NEAR(h.normal.z, 1.0, 1e-6);
}

// ---------------------------------------------------------------
void test_heightfield_slope()
{
	// A ramp: z = x, over a 5x5 grid, resolution 1, x,y in [0,4].
	HeightField hf;
	hf.z.setSize(5, 5);
	for (int r = 0; r < 5; r++)
	{
		for (int c = 0; c < 5; c++)
		{
			hf.z(r, c) = static_cast<double>(c);
		}
	}
	hf.minX = 0.0;
	hf.minY = 0.0;
	hf.resolution = 1.0;
	Primitive prim{hf, "ramp"};

	// Straight down at x=2 should hit at z=2:
	const Ray r = makeRay({2.0, 2.0, 10}, {0, 0, -1});
	Hit h;
	EXPECT_TRUE(intersectPrimitive(prim, r, h));
	const auto p = r.at(h.t);
	EXPECT_NEAR(p.z, 2.0, 1e-6);
}

// ---------------------------------------------------------------
void test_aabb()
{
	Sphere sp;
	sp.center = {1, 2, 3};
	sp.radius = 0.5;
	Primitive prim{sp, "s"};
	const AABB box = primitiveAABB(prim);
	EXPECT_NEAR(box.min.x, 0.5, 1e-9);
	EXPECT_NEAR(box.max.x, 1.5, 1e-9);
	EXPECT_TRUE(box.isValid());
}

}  // namespace

// ---------------------------------------------------------------
int main()
{
	test_plane();
	test_sphere();
	test_box_prism();
	test_lshape_prism_nonconvex();
	test_cylinder_exact_curved_surface();
	test_cylinder_rotated_pose();
	test_triangle();
	test_heightfield_flat_matches_plane();
	test_heightfield_slope();
	test_aabb();

	if (g_failures == 0)
	{
		std::printf("All raytracer primitive tests passed.\n");
	}
	else
	{
		std::fprintf(stderr, "%d test(s) FAILED.\n", g_failures);
	}
	return g_failures == 0 ? 0 : 1;
}
