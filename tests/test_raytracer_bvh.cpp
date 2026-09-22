/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/math/TPoint3D.h>
#include <mvsim/raytracer/RayScene.h>

#include <cstdio>
#include <optional>
#include <random>

#include "test_utils.h"

int g_failures = 0;

using namespace mvsim::rt;
using mrpt::math::TPoint3D;

namespace
{
/** Brute-force reference: tests every primitive, no BVH. */
std::optional<Hit> bruteForceCastRay(const RayScene& scene, const Ray& ray)
{
	std::optional<Hit> best;
	Ray r = ray;
	for (size_t i = 0; i < scene.primitiveCount(); i++)
	{
		Hit h;
		if (intersectPrimitive(scene.primitive(i), r, h) && h.t < r.tMax)
		{
			h.primIndex = static_cast<uint32_t>(i);
			best = h;
			r.tMax = h.t;
		}
	}
	return best;
}

RayScene buildRandomScene(std::mt19937& rng, size_t nSpheres, size_t nBoxes)
{
	RayScene scene;
	std::uniform_real_distribution<double> posDist(-20.0, 20.0);
	std::uniform_real_distribution<double> radiusDist(0.2, 2.0);

	for (size_t i = 0; i < nSpheres; i++)
	{
		Sphere sp;
		sp.center = {posDist(rng), posDist(rng), posDist(rng)};
		sp.radius = radiusDist(rng);
		scene.addPrimitive(Primitive{sp, "sphere"});
	}
	for (size_t i = 0; i < nBoxes; i++)
	{
		const double cx = posDist(rng), cy = posDist(rng);
		const double hx = radiusDist(rng), hy = radiusDist(rng);
		Prism box;
		box.contour = mrpt::math::TPolygon2D(
			{mrpt::math::TPoint2D(cx - hx, cy - hy), mrpt::math::TPoint2D(cx + hx, cy - hy),
			 mrpt::math::TPoint2D(cx + hx, cy + hy), mrpt::math::TPoint2D(cx - hx, cy + hy)});
		box.zMin = posDist(rng);
		box.zMax = box.zMin + radiusDist(rng);
		scene.addPrimitive(Primitive{box, "box"});
	}
	scene.build();
	return scene;
}

void test_bvh_matches_bruteforce_random_scene()
{
	std::mt19937 rng(42);
	const RayScene scene = buildRandomScene(rng, 60, 60);

	std::uniform_real_distribution<double> orgDist(-30.0, 30.0);
	std::uniform_real_distribution<double> dirDist(-1.0, 1.0);

	int nBvhHits = 0;
	for (int trial = 0; trial < 4000; trial++)
	{
		TPoint3D org{orgDist(rng), orgDist(rng), orgDist(rng)};
		TPoint3D dir{dirDist(rng), dirDist(rng), dirDist(rng)};
		const double n = dir.norm();
		if (n < 1e-6)
		{
			continue;
		}
		dir.x /= n;
		dir.y /= n;
		dir.z /= n;

		Ray ray;
		ray.org = org;
		ray.dir = dir;

		const auto refHit = bruteForceCastRay(scene, ray);
		const auto bvhHit = scene.castRay(ray);

		EXPECT_TRUE(refHit.has_value() == bvhHit.has_value());
		if (refHit && bvhHit)
		{
			nBvhHits++;
			EXPECT_NEAR(refHit->t, bvhHit->t, 1e-6);
			EXPECT_NEAR(refHit->normal.x, bvhHit->normal.x, 1e-6);
			EXPECT_NEAR(refHit->normal.y, bvhHit->normal.y, 1e-6);
			EXPECT_NEAR(refHit->normal.z, bvhHit->normal.z, 1e-6);
		}
	}
	// Sanity: the random scene should actually produce a reasonable number of
	// hits, or this test would trivially pass by testing nothing.
	EXPECT_GT(nBvhHits, 200);
}

void test_bvh_empty_scene()
{
	RayScene scene;
	scene.build();
	Ray r;
	r.org = {0, 0, 0};
	r.dir = {1, 0, 0};
	EXPECT_FALSE(scene.castRay(r).has_value());
}

void test_bvh_single_primitive()
{
	RayScene scene;
	Sphere sp;
	sp.center = {5, 0, 0};
	sp.radius = 1.0;
	scene.addPrimitive(Primitive{sp, "s"});
	scene.build();

	Ray r;
	r.org = {0, 0, 0};
	r.dir = {1, 0, 0};
	const auto h = scene.castRay(r);
	EXPECT_TRUE(h.has_value());
	if (h)
	{
		EXPECT_NEAR(h->t, 4.0, 1e-9);
	}
}

}  // namespace

// ---------------------------------------------------------------
int main()
{
	test_bvh_empty_scene();
	test_bvh_single_primitive();
	test_bvh_matches_bruteforce_random_scene();

	if (g_failures == 0)
	{
		std::printf("All raytracer BVH tests passed.\n");
	}
	else
	{
		std::fprintf(stderr, "%d test(s) FAILED.\n", g_failures);
	}
	return g_failures == 0 ? 0 : 1;
}
