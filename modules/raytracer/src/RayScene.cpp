/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/raytracer/RayScene.h>

using namespace mvsim::rt;

void RayScene::build()
{
	std::vector<AABB> boxes;
	boxes.reserve(primitives_.size());
	for (const auto& p : primitives_)
	{
		boxes.push_back(primitiveAABB(p));
	}
	bvh_.build(boxes);
}

std::optional<Hit> RayScene::castRay(const Ray& ray) const
{
	Hit hit;
	const bool found = bvh_.traverse(
		ray,
		[this](uint32_t primIndex, const Ray& r, Hit& outHit)
		{ return intersectPrimitive(primitives_[primIndex], r, outHit); },
		hit);
	if (!found)
	{
		return std::nullopt;
	}
	return hit;
}
