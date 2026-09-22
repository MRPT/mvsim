/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/raytracer/BVH.h>
#include <mvsim/raytracer/Primitive.h>

#include <optional>
#include <vector>

namespace mvsim::rt
{
/** A static collection of ray-traceable primitives, indexed by a BVH for
 * fast casting. Build once via addPrimitive()/build(), then castRay() from
 * as many threads as desired: after build(), the scene is read-only. */
class RayScene
{
   public:
	void addPrimitive(Primitive prim) { primitives_.push_back(std::move(prim)); }

	/** Builds the BVH over the primitives added so far. Must be called once,
	 * after the last addPrimitive() and before any castRay(). */
	void build();

	size_t primitiveCount() const { return primitives_.size(); }
	const Primitive& primitive(size_t i) const { return primitives_[i]; }

	/** Casts one ray and returns the nearest hit within
	 * [ray.tMin, ray.tMax], if any. Thread-safe (const, no shared mutable
	 * state). */
	std::optional<Hit> castRay(const Ray& ray) const;

   private:
	std::vector<Primitive> primitives_;
	BVH bvh_;
};

}  // namespace mvsim::rt
