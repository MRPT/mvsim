/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/raytracer/AABB.h>
#include <mvsim/raytracer/Ray.h>

#include <cstdint>
#include <functional>
#include <vector>

namespace mvsim::rt
{
/** A simple median-split BVH over a set of AABBs. Const-only after build(),
 * so a single instance may be traversed from multiple threads concurrently.
 *
 * This intentionally does not implement a surface-area-heuristic build:
 * scenes here run to a few thousand primitives, where a median-split BVH
 * already reduces a cast to a handful of node visits.
 */
class BVH
{
   public:
	/** (Re)builds the tree over the given per-primitive bounding boxes.
	 * `boxes[i]` must correspond to primitive index `i`. */
	void build(const std::vector<AABB>& boxes);

	bool empty() const { return nodes_.empty(); }

	/** Visits every primitive whose leaf AABB the ray may intersect, nearest
	 * first among sibling subtrees (not globally sorted), calling
	 * `testPrim(primIndex, ray)` for each; `testPrim` must return true and
	 * fill `outHit` only on a strictly closer hit than any hit so far (i.e.
	 * it is expected to check against `ray.tMax`, which this function
	 * shrinks as better hits are found). Returns true iff any hit was
	 * found. */
	bool traverse(
		Ray ray,
		const std::function<bool(uint32_t primIndex, const Ray& ray, Hit& outHit)>& testPrim,
		Hit& outHit) const;

   private:
	struct Node
	{
		AABB bounds;
		int32_t leftChild = -1;	 //!< -1 for a leaf
		int32_t rightChild = -1;
		uint32_t start = 0;	 //!< Leaf only: offset into primIndices_
		uint32_t count = 0;	 //!< Leaf only: number of primitives
	};

	int32_t buildRecursive(
		std::vector<uint32_t>& order, uint32_t start, uint32_t end, const std::vector<AABB>& boxes);

	std::vector<Node> nodes_;
	std::vector<uint32_t> primIndices_;

	static constexpr uint32_t kLeafSize = 4;
};

}  // namespace mvsim::rt
