/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/raytracer/BVH.h>

#include <algorithm>
#include <numeric>

using namespace mvsim::rt;

void BVH::build(const std::vector<AABB>& boxes)
{
	nodes_.clear();
	primIndices_.clear();

	if (boxes.empty())
	{
		return;
	}

	std::vector<uint32_t> order(boxes.size());
	std::iota(order.begin(), order.end(), 0u);

	nodes_.reserve(2 * boxes.size());
	buildRecursive(order, 0, static_cast<uint32_t>(order.size()), boxes);
}

int32_t BVH::buildRecursive(
	std::vector<uint32_t>& order, uint32_t start, uint32_t end, const std::vector<AABB>& boxes)
{
	AABB bounds;
	for (uint32_t i = start; i < end; i++)
	{
		bounds.grow(boxes[order[i]]);
	}

	const uint32_t count = end - start;
	const auto nodeIdx = static_cast<int32_t>(nodes_.size());
	nodes_.push_back(Node{});
	nodes_[nodeIdx].bounds = bounds;

	if (count <= kLeafSize)
	{
		nodes_[nodeIdx].start = static_cast<uint32_t>(primIndices_.size());
		nodes_[nodeIdx].count = count;
		for (uint32_t i = start; i < end; i++)
		{
			primIndices_.push_back(order[i]);
		}
		return nodeIdx;
	}

	// Split at the median along the widest axis of the centroid bounds.
	AABB centroidBounds;
	for (uint32_t i = start; i < end; i++)
	{
		centroidBounds.grow(boxes[order[i]].centroid());
	}
	const int axis = centroidBounds.longestAxis();

	const auto mid = start + count / 2;
	std::nth_element(
		order.begin() + start, order.begin() + mid, order.begin() + end,
		[&](uint32_t a, uint32_t b)
		{ return boxes[a].centroid()[axis] < boxes[b].centroid()[axis]; });

	// nodes_ may reallocate on push_back in the recursive calls below, so
	// re-fetch the pointer to this node each time we write to it instead of
	// keeping a reference across the calls.
	const int32_t left = buildRecursive(order, start, mid, boxes);
	const int32_t right = buildRecursive(order, mid, end, boxes);
	nodes_[nodeIdx].leftChild = left;
	nodes_[nodeIdx].rightChild = right;
	return nodeIdx;
}

bool BVH::traverse(
	Ray ray, const std::function<bool(uint32_t primIndex, const Ray& ray, Hit& outHit)>& testPrim,
	Hit& outHit) const
{
	if (nodes_.empty())
	{
		return false;
	}

	bool found = false;
	std::vector<int32_t> stack;
	stack.reserve(64);
	stack.push_back(0);

	while (!stack.empty())
	{
		const int32_t nodeIdx = stack.back();
		stack.pop_back();
		const Node& node = nodes_[nodeIdx];

		double tNear, tFar;
		if (!node.bounds.intersect(ray, tNear, tFar))
		{
			continue;
		}
		if (tNear > ray.tMax)
		{
			continue;  // Node is beyond the current best hit.
		}

		if (node.leftChild < 0)
		{
			// Leaf: test every primitive it holds.
			for (uint32_t i = 0; i < node.count; i++)
			{
				const uint32_t primIdx = primIndices_[node.start + i];
				Hit h;
				if (testPrim(primIdx, ray, h) && h.t < ray.tMax)
				{
					outHit = h;
					outHit.primIndex = primIdx;
					ray.tMax = h.t;
					found = true;
				}
			}
			continue;
		}

		// Visit the nearer child first: push the farther one first so the
		// LIFO stack pops the nearer one next.
		const Node& left = nodes_[node.leftChild];
		const Node& right = nodes_[node.rightChild];
		double lNear, lFar, rNear, rFar;
		const bool lHit = left.bounds.intersect(ray, lNear, lFar) && lNear <= ray.tMax;
		const bool rHit = right.bounds.intersect(ray, rNear, rFar) && rNear <= ray.tMax;

		if (lHit && rHit)
		{
			if (lNear <= rNear)
			{
				stack.push_back(node.rightChild);
				stack.push_back(node.leftChild);
			}
			else
			{
				stack.push_back(node.leftChild);
				stack.push_back(node.rightChild);
			}
		}
		else if (lHit)
		{
			stack.push_back(node.leftChild);
		}
		else if (rHit)
		{
			stack.push_back(node.rightChild);
		}
	}

	return found;
}
