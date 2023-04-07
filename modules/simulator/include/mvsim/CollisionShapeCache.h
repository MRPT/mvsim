/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/CRenderizable.h>
#include <mvsim/Shape2p5.h>

#include <map>
#include <optional>
#include <string>

namespace mvsim
{
class CollisionShapeCache
{
   public:
	static CollisionShapeCache& Instance();

	/** Computes the convex hull of a given model, or gets it from the cache */
	Shape2p5 get(
		mrpt::opengl::CRenderizable& obj, float zMin, float zMax,
		const mrpt::poses::CPose3D& modelPose, const float modelScale,
		const std::optional<std::string>& modelFile = std::nullopt);

	void clear() { cache.clear(); }

   private:
	CollisionShapeCache() = default;
	~CollisionShapeCache() = default;

	struct Entry
	{
		Shape2p5 shape;
	};

	std::map<std::string, Entry> cache;

	/** Returns a valid shape if the geometry is recognized and shape already
	 * filled in.
	 */
	std::optional<Shape2p5> processSimpleGeometries(
		const mrpt::opengl::CRenderizable& obj, float zMin, float zMax,
		const mrpt::poses::CPose3D& modelPose, const float modelScale);

	Shape2p5 processGenericGeometry(
		mrpt::opengl::CRenderizable& obj, float zMin, float zMax,
		const mrpt::poses::CPose3D& modelPose, const float modelScale);

	Shape2p5 processCylinderLike(
		const size_t actualEdgeCount, double actualRadius, float zMin,
		float zMax, const mrpt::poses::CPose3D& modelPose,
		const float modelScale);
};

}  // namespace mvsim
