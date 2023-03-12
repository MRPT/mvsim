/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/math/TPolygon2D.h>
#include <mrpt/opengl/CRenderizable.h>

#include <map>
#include <optional>
#include <string>

namespace mvsim
{
class ConvexHullCache
{
   public:
	static ConvexHullCache& Instance();

	/** Computes the convex hull of a given model, or gets it from the cache */
	mrpt::math::TPolygon2D get(
		mrpt::opengl::CRenderizable& obj, float zMin, float zMax,
		const mrpt::poses::CPose3D& modelPose, const float modelScale,
		const std::optional<std::string>& modelFile = std::nullopt);

	void clear() { cache.clear(); }

   private:
	ConvexHullCache() = default;
	~ConvexHullCache() = default;

	struct Entry
	{
		mrpt::math::TPolygon2D convexHull;
	};

	std::map<std::string, Entry> cache;
};

}  // namespace mvsim
