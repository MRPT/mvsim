/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/viz/CAssimpModel.h>

#include <map>

namespace mvsim
{
class ModelsCache
{
   public:
	static ModelsCache& Instance();

	struct Options
	{
		mrpt::img::TColor modelColor = mrpt::img::TColor::white();
		std::string modelCull = "NONE";

		/** See mrpt::viz::CAssimpModel::split_triangles_rendering_bbox().
		 *  Default (0)=disabled. Any other value, split the model into voxels of this size
		 *  to help sorting triangles by depth so semitransparent meshes are rendered correctly.
		 */
		float splitSize = .0f;
	};

	mrpt::viz::CAssimpModel::Ptr get(const std::string& url, const Options& options);

	void clear() { cache.clear(); }

   private:
	ModelsCache() = default;
	~ModelsCache() = default;

	ModelsCache(const ModelsCache&) = default;
	ModelsCache& operator=(const ModelsCache&) = default;
	ModelsCache(ModelsCache&&) = default;
	ModelsCache& operator=(ModelsCache&&) = default;

	std::map<std::string, mrpt::viz::CAssimpModel::Ptr> cache;
};

}  // namespace mvsim
