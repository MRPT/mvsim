/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/CAssimpModel.h>

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
	};

	mrpt::opengl::CAssimpModel::Ptr get(const std::string& url, const Options& options);

	void clear() { cache.clear(); }

   private:
	ModelsCache() = default;
	~ModelsCache() = default;

	std::map<std::string, mrpt::opengl::CAssimpModel::Ptr> cache;
};

}  // namespace mvsim
