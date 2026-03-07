/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "ModelsCache.h"

#include <mrpt/core/get_env.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

#include <cstdlib>
#include <cstring>

using namespace mvsim;

ModelsCache& ModelsCache::Instance()
{
	static ModelsCache o;
	return o;
}

mrpt::viz::CAssimpModel::Ptr ModelsCache::get(
	const std::string& localFileName, const Options& options)
{
	// already cached?
	if (auto it = cache.find(localFileName); it != cache.end())
	{
		return it->second;
	}

	// No, it's a new model path, create its placeholder:
	auto m = cache[localFileName] = mrpt::viz::CAssimpModel::Create();

	ASSERT_FILE_EXISTS_(localFileName);

	// En/Dis-able the extra verbosity while loading the 3D model:
	int loadFlags = mrpt::viz::CAssimpModel::LoadFlags::RealTimeMaxQuality |
					mrpt::viz::CAssimpModel::LoadFlags::FlipUVs;

	if (options.modelColor != mrpt::img::TColor::white())
	{
		loadFlags |= mrpt::viz::CAssimpModel::LoadFlags::IgnoreMaterialColor;
	}

	m->setColor_u8(options.modelColor);

	if (mrpt::get_env<bool>("MVSIM_LOAD_MODELS_VERBOSE", false))
	{
		loadFlags |= mrpt::viz::CAssimpModel::LoadFlags::Verbose;
	}

	m->loadScene(localFileName, loadFlags);

	m->cullFaces(mrpt::typemeta::TEnumType<mrpt::viz::TCullFace>::name2value(options.modelCull));

	m->split_triangles_rendering_bbox(options.splitSize);

	return m;
}
