/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "ModelsCache.h"

#include <mrpt/core/get_env.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>

using namespace mvsim;

ModelsCache& ModelsCache::Instance()
{
	static ModelsCache o;
	return o;
}

mrpt::opengl::CAssimpModel::Ptr ModelsCache::get(
	const std::string& url, const Options& options)
{
	// already cached?
	if (auto it = cache.find(url); it != cache.end()) return it->second;

	// No, it's a new model path, create its placeholder:
	auto m = cache[url] = mrpt::opengl::CAssimpModel::Create();

	// Is it a remote resource on the Internet?
	std::string localFileName;
	if (0)
	{
	}
	else
	{
		// local file:
		localFileName = url;
	}

	ASSERT_FILE_EXISTS_(localFileName);

	// En/Dis-able the extra verbosity while loading the 3D model:
	int loadFlags = mrpt::opengl::CAssimpModel::LoadFlags::RealTimeMaxQuality |
					mrpt::opengl::CAssimpModel::LoadFlags::FlipUVs;

	if (options.modelColor != mrpt::img::TColor::white())
		loadFlags |= mrpt::opengl::CAssimpModel::LoadFlags::IgnoreMaterialColor;

	m->setColor_u8(options.modelColor);

	if (mrpt::get_env<bool>("MVSIM_LOAD_MODELS_VERBOSE", false))
		loadFlags |= mrpt::opengl::CAssimpModel::LoadFlags::Verbose;

	m->loadScene(localFileName, loadFlags);

	m->cullFaces(mrpt::typemeta::TEnumType<mrpt::opengl::TCullFace>::name2value(
		options.modelCull));

	return m;
}
