/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "ModelsCache.h"

#include <mrpt/core/format.h>
#include <mrpt/core/get_env.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>
#include <mrpt/viz/CVisualObject.h>

#include <cstdlib>
#include <cstring>

using namespace mvsim;

ModelsCache& ModelsCache::Instance()
{
	static ModelsCache o;
	return o;
}

std::string ModelsCache::Options::asKey() const
{
	return mrpt::format(
		"|%02x%02x%02x%02x|%s|%02x%02x%02x%02x", modelColor.R, modelColor.G, modelColor.B,
		modelColor.A, modelCull.c_str(), modelEmissive.R, modelEmissive.G, modelEmissive.B,
		modelEmissive.A);
}

mrpt::viz::CAssimpModel::Ptr ModelsCache::get(
	const std::string& localFileName, const Options& options)
{
	// Models are shared, so instances with different options need their own:
	const std::string key = localFileName + options.asKey();

	auto m = mrpt::viz::CAssimpModel::Create();
	if (options.shared)
	{
		// already cached?
		if (auto it = cache.find(key); it != cache.end())
		{
			return it->second;
		}
		// No, it's a new model:
		cache[key] = m;
	}

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

	{
		const auto cf =
			mrpt::typemeta::TEnumType<mrpt::viz::TCullFace>::name2value(options.modelCull);
		for (auto& child : *m)
		{
			if (!child) continue;
			if (auto* t = dynamic_cast<mrpt::viz::VisualObjectParams_Triangles*>(child.get()))
				t->cullFaces(cf);
			if (auto* tt =
					dynamic_cast<mrpt::viz::VisualObjectParams_TexturedTriangles*>(child.get()))
				tt->cullFaces(cf);
		}
	}

	if (options.modelEmissive.A != 0)
	{
		// Parts with an emissive map in the model file keep glowing only where
		// the map is bright:
		const auto emissive = mrpt::img::TColorf(options.modelEmissive);
		for (auto& child : *m)
		{
			if (child)
			{
				child->materialEmissive(emissive);
			}
		}
	}

	return m;
}
