/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/World.h>
#include <mvsim/WorldElements/SkyBox.h>

//
#include <mrpt/version.h>
#if MRPT_VERSION >= 0x270
#include <mrpt/opengl/CSkyBox.h>  // class introduced in mrpt 2.7.0
#else
#endif

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

SkyBox::SkyBox(World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent)
{
	// Create opengl object: in this class, we'll store most state data directly
	// in the mrpt::opengl object.
	loadConfigFrom(root);
}

SkyBox::~SkyBox() = default;

void SkyBox::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	if (!root) return;	// Assume defaults

	std::string texturesPattern;

	TParameterDefinitions params;
	params["textures"] = TParamEntry("%s", &texturesPattern);

	parse_xmlnode_children_as_param(
		*root, params, world_->user_defined_variables());

	ASSERTMSG_(
		!texturesPattern.empty(),
		"Textures must be defined in a <textures>...</textures> tag.");

	ASSERTMSG_(
		texturesPattern.find("%s") != std::string::npos,
		"Texture pattern URI in <textures>...</textures> must contain one '%s' "
		"placeholder.");

#if MRPT_VERSION >= 0x270
	using mrpt::opengl::CUBE_TEXTURE_FACE;

	const std::vector<std::pair<CUBE_TEXTURE_FACE, const char*>> faceImages = {
		{CUBE_TEXTURE_FACE::FRONT, "Front"},
		{CUBE_TEXTURE_FACE::BACK, "Back"},
		{CUBE_TEXTURE_FACE::BOTTOM, "Down"},
		{CUBE_TEXTURE_FACE::TOP, "Up"},
		{CUBE_TEXTURE_FACE::LEFT, "Left"},
		{CUBE_TEXTURE_FACE::RIGHT, "Right"},
	};

	auto sb = mrpt::opengl::CSkyBox::Create();

	for (const auto& p : faceImages)
	{
		std::string fil = mrpt::format(texturesPattern.c_str(), p.second);

		// MRPT_LOG_DEBUG_STREAM("Loading face texture: " << fil);
		fil = world_->xmlPathToActualPath(fil);

		sb->assignImage(p.first, mrpt::img::CImage::LoadFromFile(fil));
	}

	glSkyBoxPrepared_ = sb;
#else
	std::cerr
		<< "[mvsim::SkyBox] Ignoring SkyBox since MRPT>=2.7.0 is not available."
		<< std::endl;
#endif
}

void SkyBox::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	using namespace mrpt::math;

	// 1st call?
	if (!glSkyBox_ && viz && physical && glSkyBoxPrepared_)
	{
		glSkyBox_ = glSkyBoxPrepared_;
		viz->get().insert(glSkyBox_);
		physical->get().insert(glSkyBox_);
	}

	// No need to update, this is a static, "background" entity.
}
