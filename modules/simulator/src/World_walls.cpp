/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/World.h>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

void World::processLoadWalls(const rapidxml::xml_node<char>& node)
{
	MRPT_START

	// Sanity checks:
	ASSERT_(0 == strcmp(node.name(), "walls"));

	std::string wallModelFileName;
	double wallThickness = 0.10;
	double scale = 1.0;

	TParameterDefinitions params;
	params["model_uri"] = TParamEntry("%s", &wallModelFileName);
	params["wallThickness"] = TParamEntry("%lf", &wallThickness);
	params["scale"] = TParamEntry("%lf", &scale);

	// Parse XML params:
	parse_xmlnode_children_as_param(node, params);

	ASSERT_(!wallModelFileName.empty());
	const std::string localFileName = xmlPathToActualPath(wallModelFileName);
	ASSERT_FILE_EXISTS_(localFileName);

	MRPT_LOG_DEBUG_STREAM(
		"Loading walls definition model from: " << localFileName);

	auto glModel = mrpt::opengl::CAssimpModel::Create();
	glModel->loadScene(localFileName);

	const auto& points = glModel->shaderWireframeVertexPointBuffer();
	MRPT_LOG_DEBUG_STREAM("Walls loaded, " << points.size() << " segments.");

	for (const auto& pt : points)
	{
		std::cout << "pt: " << pt << "\n";
	}

	MRPT_END
}
