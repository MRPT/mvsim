/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/VisualObject.h>
#include <mvsim/World.h>

#include <atomic>
#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;

static std::atomic_int32_t g_uniqueCustomVisualId = 0;

void VisualObject::guiUpdate(mrpt::opengl::COpenGLScene& scene)
{
	using namespace std::string_literals;

	const auto objectPose = internalGuiGetVisualPose();

	if (m_glCustomVisual)
	{
		// Assign a unique ID on first call:
		if (m_glCustomVisualId < 0)
		{
			// Assign a unique name, so we can localize the object in the scene
			// if needed.
			m_glCustomVisualId = g_uniqueCustomVisualId++;
			const auto name = "_autoViz"s + std::to_string(m_glCustomVisualId);
			m_glCustomVisual->setName(name);
			// Add to the 3D scene:
			scene.insert(m_glCustomVisual);
		}

		// Update pose:
		m_glCustomVisual->setPose(objectPose);
	}

	if (m_glBoundingBox)
	{
		if (m_glBoundingBox->empty())
		{
			auto glBox = mrpt::opengl::CBox::Create();
			glBox->setWireframe(true);
			glBox->setBoxCorners(viz_bbmin_, viz_bbmax_);
			m_glBoundingBox->insert(glBox);
			m_glBoundingBox->setVisibility(false);
			scene.insert(m_glBoundingBox);
		}
		m_glBoundingBox->setPose(objectPose);
	}

	const bool childrenOnly = !!m_glCustomVisual;

	internalGuiUpdate(scene, childrenOnly);
}

bool VisualObject::parseVisual(const rapidxml::xml_node<char>* visual_node)
{
	MRPT_TRY_START

	m_glBoundingBox = mrpt::opengl::CSetOfObjects::Create();

	if (visual_node == nullptr) return false;

	std::string modelURI;
	double modelScale = 1.0;
	mrpt::math::TPose3D modelPose;
	bool initialShowBoundingBox = false;

	TParameterDefinitions params;
	params["model_uri"] = TParamEntry("%s", &modelURI);
	params["model_scale"] = TParamEntry("%lf", &modelScale);
	params["model_offset_x"] = TParamEntry("%lf", &modelPose.x);
	params["model_offset_y"] = TParamEntry("%lf", &modelPose.y);
	params["model_offset_z"] = TParamEntry("%lf", &modelPose.z);
	params["model_yaw"] = TParamEntry("%lf_deg", &modelPose.yaw);
	params["model_pitch"] = TParamEntry("%lf_deg", &modelPose.pitch);
	params["model_roll"] = TParamEntry("%lf_deg", &modelPose.roll);
	params["show_bounding_box"] = TParamEntry("%bool", &initialShowBoundingBox);

	// Parse XML params:
	parse_xmlnode_children_as_param(*visual_node, params);

	if (modelURI.empty()) return false;

	const std::string localFileName = m_world->xmlPathToActualPath(modelURI);
	ASSERT_FILE_EXISTS_(localFileName);

	auto glGroup = mrpt::opengl::CSetOfObjects::Create();
	auto glModel = mrpt::opengl::CAssimpModel::Create();

	glModel->loadScene(localFileName);

	mrpt::math::TPoint3D bbmin, bbmax;
	glModel->getBoundingBox(bbmin, bbmax);

	glGroup->insert(glModel);

	glGroup->setScale(modelScale);
	glGroup->setPose(modelPose);
	glGroup->setName("group");

	m_glCustomVisual = mrpt::opengl::CSetOfObjects::Create();
	m_glCustomVisual->insert(glGroup);
	m_glBoundingBox->setVisibility(initialShowBoundingBox);

	// Auto bounds from visual model bounding-box:

	// Apply transformation to bounding box too:
	viz_bbmin_ = modelPose.composePoint(bbmin * modelScale);
	viz_bbmax_ = modelPose.composePoint(bbmax * modelScale);

	return true;
	MRPT_TRY_END
}

void VisualObject::showBoundingBox(bool show)
{
	if (!m_glBoundingBox) return;
	m_glBoundingBox->setVisibility(show);
}
