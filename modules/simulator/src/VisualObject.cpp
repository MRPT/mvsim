/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/get_env.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>
#include <mvsim/Simulable.h>
#include <mvsim/VisualObject.h>
#include <mvsim/World.h>

#include <atomic>
#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;

static std::atomic_int32_t g_uniqueCustomVisualId = 0;

VisualObject::~VisualObject() = default;

void VisualObject::guiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical)
{
	using namespace std::string_literals;

	const auto* meSim = dynamic_cast<Simulable*>(this);
	ASSERT_(meSim);

	// If "viz" does not have a value, it's because we are already inside a
	// setPose() change event, so my caller already holds the mutex and we don't
	// need/can't acquire it again:
	const auto objectPose =
		viz.has_value() ? meSim->getPose() : meSim->getPoseNoLock();

	if (glCustomVisual_ && viz.has_value() && physical.has_value())
	{
		// Assign a unique ID on first call:
		if (glCustomVisualId_ < 0)
		{
			// Assign a unique name, so we can localize the object in the scene
			// if needed.
			glCustomVisualId_ = g_uniqueCustomVisualId++;
			const auto name = "_autoViz"s + std::to_string(glCustomVisualId_);
			glCustomVisual_->setName(name);

			// Add to the 3D scene:
			if (insertCustomVizIntoViz_) viz->get().insert(glCustomVisual_);

			if (insertCustomVizIntoPhysical_)
				physical->get().insert(glCustomVisual_);
		}

		// Update pose:
		glCustomVisual_->setPose(objectPose);
	}

	if (glBoundingBox_ && viz.has_value())
	{
		if (glBoundingBox_->empty())
		{
			auto glBox = mrpt::opengl::CBox::Create();
			glBox->setWireframe(true);
			glBox->setBoxCorners(viz_bbmin_, viz_bbmax_);
			glBoundingBox_->insert(glBox);
			glBoundingBox_->setVisibility(false);
			viz->get().insert(glBoundingBox_);
		}
		glBoundingBox_->setPose(objectPose);
	}

	const bool childrenOnly = !!glCustomVisual_;

	internalGuiUpdate(viz, physical, childrenOnly);
}

class ModelsCache
{
   public:
	std::map<std::string, mrpt::opengl::CAssimpModel::Ptr> cache;

	static ModelsCache& Instance()
	{
		static ModelsCache o;
		return o;
	}

   private:
	ModelsCache() = default;
	~ModelsCache() = default;
};

void VisualObject::FreeOpenGLResources()
{
	ModelsCache::Instance().cache.clear();
}

bool VisualObject::parseVisual(const rapidxml::xml_node<char>* visual_node)
{
	MRPT_TRY_START

	glBoundingBox_ = mrpt::opengl::CSetOfObjects::Create();
	glBoundingBox_->setName("bbox");

	if (visual_node == nullptr) return false;

	std::string modelURI;
	double modelScale = 1.0;
	mrpt::math::TPose3D modelPose;
	bool initialShowBoundingBox = false;

	std::string modelCull = "NONE";
	mrpt::img::TColor modelColor = mrpt::img::TColor::white();

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
	params["model_cull_faces"] = TParamEntry("%s", &modelCull);
	params["model_color"] = TParamEntry("%color", &modelColor);

	// Parse XML params:
	parse_xmlnode_children_as_param(*visual_node, params);

	if (modelURI.empty()) return false;

	const std::string localFileName = world_->xmlPathToActualPath(modelURI);
	ASSERT_FILE_EXISTS_(localFileName);

	auto& gModelsCache = ModelsCache::Instance();

	auto glGroup = mrpt::opengl::CSetOfObjects::Create();

	auto glModel = [&]() {
		if (auto it = gModelsCache.cache.find(localFileName);
			it != gModelsCache.cache.end())
			return it->second;
		else
		{
			auto m = gModelsCache.cache[localFileName] =
				mrpt::opengl::CAssimpModel::Create();

			// En/Dis-able the extra verbosity while loading the 3D model:
			int loadFlags =
				mrpt::opengl::CAssimpModel::LoadFlags::RealTimeMaxQuality |
				mrpt::opengl::CAssimpModel::LoadFlags::FlipUVs;

#if MRPT_VERSION >= 0x250
			if (modelColor != mrpt::img::TColor::white())
				loadFlags |=
					mrpt::opengl::CAssimpModel::LoadFlags::IgnoreMaterialColor;

			m->setColor_u8(modelColor);
#endif
			if (mrpt::get_env<bool>("MVSIM_LOAD_MODELS_VERBOSE", false))
				loadFlags |= mrpt::opengl::CAssimpModel::LoadFlags::Verbose;

			m->loadScene(localFileName, loadFlags);

#if MRPT_VERSION >= 0x240
			m->cullFaces(
				mrpt::typemeta::TEnumType<mrpt::opengl::TCullFace>::name2value(
					modelCull));
#endif

			return m;
		}
	}();

	mrpt::math::TPoint3D bbmin, bbmax;
#if MRPT_VERSION >= 0x218
	const auto bb = glModel->getBoundingBox();
	bbmin = bb.min;
	bbmax = bb.max;
#else
	glModel->getBoundingBox(bbmin, bbmax);
#endif
	glGroup->insert(glModel);

	glGroup->setScale(modelScale);
	glGroup->setPose(modelPose);
	glGroup->setName("group");

	glCustomVisual_ = mrpt::opengl::CSetOfObjects::Create();
	glCustomVisual_->setName("glCustomVisual");
	glCustomVisual_->insert(glGroup);
	glBoundingBox_->setVisibility(initialShowBoundingBox);

	// Auto bounds from visual model bounding-box:

	// Apply transformation to bounding box too:
	viz_bbmin_ = modelPose.composePoint(bbmin * modelScale);
	viz_bbmax_ = modelPose.composePoint(bbmax * modelScale);

	return true;
	MRPT_TRY_END
}

void VisualObject::showBoundingBox(bool show)
{
	if (!glBoundingBox_) return;
	glBoundingBox_->setVisibility(show);
}

void VisualObject::customVisualVisible(const bool visible)
{
	if (!glCustomVisual_) return;

	glCustomVisual_->setVisibility(visible);
}

bool VisualObject::customVisualVisible() const
{
	return glCustomVisual_ && glCustomVisual_->isVisible();
}
