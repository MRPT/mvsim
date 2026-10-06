/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/version.h>
#include <mrpt/viz/CPolyhedron.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/TLightParameters.h>	// MRPT_VIZ_HAS_CLIGHT
#if defined(MRPT_VIZ_HAS_CLIGHT)
#include <mrpt/viz/CLight.h>
#endif
#include <mvsim/Block.h>
#include <mvsim/CollisionShapeCache.h>
#include <mvsim/Simulable.h>
#include <mvsim/VisualObject.h>
#include <mvsim/World.h>

#include <atomic>
#include <rapidxml.hpp>

#include "JointXMLnode.h"
#include "ModelsCache.h"
#include "xml_utils.h"

using namespace mvsim;

static std::atomic_int32_t g_uniqueCustomVisualId = 0;
double CVisualObject::GeometryEpsilon = 1e-3;

CVisualObject::CVisualObject(
	World* parent, bool insertCustomVizIntoViz, bool insertCustomVizIntoPhysical)
	: world_(parent),
	  insertCustomVizIntoViz_(insertCustomVizIntoViz),
	  insertCustomVizIntoPhysical_(insertCustomVizIntoPhysical)
{
	glCollision_ = mrpt::viz::CSetOfObjects::Create();
	glCollision_->setName("bbox");
}

CVisualObject::~CVisualObject() = default;

void CVisualObject::guiUpdate(
	const mrpt::optional_ref<mrpt::viz::Scene>& viz,
	const mrpt::optional_ref<mrpt::viz::Scene>& physical)
{
	using namespace std::string_literals;

	const auto* meSim = dynamic_cast<Simulable*>(this);
	ASSERT_(meSim);

	// If "viz" does not have a value, it's because we are already inside a
	// setPose() change event, so my caller already holds the mutex and we don't
	// need/can't acquire it again:
	const auto objectPoseOrg = viz.has_value() ? meSim->getPose() : meSim->getPoseNoLock();
	const auto objectPose = parent()->applyWorldRenderOffset(objectPoseOrg);

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
			if (insertCustomVizIntoViz_)
			{
				viz->get().insert(glCustomVisual_);
			}

			if (insertCustomVizIntoPhysical_)
			{
				physical->get().insert(glCustomVisual_);
			}
		}

		// Update pose:
		glCustomVisual_->setPose(objectPose);
	}

	if (glCollision_ && viz.has_value())
	{
		if (glCollision_->empty() && collisionShape_)
		{
			const auto& cs = collisionShape_.value();

			const double height = cs.zMax() - cs.zMin();
			ASSERT_(height == height);
			ASSERT_(height > 0);

			const auto c = cs.getContour();

			// Adapt mrpt::math geometry epsilon to the scale of the smallest
			// edge in this polygon, so we don't get false positives about
			// wrong aligned points in a 3D face just becuase it's too small:
			const auto savedMrptGeomEps = mrpt::math::getEpsilon();

			double smallestEdge = std::abs(height);
			for (size_t i = 0; i < c.size(); i++)
			{
				size_t im1 = i == 0 ? c.size() - 1 : i - 1;
				const auto Ap = c[i] - c[im1];
				mrpt::keep_min(smallestEdge, Ap.norm());
			}
			mrpt::math::setEpsilon(1e-5 * smallestEdge);

			mrpt::viz::CPolyhedron::Ptr glCS;

			try
			{
				glCS = mrpt::viz::CPolyhedron::CreateCustomPrism(c, height);
			}
			catch (const std::exception& e)
			{
#if 0
				std::cerr << "[mvsim::CVisualObject] **WARNING**: Ignoring the "
							 "following error while building the visualization "
							 "of the collision shape for object named '"
						  << meSim->getName()
						  << "' placed by pose=" << meSim->getPose()
						  << "). Falling back to rectangular collision shape "
							 "from bounding box:\n"
						  << e.what() << std::endl;
#endif

				mrpt::math::TPoint2D bbMax, bbMin;
				cs.getContour().getBoundingBox(bbMin, bbMax);
				mrpt::math::TPolygon2D p;
				p.emplace_back(bbMin.x, bbMin.y);
				p.emplace_back(bbMin.x, bbMax.y);
				p.emplace_back(bbMax.x, bbMax.y);
				p.emplace_back(bbMax.x, bbMin.y);
				glCS = mrpt::viz::CPolyhedron::CreateCustomPrism(p, height);
			}
			glCS->setWireframe(true);

			mrpt::math::setEpsilon(savedMrptGeomEps);
			// Default epsilon is restored now

			glCS->setLocation(0, 0, cs.zMin());

			glCollision_->insert(glCS);
			glCollision_->setVisibility(false);
			viz->get().insert(glCollision_);
		}
		glCollision_->setPose(objectPose);
	}

	if (glLightGroups_ && viz.has_value() && physical.has_value())
	{
		if (!glLightGroupsInserted_)
		{
			glLightGroupsInserted_ = true;
			if (insertCustomVizIntoViz_)
			{
				viz->get().insert(glLightGroups_);
			}
			// Always in the physical scene, so camera sensors see the lights:
			physical->get().insert(glLightGroups_);
		}
		glLightGroups_->setPose(objectPose);
	}

	const bool childrenOnly = !!glCustomVisual_;

	internalGuiUpdate(viz, physical, childrenOnly);
}

void CVisualObject::FreeOpenGLResources() { ModelsCache::Instance().clear(); }

bool CVisualObject::parseVisual(const rapidxml::xml_node<char>& rootNode)
{
	MRPT_TRY_START

	for (auto n = rootNode.first_node("light_group"); n; n = n->next_sibling("light_group"))
	{
		implParseLightGroup(*n);
	}

	bool any = false;
	for (auto n = rootNode.first_node("visual"); n; n = n->next_sibling("visual"))
	{
		bool hasViz = implParseVisual(*n);
		any = any || hasViz;
	}
	return any;

	MRPT_TRY_END
}

bool CVisualObject::parseVisual(const JointXMLnode<>& rootNode)
{
	MRPT_TRY_START

	bool any = false;
	for (const auto& n : rootNode.getListOfNodes())
	{
		bool hasViz = parseVisual(*n);
		any = any || hasViz;
	}

	return any;
	MRPT_TRY_END
}

bool CVisualObject::implParseVisual(const rapidxml::xml_node<char>& visNode)
{
	MRPT_TRY_START

	std::string lightGroupName;
	{
		bool visualEnabled = true;
		TParameterDefinitions auxPar;
		auxPar["enabled"] = TParamEntry("%bool", &visualEnabled);
		auxPar["light_group"] = TParamEntry("%s", &lightGroupName);
		parse_xmlnode_attribs(visNode, auxPar);
		if (!visualEnabled)
		{
			// "enabled=false" -> Ignore the rest of the contents
			return false;
		}
	}

	std::string modelURI;
	double modelScale = 1.0;
	mrpt::math::TPose3D modelPose;
	bool initialShowBoundingBox = false;
	bool castShadows = true;
	std::string objectName = "group";

	ModelsCache::Options opts;

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
	params["model_cull_faces"] = TParamEntry("%s", &opts.modelCull);
	params["model_color"] = TParamEntry("%color", &opts.modelColor);
	params["model_emissive"] = TParamEntry("%color", &opts.modelEmissive);
	params["cast_shadows"] = TParamEntry("%bool", &castShadows);
	params["name"] = TParamEntry("%s", &objectName);

	// Parse XML params:
	if (world_)
	{
		parse_xmlnode_children_as_param(visNode, params, world_->user_defined_variables());
	}
	else
	{
		parse_xmlnode_children_as_param(visNode, params);
	}

	if (modelURI.empty())
	{
		return false;
	}

	const std::string localFileName = world_->xmlPathToActualPath(modelURI);

	auto& gModelsCache = ModelsCache::Instance();

	// Models that glow while a light group is on are switched per instance:
	opts.shared = lightGroupName.empty();

	auto glModel = gModelsCache.get(localFileName, opts);

	if (!lightGroupName.empty())
	{
		auto lck = mrpt::lockHelper(lightGroupsMtx_);
		auto& g = lightGroup(lightGroupName);
		for (const auto& part : *glModel)
		{
			if (part)
			{
				g.emissiveParts.emplace_back(part, part->materialEmissive());
			}
		}
		g.apply();
	}

	// Check if this is a Block with a visual_scale override
	std::optional<double> scaleOverride;
	if (const Block* block = dynamic_cast<const Block*>(this))
	{
		const double blockScale = block->visual_scale();
		if (blockScale == blockScale)  // not NaN
		{
			scaleOverride = blockScale;
		}
	}

	// Add the 3D model as custom viz:
	auto glGroup = addCustomVisualization(
		glModel, mrpt::poses::CPose3D(modelPose), static_cast<float>(modelScale), objectName,
		modelURI, initialShowBoundingBox, scaleOverride);
	glGroup->castShadows(castShadows);

	return true;  // yes, we have a custom viz model

	MRPT_TRY_END
}

void CVisualObject::showCollisionShape(bool show)
{
	if (!glCollision_)
	{
		return;
	}
	glCollision_->setVisibility(show);
}

void CVisualObject::customVisualVisible(const bool visible)
{
	if (!glCustomVisual_)
	{
		return;
	}
	glCustomVisual_->setVisibility(visible);
}

bool CVisualObject::customVisualVisible() const
{
	return glCustomVisual_ && glCustomVisual_->isVisible();
}

mrpt::viz::CSetOfObjects::Ptr CVisualObject::addCustomVisualization(
	const mrpt::viz::CVisualObject::Ptr& glModel, const mrpt::poses::CPose3D& modelPose,
	const float modelScale, const std::string& modelName,
	const std::optional<std::string>& modelURI, const bool initialShowBoundingBox,
	const std::optional<double>& scaleOverride)
{
	ASSERT_(glModel);

	auto& chc = CollisionShapeCache::Instance();

	float zMin = -std::numeric_limits<float>::max();
	float zMax = std::numeric_limits<float>::max();

	if (const Block* block = dynamic_cast<const Block*>(this);
		block && !block->default_block_z_min_max())
	{
		zMin = static_cast<float>(block->block_z_min() - GeometryEpsilon);
		zMax = static_cast<float>(block->block_z_max() + GeometryEpsilon);
	}

	// Apply scale override if provided
	const float effectiveScale =
		scaleOverride.has_value() ? static_cast<float>(scaleOverride.value()) : modelScale;

#if 0
	std::cout << "MODEL: " << (modelURI ? *modelURI : "none")
			  << " glModel: " << glModel->GetRuntimeClass()->className
			  << " modelScale: " << modelScale << " zmin=" << zMin
			  << " zMax:" << zMax << "\n";
#endif

	// Calculate its convex hull:
	const auto shape = chc.get(*glModel, zMin, zMax, modelPose, effectiveScale, modelURI);

	auto glGroup = mrpt::viz::CSetOfObjects::Create();

	// Note: we cannot apply pose/scale to the original glModel since
	// it may be shared (many instances of the same object):
	glGroup->insert(glModel);
	glGroup->setScale(effectiveScale);
	glGroup->setPose(modelPose);

	glGroup->setName(modelName);

	if (!glCustomVisual_)
	{
		glCustomVisual_ = mrpt::viz::CSetOfObjects::Create();
		glCustomVisual_->setName("glCustomVisual");
	}
	glCustomVisual_->insert(glGroup);

	if (glCollision_)
	{
		glCollision_->setVisibility(initialShowBoundingBox);
	}

	// Auto bounds from visual model bounding-box:
	if (!collisionShape_)
	{
		// Copy:
		collisionShape_ = shape;
	}
	else
	{
		// ... or update collision volume:
		collisionShape_->mergeWith(shape);
	}

	return glGroup;
}

void CVisualObject::LightGroup::apply() const
{
	glLights->setVisibility(on);
	for (const auto& [part, emissive] : emissiveParts)
	{
		part->materialEmissive(on ? emissive : mrpt::img::TColorf(0, 0, 0, 0));
	}
}

CVisualObject::LightGroup& CVisualObject::lightGroup(const std::string& name)
{
	if (auto it = lightGroups_.find(name); it != lightGroups_.end())
	{
		return it->second;
	}

	if (!glLightGroups_)
	{
		glLightGroups_ = mrpt::viz::CSetOfObjects::Create();
		glLightGroups_->setName("light_groups");
	}
	auto& g = lightGroups_[name];
	g.glLights = mrpt::viz::CSetOfObjects::Create();
	g.glLights->setName(name);
	glLightGroups_->insert(g.glLights);
	return g;
}

void CVisualObject::implParseLightGroup(const rapidxml::xml_node<char>& node)
{
	const std::map<std::string, std::string> noVars;
	const auto& vars = world_ ? world_->user_defined_variables() : noVars;

	std::string name;
	bool initiallyOn = true;
	TParameterDefinitions attribs;
	attribs["name"] = TParamEntry("%s", &name);
	attribs["initially_on"] = TParamEntry("%bool", &initiallyOn);
	parse_xmlnode_attribs(node, attribs, vars, "[CVisualObject::light_group]");
	if (name.empty())
	{
		THROW_EXCEPTION("<light_group> tags must have a 'name' attribute");
	}

	std::vector<mrpt::viz::TLight> lights;
	for (auto* n = node.first_node(); n; n = n->next_sibling())
	{
		if (n->type() == rapidxml::node_element)
		{
			lights.push_back(parse_light_xml_node(*n, vars));
		}
	}

#if !defined(MRPT_VIZ_HAS_CLIGHT)
	if (!lights.empty() && world_)
	{
		world_->logFmt(
			mrpt::system::LVL_WARN,
			"The lights of light group '%s' are ignored: they need a newer MRPT version (only its "
			"emissive models are switched)",
			name.c_str());
	}
#endif

	auto lck = mrpt::lockHelper(lightGroupsMtx_);
	auto& g = lightGroup(name);
	g.on = initiallyOn;
#if defined(MRPT_VIZ_HAS_CLIGHT)
	for (const auto& l : lights)
	{
		g.glLights->insert(mrpt::viz::CLight::Create(l));
	}
#endif
	g.apply();
}

std::vector<std::string> CVisualObject::lightGroupNames() const
{
	auto lck = mrpt::lockHelper(lightGroupsMtx_);
	std::vector<std::string> names;
	for (const auto& [name, g] : lightGroups_)
	{
		names.push_back(name);
	}
	return names;
}

bool CVisualObject::setLightGroupState(const std::string& groupName, bool on)
{
	auto lck = mrpt::lockHelper(lightGroupsMtx_);
	auto it = lightGroups_.find(groupName);
	if (it == lightGroups_.end())
	{
		return false;
	}
	it->second.on = on;
	it->second.apply();
	return true;
}

std::optional<bool> CVisualObject::lightGroupState(const std::string& groupName) const
{
	auto lck = mrpt::lockHelper(lightGroupsMtx_);
	auto it = lightGroups_.find(groupName);
	if (it == lightGroups_.end())
	{
		return {};
	}
	return it->second.on;
}
