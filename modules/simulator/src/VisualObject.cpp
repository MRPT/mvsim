/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/version.h>
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
double VisualObject::GeometryEpsilon = 1e-3;

VisualObject::VisualObject(
	World* parent, bool insertCustomVizIntoViz, bool insertCustomVizIntoPhysical)
	: world_(parent),
	  insertCustomVizIntoViz_(insertCustomVizIntoViz),
	  insertCustomVizIntoPhysical_(insertCustomVizIntoPhysical)
{
	glCollision_ = mrpt::opengl::CSetOfObjects::Create();
	glCollision_->setName("bbox");
}

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
	const auto objectPose = viz.has_value() ? meSim->getPose() : meSim->getPoseNoLock();

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

			if (insertCustomVizIntoPhysical_) physical->get().insert(glCustomVisual_);
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

			mrpt::opengl::CPolyhedron::Ptr glCS;

			try
			{
				glCS = mrpt::opengl::CPolyhedron::CreateCustomPrism(c, height);
			}
			catch (const std::exception& e)
			{
#if 0
				std::cerr << "[mvsim::VisualObject] **WARNING**: Ignoring the "
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
				glCS = mrpt::opengl::CPolyhedron::CreateCustomPrism(p, height);
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

	const bool childrenOnly = !!glCustomVisual_;

	internalGuiUpdate(viz, physical, childrenOnly);
}

void VisualObject::FreeOpenGLResources() { ModelsCache::Instance().clear(); }

bool VisualObject::parseVisual(const rapidxml::xml_node<char>& rootNode)
{
	MRPT_TRY_START

	bool any = false;
	for (auto n = rootNode.first_node("visual"); n; n = n->next_sibling("visual"))
	{
		bool hasViz = implParseVisual(*n);
		any = any || hasViz;
	}
	return any;

	MRPT_TRY_END
}

bool VisualObject::parseVisual(const JointXMLnode<>& rootNode)
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

bool VisualObject::implParseVisual(const rapidxml::xml_node<char>& visNode)
{
	MRPT_TRY_START

	{
		bool visualEnabled = true;
		TParameterDefinitions auxPar;
		auxPar["enabled"] = TParamEntry("%bool", &visualEnabled);
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
	params["model_split_size"] = TParamEntry("%f", &opts.splitSize);
	params["name"] = TParamEntry("%s", &objectName);

	// Parse XML params:
	parse_xmlnode_children_as_param(visNode, params);

	if (modelURI.empty()) return false;

	const std::string localFileName = world_->xmlPathToActualPath(modelURI);

	auto& gModelsCache = ModelsCache::Instance();

	auto glModel = gModelsCache.get(localFileName, opts);

	// Add the 3D model as custom viz:
	addCustomVisualization(
		glModel, mrpt::poses::CPose3D(modelPose), modelScale, objectName, modelURI);

	return true;  // yes, we have a custom viz model

	MRPT_TRY_END
}

void VisualObject::showCollisionShape(bool show)
{
	if (!glCollision_) return;
	glCollision_->setVisibility(show);
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

void VisualObject::addCustomVisualization(
	const mrpt::opengl::CRenderizable::Ptr& glModel, const mrpt::poses::CPose3D& modelPose,
	const float modelScale, const std::string& modelName,
	const std::optional<std::string>& modelURI, const bool initialShowBoundingBox)
{
	ASSERT_(glModel);

	auto& chc = CollisionShapeCache::Instance();

	float zMin = -std::numeric_limits<float>::max();
	float zMax = std::numeric_limits<float>::max();

	if (const Block* block = dynamic_cast<const Block*>(this);
		block && !block->default_block_z_min_max())
	{
		zMin = block->block_z_min() - GeometryEpsilon;
		zMax = block->block_z_max() + GeometryEpsilon;
	}
#if 0
	std::cout << "MODEL: " << (modelURI ? *modelURI : "none")
			  << " glModel: " << glModel->GetRuntimeClass()->className
			  << " modelScale: " << modelScale << " zmin=" << zMin
			  << " zMax:" << zMax << "\n";
#endif

	// Calculate its convex hull:
	const auto shape = chc.get(*glModel, zMin, zMax, modelPose, modelScale, modelURI);

	auto glGroup = mrpt::opengl::CSetOfObjects::Create();

	// Note: we cannot apply pose/scale to the original glModel since
	// it may be shared (many instances of the same object):
	glGroup->insert(glModel);
	glGroup->setScale(modelScale);
	glGroup->setPose(modelPose);

	glGroup->setName(modelName);

	if (!glCustomVisual_)
	{
		glCustomVisual_ = mrpt::opengl::CSetOfObjects::Create();
		glCustomVisual_->setName("glCustomVisual");
	}
	glCustomVisual_->insert(glGroup);

	if (glCollision_) glCollision_->setVisibility(initialShowBoundingBox);

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
}
