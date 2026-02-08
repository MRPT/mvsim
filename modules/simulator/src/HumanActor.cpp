/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/wrap2pi.h>
#include <mvsim/HumanActor.h>
#include <mvsim/World.h>

#include <cmath>
#include <rapidxml.hpp>

#include "JointXMLnode.h"
#include "XMLClassesRegistry.h"
#include "parse_utils.h"
#include "xml_utils.h"

using namespace mvsim;
using namespace std;

static XmlClassesRegistry actor_classes_registry("actor:class");

// ============================================================================
// Constructor
// ============================================================================

HumanActor::HumanActor(World* parent) : VisualObject(parent), Simulable(parent) {}

// ============================================================================
// Class Registration
// ============================================================================

void HumanActor::register_actor_class(const World& parent, const rapidxml::xml_node<char>* xml_node)
{
	if (!xml_node)
	{
		throw runtime_error("[HumanActor::register_actor_class] XML node is nullptr");
	}
	if (0 != strcmp(xml_node->name(), "actor:class"))
	{
		throw runtime_error(mrpt::format(
			"[HumanActor::register_actor_class] XML element is '%s' "
			"('actor:class' expected)",
			xml_node->name()));
	}

	actor_classes_registry.add(xml_to_str_solving_includes(parent, xml_node));
}

// ============================================================================
// Factory
// ============================================================================

HumanActor::Ptr HumanActor::factory(World* parent, const rapidxml::xml_node<char>* root)
{
	using namespace rapidxml;

	if (!root)
	{
		throw runtime_error("[HumanActor::factory] XML node is nullptr");
	}
	if (0 != strcmp(root->name(), "actor"))
	{
		throw runtime_error(mrpt::format(
			"[HumanActor::factory] XML root element is '%s' "
			"('actor' expected)",
			root->name()));
	}

	// ----- Combine class definition + instance definition -----
	JointXMLnode<> nodes;
	const rapidxml::xml_node<char>* classRoot = nullptr;

	// Always add instance root first (JointXMLnode searches first-added
	// nodes first, but we control parse order explicitly below).
	nodes.add(root);

	if (const xml_attribute<>* actorClass = root->first_attribute("class"); actorClass)
	{
		const string sClassName = actorClass->value();
		classRoot = actor_classes_registry.get(sClassName);
		if (!classRoot)
		{
			THROW_EXCEPTION_FMT(
				"[HumanActor::factory] Actor class '%s' undefined", sClassName.c_str());
		}
		nodes.add(classRoot);
	}

	// ----- Create actor -----
	HumanActor::Ptr actor = std::make_shared<HumanActor>(parent);

	// Parse name attribute
	if (const xml_attribute<>* attribName = root->first_attribute("name");
		attribName && attribName->value())
	{
		actor->name_ = attribName->value();
	}
	else
	{
		static int cnt = 0;
		actor->name_ = mrpt::format("actor%03i", ++cnt);
	}

	// ----- Parse configuration -----
	// Class defaults FIRST, so that instance values override them:
	if (classRoot)
	{
		actor->parseConfig(classRoot);
	}
	// Instance-specific values SECOND:
	actor->parseConfig(root);

	// Common Simulable properties (init_pose, animation keyframes, etc.)
	actor->parseSimulable(nodes);

	// 3D visual model
	actor->parseVisual(nodes);

	// If the model supports skeletal animation, upgrade to a per-instance
	// CAnimatedAssimpModel (shared models from the cache don't have
	// per-instance animation state).
	actor->upgradeToAnimatedModel();

	return actor;
}

// ============================================================================
// Configuration Parsing
// ============================================================================

void HumanActor::parseConfig(const rapidxml::xml_node<char>* root)
{
	// Parse scalar parameters from the TParameterDefinitions table.
	parse_xmlnode_children_as_param(
		*root, params_, world_->user_defined_variables(), "[HumanActor::parseConfig]");

	// Parse <path> if present.
	if (const rapidxml::xml_node<char>* pathNode = root->first_node("path"); pathNode)
	{
		parsePath(pathNode);
	}

	// Capture model_uri for later animated-model upgrade.
	for (auto n = root->first_node("visual"); n; n = n->next_sibling("visual"))
	{
		std::string uri;
		TParameterDefinitions visPar;
		visPar["model_uri"] = TParamEntry("%s", &uri);
		parse_xmlnode_children_as_param(*n, visPar, world_->user_defined_variables());
		if (!uri.empty())
		{
			modelFilePath_ = world_->xmlPathToActualPath(uri);
		}
	}
}

// ============================================================================
// Path Parsing
// ============================================================================

void HumanActor::parsePath(const rapidxml::xml_node<char>* pathNode)
{
	using namespace rapidxml;

	path_.clear();

	// Parse loop attribute.
	if (const xml_attribute<>* loopAttr = pathNode->first_attribute("loop"); loopAttr)
	{
		pathLoop_ = (strcmp(loopAttr->value(), "true") == 0 || strcmp(loopAttr->value(), "1") == 0);
	}

	// Parse child <waypoint> nodes.
	for (auto n = pathNode->first_node(); n; n = n->next_sibling())
	{
		if (strcmp(n->name(), "waypoint") != 0)
		{
			continue;
		}

		Waypoint wp;

		// Parse position: "x y yaw" or "x y z yaw pitch roll"
		const std::string value = mvsim::parse(n->value(), world_->user_defined_variables());

		double x = 0;
		double y = 0;
		double z = 0;
		double yaw = 0;
		double pitch = 0;
		double roll = 0;

		const int parsed =
			sscanf(value.c_str(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &yaw, &pitch, &roll);

		if (parsed == 3)
		{
			// "x y yaw_deg" format — third value is yaw.
			yaw = z;
			z = 0;
			wp.pose = mrpt::math::TPose3D(x, y, 0, yaw * M_PI / 180.0, 0, 0);
		}
		else if (parsed == 6)
		{
			wp.pose = mrpt::math::TPose3D(
				x, y, z, yaw * M_PI / 180.0, pitch * M_PI / 180.0, roll * M_PI / 180.0);
		}
		else
		{
			THROW_EXCEPTION_FMT("Error parsing <waypoint>: '%s'", value.c_str());
		}

		// Optional attributes.
		if (const xml_attribute<>* pauseAttr = n->first_attribute("pause"); pauseAttr)
		{
			wp.pauseDuration = atof(pauseAttr->value());
		}

		if (const xml_attribute<>* animAttr = n->first_attribute("animation"); animAttr)
		{
			wp.animationHint = animAttr->value();
		}

		path_.push_back(wp);
	}

	// Initialise segment tracking.
	if (!path_.empty())
	{
		currentWaypointIdx_ = 0;
		pathStartPose_ = getPose();
		pathTargetPose_ = path_[0].pose;

		const auto delta = pathTargetPose_.translation() - pathStartPose_.translation();
		pathSegmentLength_ = delta.norm();
		pathSegmentProgress_ = 0.0;
	}
}

// ============================================================================
// Path Control — public API
// ============================================================================

void HumanActor::setPath(const std::vector<Waypoint>& waypoints, bool loop)
{
	path_ = waypoints;
	pathLoop_ = loop;
	currentWaypointIdx_ = 0;
	waypointPauseTimer_ = 0.0;
	isAtWaypoint_ = false;

	initializePathFromCurrentPose();
}

void HumanActor::initializePathFromCurrentPose()
{
	if (path_.empty())
	{
		return;
	}
	pathStartPose_ = getPose();
	pathTargetPose_ = path_[0].pose;

	const auto delta = pathTargetPose_.translation() - pathStartPose_.translation();
	pathSegmentLength_ = delta.norm();
	pathSegmentProgress_ = 0.0;
}

void HumanActor::clearPath()
{
	path_.clear();
	currentWaypointIdx_ = 0;
	animState_ = AnimationState::Idle;
}

void HumanActor::setAnimation(const std::string& animationName)
{
	manualAnimationOverride_ = true;
	manualAnimationName_ = animationName;
}

void HumanActor::setAutomaticAnimation()
{
	manualAnimationOverride_ = false;
	manualAnimationName_.clear();
}

// ============================================================================
// Simulation Step
// ============================================================================

void HumanActor::simul_pre_timestep(const TSimulContext& context)
{
	// Let the base class handle keyframe animations if defined.
	Simulable::simul_pre_timestep(context);

	// Waypoint-based path following.
	if (!path_.empty())
	{
		updatePathFollowing(context);
	}

	// Animation state machine.
	updateAnimationState(context.dt);

	// Skeletal animation clock.
	updateSkeletalAnimation(context.dt);
}

void HumanActor::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);
}

// ============================================================================
// Path Following
// ============================================================================

void HumanActor::updatePathFollowing(const TSimulContext& context)
{
	if (path_.empty())
	{
		return;
	}

	const double dt = context.dt;

	// Handle pause at waypoint.
	if (isAtWaypoint_)
	{
		waypointPauseTimer_ -= dt;
		if (waypointPauseTimer_ <= 0.0)
		{
			isAtWaypoint_ = false;
			advanceToNextWaypoint();
		}
		currentMovementSpeed_ = 0.0;
		return;
	}

	// Compute desired speed.
	const double desiredSpeed = computeDesiredSpeed();
	currentMovementSpeed_ = desiredSpeed;

	if (pathSegmentLength_ < 1e-3)
	{
		// Already at target.
		pathSegmentProgress_ = 1.0;
	}
	else
	{
		const double distanceToTravel = desiredSpeed * dt;
		pathSegmentProgress_ += distanceToTravel / pathSegmentLength_;
	}

	// Check if we reached the waypoint.
	if (pathSegmentProgress_ >= 1.0)
	{
		pathSegmentProgress_ = 1.0;

		const Waypoint& wp = path_[currentWaypointIdx_];

		if (wp.pauseDuration > 0.0)
		{
			isAtWaypoint_ = true;
			waypointPauseTimer_ = wp.pauseDuration;
			currentMovementSpeed_ = 0.0;
		}
		else
		{
			advanceToNextWaypoint();
		}
	}

	// Interpolate pose along segment.
	const auto newPose = interpolatePathPose(pathSegmentProgress_);
	setPose(newPose);
}

void HumanActor::advanceToNextWaypoint()
{
	if (path_.empty())
	{
		return;
	}

	currentWaypointIdx_++;

	if (currentWaypointIdx_ >= path_.size())
	{
		if (pathLoop_)
		{
			currentWaypointIdx_ = 0;
		}
		else
		{
			// Path finished — stay at last waypoint.
			currentWaypointIdx_ = path_.size() - 1;
			currentMovementSpeed_ = 0.0;
			animState_ = AnimationState::Idle;
			return;
		}
	}

	// Set up new segment.
	pathStartPose_ = getPose();
	pathTargetPose_ = path_[currentWaypointIdx_].pose;

	const auto delta = pathTargetPose_.translation() - pathStartPose_.translation();
	pathSegmentLength_ = delta.norm();
	pathSegmentProgress_ = 0.0;
}

double HumanActor::computeDesiredSpeed() const
{
	if (path_.empty())
	{
		return 0.0;
	}

	const Waypoint& wp = path_[currentWaypointIdx_];

	if (wp.animationHint == "run")
	{
		return runningSpeed_;
	}
	if (wp.animationHint == "idle")
	{
		return 0.0;
	}

	// Default: walking speed.
	return walkingSpeed_;
}

mrpt::math::TPose3D HumanActor::interpolatePathPose(double progress) const
{
	progress = std::clamp(progress, 0.0, 1.0);

	// Linear position interpolation.
	const auto startPos = pathStartPose_.translation();
	const auto endPos = pathTargetPose_.translation();
	const auto pos = startPos + (endPos - startPos) * progress;

	// Face the direction of movement by default.
	double yaw = pathTargetPose_.yaw;

	if (pathSegmentLength_ > 0.1)
	{
		const auto delta = endPos - startPos;
		yaw = std::atan2(delta.y, delta.x);
	}

	// Smooth yaw change at the beginning.
	if (progress < 0.1)
	{
		const double t = progress / 0.1;
		const double startYaw = pathStartPose_.yaw;
		const double endYaw = yaw;

		// Shortest-path angle interpolation.
		const double deltaYaw = mrpt::math::wrapToPi(endYaw - startYaw);
		yaw = startYaw + deltaYaw * t;
	}

	// Smooth yaw towards target orientation near the end.
	if (progress > 0.8)
	{
		const double t = (progress - 0.8) / 0.2;
		const double startYaw = std::atan2(endPos.y - startPos.y, endPos.x - startPos.x);
		const double endYaw = pathTargetPose_.yaw;

		// Shortest-path angle interpolation.
		const double deltaYaw = mrpt::math::wrapToPi(endYaw - startYaw);
		yaw = startYaw + deltaYaw * t;
	}

	return mrpt::math::TPose3D(pos.x, pos.y, pos.z, yaw, 0, 0);
}

// ============================================================================
// Animation State Machine
// ============================================================================

void HumanActor::updateAnimationState(double /*dt*/)
{
	if (manualAnimationOverride_)
	{
		return;
	}

	AnimationState newState = AnimationState::Idle;

	if (currentMovementSpeed_ > runningSpeed_ * 0.8)
	{
		newState = AnimationState::Running;
	}
	else if (currentMovementSpeed_ > 0.1)
	{
		newState = AnimationState::Walking;
	}

	animState_ = newState;
}

// ============================================================================
// Skeletal Animation
// ============================================================================

void HumanActor::updateSkeletalAnimation(double dt)
{
	// Determine which animation clip to play.
	std::string animName;

	if (manualAnimationOverride_)
	{
		animName = manualAnimationName_;
	}
	else
	{
		switch (animState_)
		{
			case AnimationState::Walking:
				animName = animNameWalk_;
				break;
			case AnimationState::Running:
				animName = animNameRun_;
				break;
			case AnimationState::Idle:
			default:
				animName = animNameIdle_;
				break;
		}
	}

	// Scale animation playback speed to match movement speed.
	double animSpeedScale = 1.0;

	if (animState_ == AnimationState::Walking && walkingSpeed_ > 0.01)
	{
		animSpeedScale = currentMovementSpeed_ / walkingSpeed_;
	}
	else if (animState_ == AnimationState::Running && runningSpeed_ > 0.01)
	{
		animSpeedScale = currentMovementSpeed_ / runningSpeed_;
	}

	currentAnimTime_ += dt * animSpeedScale;

#if MRPT_VERSION >= MIN_MRPT_VERSION_ANIMATED_ASSIMP
	// Drive the CAnimatedAssimpModel if we have one.
	// Lazy discovery: walk the visual hierarchy on first use.
	if (glCustomVisual_ && !glModel_)
	{
		// The hierarchy built by VisualObject::addCustomVisualization is:
		//   glCustomVisual_ -> CSetOfObjects("group") -> CAssimpModel
		// We look for a CAnimatedAssimpModel at any depth.
		for (auto& obj : *glCustomVisual_)
		{
			// Direct child?
			glModel_ = std::dynamic_pointer_cast<mrpt::opengl::CAnimatedAssimpModel>(obj);
			if (glModel_)
			{
				break;
			}
			// Inside a CSetOfObjects wrapper?
			if (auto grp = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(obj); grp)
			{
				for (auto& inner : *grp)
				{
					glModel_ = std::dynamic_pointer_cast<mrpt::opengl::CAnimatedAssimpModel>(inner);
					if (glModel_)
					{
						break;
					}
				}
			}
			if (glModel_)
			{
				break;
			}
		}
	}

	if (glModel_)
	{
		glModel_->setActiveAnimation(animName);
		glModel_->setAnimationTime(currentAnimTime_);
	}
#endif
}

// ============================================================================
// Rendering
// ============================================================================

void HumanActor::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly)
{
	// On first call, if no <visual> was defined, create a simple
	// placeholder capsule so the actor is visible.
	if (!glInitialized_ && viz.has_value() && physical.has_value())
	{
		if (!glCustomVisual_)
		{
			auto cylinder = mrpt::opengl::CCylinder::Create(
				static_cast<float>(collisionRadius_), static_cast<float>(collisionRadius_),
				static_cast<float>(collisionHeight_));
			cylinder->setColor_u8(mrpt::img::TColor(100, 150, 200, 200));
			cylinder->setLocation(0, 0, 0);

			auto head = mrpt::opengl::CSphere::Create(static_cast<float>(collisionRadius_ * 0.8));
			head->setColor_u8(mrpt::img::TColor(220, 180, 150));
			head->setLocation(0, 0, static_cast<float>(collisionHeight_));

			auto placeholder = mrpt::opengl::CSetOfObjects::Create();
			placeholder->insert(cylinder);
			placeholder->insert(head);

			addCustomVisualization(placeholder);
		}

		glInitialized_ = true;
	}

	// Everything else (scene insertion, pose update) is handled by the
	// base class VisualObject::guiUpdate(), which calls us here.
	// We deliberately do NOT duplicate pose-update logic.
	(void)viz;
	(void)physical;
	(void)childrenOnly;
}

// ============================================================================
// Animated Model Upgrade
// ============================================================================

void HumanActor::upgradeToAnimatedModel()
{
	// Update initial pose for animated paths:
	initializePathFromCurrentPose();

	if (!glCustomVisual_)
	{
		return;
	}
	if (modelFilePath_.empty())
	{
		return;
	}

	// Walk the visual hierarchy to find the CAssimpModel and its wrapper.
	mrpt::opengl::CAssimpModel::Ptr foundModel;
	mrpt::opengl::CSetOfObjects::Ptr containingGroup;

	for (auto& obj : *glCustomVisual_)
	{
		auto grp = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(obj);
		if (!grp)
		{
			continue;
		}
		for (auto& inner : *grp)
		{
			auto model = std::dynamic_pointer_cast<mrpt::opengl::CAssimpModel>(inner);
			if (model)
			{
				foundModel = model;
				containingGroup = grp;
				break;
			}
		}
		if (foundModel)
		{
			break;
		}
	}

	if (!foundModel)
	{
		return;	 // no 3D model in hierarchy — placeholder visual
	}

	// Create a dedicated CAnimatedAssimpModel and load from file.
#if MRPT_VERSION >= MIN_MRPT_VERSION_ANIMATED_ASSIMP
	auto animModel = mrpt::opengl::CAnimatedAssimpModel::Create();
#else
	auto animModel = mrpt::opengl::CAssimpModel::Create();
#endif
	const int loadFlags = mrpt::opengl::CAssimpModel::LoadFlags::RealTimeMaxQuality |
						  mrpt::opengl::CAssimpModel::LoadFlags::FlipUVs;

	try
	{
		animModel->loadScene(modelFilePath_, loadFlags);
	}
	catch (const std::exception& e)
	{
		std::cerr << "[HumanActor] Warning: could not load animated model '" << modelFilePath_
				  << "': " << e.what() << "\n";
		return;
	}
#if MRPT_VERSION >= MIN_MRPT_VERSION_ANIMATED_ASSIMP
	std::cout << "[HumanActor] Animations loaded: " << animModel->getAnimationCount() << ": ";
	for (size_t animIdx = 0; animIdx < animModel->getAnimationCount(); animIdx++)
	{
		std::cout << "'" << animModel->getAnimationName(animIdx) << "', ";
	}
	std::cout << "\n";

	// If no animations were found, keep the original (lighter).
	if (animModel->getAnimationCount() == 0)
	{
		return;
	}

	// Carry over visual properties from the original model.
	animModel->setColor_u8(foundModel->getColor_u8());
#endif

	// Preserve the wrapper group's pose and scale, then swap the model.
	const auto savedPose = containingGroup->getPose();
	const float savedScale = containingGroup->getScaleX();

	containingGroup->clear();
	containingGroup->insert(animModel);
	containingGroup->setPose(savedPose);
	containingGroup->setScale(savedScale);

	// Keep a direct handle for the animation driver.
	glModel_ = animModel;
}
