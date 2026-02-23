/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/version.h>
#include <mvsim/ClassFactory.h>
#include <mvsim/Simulable.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/VisualObject.h>

#define MIN_MRPT_VERSION_ANIMATED_ASSIMP 0x020f08

#if MRPT_VERSION >= MIN_MRPT_VERSION_ANIMATED_ASSIMP
#include <mrpt/opengl/CAnimatedAssimpModel.h>
#else
#include <mrpt/opengl/CAssimpModel.h>
#endif

#include <memory>
#include <string>
#include <vector>

namespace mvsim
{
class World;

/** A walking human actor with skeletal animation support.
 *
 * Actors follow predefined paths (waypoints) and play appropriate
 * animations (idle, walk, run) based on movement speed.
 * They are purely kinematic: they do not respond to external forces.
 *
 * XML usage example:
 * \code{.xml}
 * <actor:class name="pedestrian">
 *     <walking_speed>1.4</walking_speed>
 *     <animation_walk>walk</animation_walk>
 *     <animation_idle>idle</animation_idle>
 *     <visual>
 *         <model_uri>https://example.com/human.glb</model_uri>
 *     </visual>
 * </actor:class>
 *
 * <actor name="person1" class="pedestrian">
 *     <init_pose>5 10 0</init_pose>
 *     <path type="waypoints" loop="true">
 *         <waypoint>0 0 0</waypoint>
 *         <waypoint>10 0 0</waypoint>
 *         <waypoint>10 10 90</waypoint>
 *     </path>
 * </actor>
 * \endcode
 *
 * \ingroup mvsim_simulator_module
 */
class HumanActor : public VisualObject, public Simulable
{
   public:
	using Ptr = std::shared_ptr<HumanActor>;

	HumanActor(World* parent);
	virtual ~HumanActor() = default;

	// Delete copy/move
	HumanActor(const HumanActor&) = delete;
	HumanActor& operator=(const HumanActor&) = delete;
	HumanActor(HumanActor&&) = delete;
	HumanActor& operator=(HumanActor&&) = delete;

	// ==================== Factory ====================

	/** Class factory: Creates an actor from XML "<actor>...</actor>" */
	static Ptr factory(World* parent, const rapidxml::xml_node<char>* xml_node);

	/** Register a new actor class from
	 *  "<actor:class name='xxx'>...</actor:class>" */
	static void register_actor_class(const World& parent, const rapidxml::xml_node<char>* xml_node);

	// ==================== Simulable interface ====================

	void simul_pre_timestep(const TSimulContext& context) override;
	void simul_post_timestep(const TSimulContext& context) override;

	void apply_force(
		const mrpt::math::TVector2D& force,
		const mrpt::math::TPoint2D& applyPoint = mrpt::math::TPoint2D(0, 0)) override
	{
		// Actors are kinematic, they do not respond to external forces.
		(void)force;
		(void)applyPoint;
	}

	// ==================== Path & Animation Control ====================

	/** Waypoint for path following */
	struct Waypoint
	{
		mrpt::math::TPose3D pose;
		double pauseDuration = 0.0;	 //!< seconds to pause at this waypoint
		std::string animationHint;	//!< "walk", "run", "idle", or empty
	};

	/** Animation state machine states */
	enum class AnimationState
	{
		Idle,
		Walking,
		Running,
		Turning,
		Custom
	};

	/** Set waypoint-based path. */
	void setPath(const std::vector<Waypoint>& waypoints, bool loop = true);

	/** Clear current path. */
	void clearPath();

	/** Get current animation state. */
	AnimationState getAnimationState() const { return animState_; }

	/** Manually set animation (overrides automatic selection). */
	void setAnimation(const std::string& animationName);

	/** Return to automatic animation selection based on movement. */
	void setAutomaticAnimation();

	// ==================== Configuration ====================

	double getWalkingSpeed() const { return walkingSpeed_; }
	void setWalkingSpeed(double speed) { walkingSpeed_ = speed; }

	double getRunningSpeed() const { return runningSpeed_; }
	void setRunningSpeed(double speed) { runningSpeed_ = speed; }

	double getHeight() const { return height_; }
	void setHeight(double h) { height_ = h; }

	// ==================== Collision ====================

	double getCollisionRadius() const { return collisionRadius_; }
	double getCollisionHeight() const { return collisionHeight_; }

   protected:
	void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	// ==================== Configuration Parameters ====================

	double walkingSpeed_ = 1.4;	 //!< m/s (typical human walking speed)
	double runningSpeed_ = 3.5;	 //!< m/s
	double height_ = 1.75;	//!< meters

	// Collision shape (capsule approximation)
	double collisionRadius_ = 0.3;
	double collisionHeight_ = 1.7;

	// Animation names (mapped to animations in the 3D model)
	std::string animNameIdle_ = "idle";
	std::string animNameWalk_ = "walk";
	std::string animNameRun_ = "run";

	/** XML-parseable parameter table, follows the Block pattern.
	 *  Addresses point to the member variables above. */
	const TParameterDefinitions params_ = {
		{"walking_speed", {"%lf", &walkingSpeed_}},
		{"running_speed", {"%lf", &runningSpeed_}},
		{"height", {"%lf", &height_}},
		{"collision_radius", {"%lf", &collisionRadius_}},
		{"collision_height", {"%lf", &collisionHeight_}},
		{"animation_idle", {"%s", &animNameIdle_}},
		{"animation_walk", {"%s", &animNameWalk_}},
		{"animation_run", {"%s", &animNameRun_}},
	};

   private:
	// ==================== Path Following ====================

	std::vector<Waypoint> path_;
	bool pathLoop_ = true;
	size_t currentWaypointIdx_ = 0;
	double waypointPauseTimer_ = 0.0;
	bool isAtWaypoint_ = false;

	// Smooth interpolation between waypoints
	mrpt::math::TPose3D pathStartPose_;
	mrpt::math::TPose3D pathTargetPose_;
	double pathSegmentProgress_ = 0.0;	//!< [0, 1]
	double pathSegmentLength_ = 0.0;

	void initializePathFromCurrentPose();

	// ==================== Animation ====================

	AnimationState animState_ = AnimationState::Idle;
	bool manualAnimationOverride_ = false;
	std::string manualAnimationName_;

	double currentAnimTime_ = 0.0;
	double currentMovementSpeed_ = 0.0;	 //!< actual speed for anim sync

	// ==================== Rendering ====================
#if MRPT_VERSION >= MIN_MRPT_VERSION_ANIMATED_ASSIMP
	mrpt::opengl::CAnimatedAssimpModel::Ptr glModel_;
#else
	mrpt::opengl::CAssimpModel::Ptr glModel_;
#endif
	bool glInitialized_ = false;

	/** Resolved filesystem path to the 3D model (used by
	 *  upgradeToAnimatedModel to reload the model per-instance). */
	std::string modelFilePath_;

	// ==================== Internal Methods ====================

	/** Parse actor scalar params + path from XML. */
	void parseConfig(const rapidxml::xml_node<char>* root);

	/** Parse <path>...</path> child node. */
	void parsePath(const rapidxml::xml_node<char>* pathNode);

	/** Update path following logic each timestep. */
	void updatePathFollowing(const TSimulContext& context);

	/** Update animation state machine. */
	void updateAnimationState(double dt);

	/** Advance skeletal animation clock & drive CAnimatedAssimpModel. */
	void updateSkeletalAnimation(double dt);

	/** Compute desired speed based on current waypoint hint. */
	double computeDesiredSpeed() const;

	/** Set up next waypoint segment. */
	void advanceToNextWaypoint();

	/** Linearly interpolate pose along current segment. */
	mrpt::math::TPose3D interpolatePathPose(double progress) const;

	/** If the loaded visual is an animated GLB/FBX, replace the shared
	 *  CAssimpModel with a per-instance CAnimatedAssimpModel so that
	 *  each actor has its own animation state. */
	void upgradeToAnimatedModel();
};

}  // namespace mvsim