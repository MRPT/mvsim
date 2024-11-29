/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_body.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mvsim/basic_types.h>

#include <shared_mutex>

namespace mvsim
{
class Client;
class World;
class VisualObject;

/** \ingroup mvsim_simulator_module */
struct ParseSimulableParams
{
	ParseSimulableParams() = default;

	bool init_pose_mandatory = true;
};

/** The basic virtual base class for all objects that can run in the simulated mvsim::World
 *
 * \ingroup virtual_interfaces_module
 */
class Simulable
{
   public:
	using Ptr = std::shared_ptr<Simulable>;

	Simulable(World* parent) : simulable_parent_(parent) {}

	/** Process right before the integration of dynamic equations for each
	 * timestep: set action forces from motors, update friction models, etc. */
	virtual void simul_pre_timestep(const TSimulContext& context);

	/** Override to do any required process right after the integration of
	 * dynamic equations for each timestep.
	 * IMPORTANT: Reimplementations MUST also call this base method,
	 * since it is in charge of important tasks (e.g. update q_, dq_)
	 */
	virtual void simul_post_timestep(const TSimulContext& context);

	/** Override to register external forces exerted by other WorldElements.
	 * Force is (fx,fy) in global coordinates. Application point is
	 * (local_ptx,local_pty) in the body local frame */
	virtual void apply_force(
		const mrpt::math::TVector2D& force,
		const mrpt::math::TPoint2D& applyPoint = mrpt::math::TPoint2D(0, 0));

	virtual VisualObject* meAsVisualObject() { return nullptr; }

	/** Last time-step velocity (of the ref. point, in local coords) */
	mrpt::math::TTwist2D getVelocityLocal() const;

	/** Last time-step pose (of the ref. point, in global coords) (ground-truth)
	 */
	mrpt::math::TPose3D getPose() const;

	/** Like getPose(), but gets the relative pose with respect to the parent
	 * object, or just exactly like getPose() (global pose) if this is a
	 * top-level entity.
	 */
	virtual mrpt::math::TPose3D getRelativePose() const { return getPose(); }

	/// No thread-safe version. Used internally only.
	mrpt::math::TPose3D getPoseNoLock() const;

	mrpt::math::TTwist2D getTwist() const;

	/** Last time-step acceleration of the ref. point (global coords).
	 * Note this is the "coordinate acceleration" vector, not the proper
	 * acceleration. It is simply estimated as a finite difference of dq_.
	 */
	mrpt::math::TVector3D getLinearAcceleration() const;

	/** Manually override vehicle pose (Use with caution!) (purposely set a
	 * "const")*/
	void setPose(const mrpt::math::TPose3D& p, bool notifyChange = true) const;

	/** Changes the relative pose of this object with respect to its parent, or
	 * the global frame if its a top-level entity. */
	virtual void setRelativePose(const mrpt::math::TPose3D& p) { setPose(p); }

	void setTwist(const mrpt::math::TTwist2D& dq) const;

	/// Alternative to getPose()
	mrpt::poses::CPose2D getCPose2D() const;

	/// Alternative to getPose()
	mrpt::poses::CPose3D getCPose3D() const;

	/** User-supplied name of the vehicle (e.g. "r1", "veh1") */
	const std::string& getName() const { return name_; }

	/** Changes object name (e.g. "r1", "veh1") */
	void setName(const std::string& s) { name_ = s; }

	/** Whether is is in collision right now. \sa  */
	bool isInCollision() const;

	/** Whether a collision occurred since the last time this flag was manually
	 * reset.
	 * \sa isInCollision(), resetCollisionFlag()  */
	bool hadCollision() const;

	/** Resets the condition reported by hadCollision() to false */
	void resetCollisionFlag();

	virtual void registerOnServer(mvsim::Client& c);

	const b2Body* b2d_body() const { return b2dBody_; }
	b2Body* b2d_body() { return b2dBody_; }

	World* getSimulableWorldObject() { return simulable_parent_; }
	const World* getSimulableWorldObject() const { return simulable_parent_; }

	virtual void freeOpenGLResources() {}

	/** If the given world-frame 2D coordinates are within the limits of this entity,
	 *  this method returns the ground height or elevation or "z" coordinate of the object
	 *  for the queried (x,y). If the coordinates do not affect this object, it will return nullopt.
	 */
	virtual std::optional<float> getElevationAt(
		[[maybe_unused]] const mrpt::math::TPoint2D& worldXY) const
	{
		return std::nullopt;
	}

   protected:
	/** User-supplied name of the vehicle (e.g. "r1", "veh1") */
	std::string name_;

	/** Derived classes must store here the body of the physical element (e.g.
	 * chassis).
	 * This is used by \a simul_post_timestep() to extract the block
	 * dynamical coords (q,\dot{q}) after each simulation step.
	 */
	b2Body* b2dBody_ = nullptr;

	bool parseSimulable(const JointXMLnode<>& node, const ParseSimulableParams& p = {});

	void internalHandlePublish(const TSimulContext& context);

	/** Will be called after the global pose of the object has changed due to a
	 * direct call to setPose() */
	virtual void notifySimulableSetPose([[maybe_unused]] const mrpt::math::TPose3D& newPose)
	{
		// Default: do nothing
	}

   private:
	World* simulable_parent_ = nullptr;

	/** protects q_, dq_, ddq_lin_ */
	mutable std::shared_mutex q_mtx_;

	/** Last time-step pose (of the ref. point, in global coords) */
	mrpt::math::TPose3D q_ = mrpt::math::TPose3D::Identity();

	/** Last time-step velocity (of the ref. point, in global coords) */
	mrpt::math::TTwist2D dq_{0, 0, 0};

	/// See notes of getLinearAcceleration()
	mrpt::math::TVector3D ddq_lin_{0, 0, 0};

	mrpt::math::TPose3D former_q_;	//!< Updated in simul_post_timestep()

	// ============ ANIMATION VARIABLES ============
	/** Initial pose, per configuration XML world file */
	mrpt::math::TPose3D initial_q_ = mrpt::math::TPose3D::Identity();

	std::optional<mrpt::poses::CPose3DInterpolator> anim_keyframes_path_;

	// ============ END OF ANIMATION VARIABLES ============

	/** Whether is is in collision right now */
	bool isInCollision_ = false;

	/** Whether a collision occurred since the last time this flag was manually
	 * reset.
	 * Multithreading: protected by q_mtx_
	 */
	bool hadCollisionFlag_ = false;

	/** If not empty, publish the pose on this topic */
	std::string publishPoseTopic_;

	std::string publishRelativePoseTopic_;
	std::vector<std::string> publishRelativePoseOfOtherObjects_;

	double publishPosePeriod_ = 100e-3;	 //! Publish period [seconds]
	double publishPoseLastTime_ = 0;
};
}  // namespace mvsim
