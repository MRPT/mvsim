/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_body.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mvsim/basic_types.h>

#include <shared_mutex>

namespace mvsim
{
class Client;
class World;

class Simulable
{
   public:
	using Ptr = std::shared_ptr<Simulable>;

	Simulable(World* parent) : m_simulable_parent(parent) {}

	/** Process right before the integration of dynamic equations for each
	 * timestep: set action forces from motors, update friction models, etc. */
	virtual void simul_pre_timestep(const TSimulContext& context);

	/** Override to do any required process right after the integration of
	 * dynamic equations for each timestep.
	 * IMPORTANT: Reimplementations MUST also call this base method,
	 * since it is in charge of important tasks (e.g. update m_q, m_dq)
	 */
	virtual void simul_post_timestep(const TSimulContext& context);

	virtual void poses_mutex_lock() = 0;
	virtual void poses_mutex_unlock() = 0;

	/** Override to register external forces exerted by other WorldElements.
	 * Force is (fx,fy) in global coordinates. Application point is
	 * (local_ptx,local_pty) in the body local frame */
	virtual void apply_force(
		const mrpt::math::TVector2D& force,
		const mrpt::math::TPoint2D& applyPoint = mrpt::math::TPoint2D(0, 0));

	/** Last time-step velocity (of the ref. point, in local coords) */
	mrpt::math::TTwist2D getVelocityLocal() const;

	/** Last time-step pose (of the ref. point, in global coords) (ground-truth)
	 */
	mrpt::math::TPose3D getPose() const
	{
		m_q_mtx.lock_shared();
		mrpt::math::TPose3D ret = m_q;
		m_q_mtx.unlock_shared();
		return ret;
	}

	mrpt::math::TTwist2D getTwist() const
	{
		m_q_mtx.lock_shared();
		mrpt::math::TTwist2D ret = m_dq;
		m_q_mtx.unlock_shared();
		return ret;
	}

	/** Manually override vehicle pose (Use with caution!) (purposely set a
	 * "const")*/
	void setPose(const mrpt::math::TPose3D& p) const;

	void setTwist(const mrpt::math::TTwist2D& dq) const
	{
		m_q_mtx.lock();
		const_cast<mrpt::math::TTwist2D&>(m_dq) = dq;
		m_q_mtx.unlock();
	}

	/// Alternative to getPose()
	mrpt::poses::CPose2D getCPose2D() const;

	/** Last time-step velocity (of the ref. point, in global coords)
	 * (ground-truth) */
	const mrpt::math::TTwist2D& getVelocity() const { return m_dq; }

	/** User-supplied name of the vehicle (e.g. "r1", "veh1") */
	const std::string& getName() const { return m_name; }

	/** Changes object name (e.g. "r1", "veh1") */
	void setName(const std::string& s) { m_name = s; }

	/** Whether is is in collision right now. \sa  */
	bool isInCollision() const { return m_isInCollision; }

	/** Whether a collision occurred since the last time this flag was manually
	 * reset.
	 * \sa isInCollision(), resetCollisionFlag()  */
	bool hadCollision() const { return m_hadCollisionFlag; }

	/** Resets the condition reported by hadCollision() to false */
	void resetCollisionFlag() { m_hadCollisionFlag = false; }

	virtual void registerOnServer(mvsim::Client& c);

	const b2Body* b2d_body() const { return m_b2d_body; }
	b2Body* b2d_body() { return m_b2d_body; }

	World* getSimulableWorldObject() { return m_simulable_parent; }
	const World* getSimulableWorldObject() const { return m_simulable_parent; }

	virtual void freeOpenGLResources() {}

   protected:
	/** User-supplied name of the vehicle (e.g. "r1", "veh1") */
	std::string m_name;

	/** Derived classes must store here the body of the physical element (e.g.
	 * chassis).
	 * This is used by \a simul_post_timestep() to extract the block
	 * dynamical coords (q,\dot{q}) after each simulation step.
	 */
	b2Body* m_b2d_body = nullptr;

	bool parseSimulable(const rapidxml::xml_node<char>* node);

	void internalHandlePublish(const TSimulContext& context);

   private:
	World* m_simulable_parent = nullptr;

	/** protects m_q, m_dq */
	mutable std::shared_mutex m_q_mtx;

	/** Last time-step pose (of the ref. point, in global coords) */
	mrpt::math::TPose3D m_q = mrpt::math::TPose3D::Identity();

	/** Last time-step velocity (of the ref. point, in global coords) */
	mrpt::math::TTwist2D m_dq{0, 0, 0};

	/** Whether is is in collision right now */
	bool m_isInCollision = false;

	/** Whether a collision occurred since the last time this flag was manually
	 * reset  */
	bool m_hadCollisionFlag = false;

	/** If not empty, publish the pose on this topic */
	std::string publishPoseTopic_;

	std::string publishRelativePoseTopic_;
	std::vector<std::string> publishRelativePoseOfOtherObjects_;

	double publishPosePeriod_ = 100e-3;	 //! Publish period [seconds]
	double publishPoseLastTime_ = 0;
};
}  // namespace mvsim
