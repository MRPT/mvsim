/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <Box2D/Dynamics/b2Body.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mvsim/basic_types.h>

namespace mvsim
{
class Simulable
{
   public:
	/** Process right before the integration of dynamic equations for each
	 * timestep: set action forces from motors, update friction models, etc. */
	virtual void simul_pre_timestep(const TSimulContext& context);

	/** Override to do any required process right after the integration of
	 * dynamic equations for each timestep.
	 * IMPORTANT: Reimplementations MUST also call this base method,
	 * since it is in charge of important tasks (e.g. update m_q, m_dq)
	 */
	virtual void simul_post_timestep(const TSimulContext& context);

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
	const mrpt::math::TPose3D& getPose() const { return m_q; }

	/** Manually override vehicle pose (Use with caution!) (purposely set a
	 * "const")*/
	void setPose(const mrpt::math::TPose3D& p) const
	{
		const_cast<mrpt::math::TPose3D&>(m_q) = p;
	}

	/// Alternative to getPose()
	mrpt::poses::CPose2D getCPose2D() const;

	/** Last time-step velocity (of the ref. point, in global coords)
	 * (ground-truth) */
	const mrpt::math::TTwist2D& getVelocity() const { return m_dq; }

   protected:
	/** Derived classes must store here the body of the physical element (e.g.
	 * chassis).
	 * This is used by \a simul_post_timestep() to extract the block
	 * dynamical coords (q,\dot{q}) after each simulation step.
	 */
	b2Body* m_b2d_body = nullptr;

	/** Last time-step pose (of the ref. point, in global coords) */
	mrpt::math::TPose3D m_q = mrpt::math::TPose3D::Identity();

	/** Last time-step velocity (of the ref. point, in global coords) */
	mrpt::math::TTwist2D m_dq{0, 0, 0};
};
}  // namespace mvsim
