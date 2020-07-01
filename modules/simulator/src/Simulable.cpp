/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/Simulable.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/World.h>

#include <mvsim/World.h>
#include "xml_utils.h"

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include "Pose.pb.h"

#endif

using namespace mvsim;

void Simulable::simul_pre_timestep(  //
	[[maybe_unused]] const TSimulContext& context)
{ /* default: do nothing*/
}

void Simulable::simul_post_timestep(  //
	[[maybe_unused]] const TSimulContext& context)
{
	if (!m_b2d_body) return;

	// Pos:
	const b2Vec2& pos = m_b2d_body->GetPosition();
	const float32 angle = m_b2d_body->GetAngle();
	m_q.x = pos(0);
	m_q.y = pos(1);
	m_q.yaw = angle;
	// The rest (z,pitch,roll) will be always 0, unless other
	// world-element modifies them! (e.g. elevation map)

	// Vel:
	const b2Vec2& vel = m_b2d_body->GetLinearVelocity();
	const float32 w = m_b2d_body->GetAngularVelocity();
	m_dq.vx = vel(0);
	m_dq.vy = vel(1);
	m_dq.omega = w;

	// Optional publish to topics:
	internalHandlePublish(context);
}

void Simulable::apply_force(
	[[maybe_unused]] const mrpt::math::TVector2D& force,
	[[maybe_unused]] const mrpt::math::TPoint2D& applyPoint)
{ /* default: do nothing*/
}

mrpt::math::TTwist2D Simulable::getVelocityLocal() const
{
	mrpt::math::TTwist2D local_vel = m_dq;
	local_vel.rotate(-m_q.yaw);  // "-" means inverse pose
	return local_vel;
}

mrpt::poses::CPose2D Simulable::getCPose2D() const
{
	return {m_q.x, m_q.y, m_q.yaw};
}

bool Simulable::parseSimulable(const rapidxml::xml_node<char>* node)
{
	MRPT_START

	if (node == nullptr) return false;

	TParameterDefinitions params;
	params["publish_pose_topic"] = TParamEntry("%s", &publishPoseTopic_);
	params["publish_pose_period"] = TParamEntry("%lf", &publishPosePeriod_);

	// Parse XML params:
	parse_xmlnode_children_as_param(*node, params);

	return true;
	MRPT_END
}

void Simulable::internalHandlePublish(const TSimulContext& context)
{
	MRPT_START
	if (publishPoseTopic_.empty()) return;

	auto& client = context.world->commsClient();

	if (!publishAdvertised_)
	{
		publishAdvertised_ = true;
		client.advertiseTopic<mvsim_msgs::Pose>(publishPoseTopic_);
	}

	const double tNow = mrpt::Clock::toDouble(mrpt::Clock::now());
	if (tNow < publishPoseLastTime_ + publishPosePeriod_) return;

	publishPoseLastTime_ = tNow;

	MRPT_TODO("Change type to timestamped pose");
	mvsim_msgs::Pose msg;
	msg.set_x(m_q.x);
	msg.set_y(m_q.y);
	msg.set_z(.0);
	msg.set_yaw(m_q.yaw);
	msg.set_pitch(m_q.pitch);
	msg.set_roll(m_q.roll);
	client.publishTopic(publishPoseTopic_, msg);

	MRPT_END
}