/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/Simulable.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/World.h>

#include "xml_utils.h"

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include "TimeStampedPose.pb.h"

#endif

using namespace mvsim;

void Simulable::simul_pre_timestep(	 //
	[[maybe_unused]] const TSimulContext& context)
{
	if (!m_b2d_body) return;

	// Pos:
	m_b2d_body->SetTransform(b2Vec2(m_q.x, m_q.y), m_q.yaw);

	// Vel:
	m_b2d_body->SetLinearVelocity(b2Vec2(m_dq.vx, m_dq.vy));
	m_b2d_body->SetAngularVelocity(m_dq.omega);
}

void Simulable::simul_post_timestep(  //
	[[maybe_unused]] const TSimulContext& context)
{
	if (!m_b2d_body) return;

	poses_mutex_lock();

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

	// Instantaneous collision flag:
	m_isInCollision = false;
	if (b2ContactEdge* cl = m_b2d_body->GetContactList();
		cl != nullptr && cl->contact != nullptr && cl->contact->IsTouching())
	{
		// We may store with which other bodies it's in collision...
		m_isInCollision = true;
	}
	// Reseteable collision flag:
	m_hadCollisionFlag = m_hadCollisionFlag || m_isInCollision;

	poses_mutex_unlock();

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
	std::shared_lock lck(m_q_mtx);

	mrpt::math::TTwist2D local_vel = m_dq;
	local_vel.rotate(-m_q.yaw);	 // "-" means inverse pose
	return local_vel;
}

mrpt::poses::CPose2D Simulable::getCPose2D() const
{
	std::shared_lock lck(m_q_mtx);

	return {m_q.x, m_q.y, m_q.yaw};
}

bool Simulable::parseSimulable(const rapidxml::xml_node<char>* node)
{
	MRPT_START

	if (node == nullptr) return false;

	TParameterDefinitions params;
	params["publish_pose_topic"] = TParamEntry("%s", &publishPoseTopic_);
	params["publish_pose_period"] = TParamEntry("%lf", &publishPosePeriod_);

	const std::map<std::string, std::string> varValues = {{"NAME", m_name}};

	// Parse XML params:
	parse_xmlnode_children_as_param(*node, params, varValues);

	return true;
	MRPT_END
}

void Simulable::internalHandlePublish(const TSimulContext& context)
{
	std::shared_lock lck(m_q_mtx);

	MRPT_START
	if (publishPoseTopic_.empty()) return;

	auto& client = context.world->commsClient();

	const double tNow = mrpt::Clock::toDouble(mrpt::Clock::now());
	if (tNow < publishPoseLastTime_ + publishPosePeriod_) return;

	publishPoseLastTime_ = tNow;

	mvsim_msgs::TimeStampedPose msg;
	msg.set_unixtimestamp(tNow);
	msg.set_objectid(m_name);

	auto pose = msg.mutable_pose();
	pose->set_x(m_q.x);
	pose->set_y(m_q.y);
	pose->set_z(.0);
	pose->set_yaw(m_q.yaw);
	pose->set_pitch(m_q.pitch);
	pose->set_roll(m_q.roll);

	client.publishTopic(publishPoseTopic_, msg);

	MRPT_END
}

void Simulable::registerOnServer(mvsim::Client& c)
{
	MRPT_START

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	// Topic:
	if (!publishPoseTopic_.empty())
		c.advertiseTopic<mvsim_msgs::TimeStampedPose>(publishPoseTopic_);
#endif

	MRPT_END
}
