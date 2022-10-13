/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <box2d/b2_contact.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/Simulable.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/World.h>

#include <cmath>  // fmod()

#include "JointXMLnode.h"
#include "xml_utils.h"

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <mvsim/mvsim-msgs/TimeStampedPose.pb.h>

#endif

using namespace mvsim;

void Simulable::simul_pre_timestep(	 //
	[[maybe_unused]] const TSimulContext& context)
{
	if (!m_b2d_body) return;

	// Follow animation, if enabled:
	if (m_anim_keyframes_path && !m_anim_keyframes_path->empty())
	{
		auto& poseSeq = m_anim_keyframes_path.value();

		mrpt::math::TPose3D q;
		bool interOk = false;
		const double tMax = mrpt::Clock::toDouble(poseSeq.rbegin()->first);

		poseSeq.interpolate(
			mrpt::Clock::fromDouble(std::fmod(context.simul_time, tMax)), q,
			interOk);

		if (interOk) this->setPose(m_initial_q + q);
	}

	// Pos:
	m_b2d_body->SetTransform(b2Vec2(m_q.x, m_q.y), m_q.yaw);

	// Vel:
	m_b2d_body->SetLinearVelocity(b2Vec2(m_dq.vx, m_dq.vy));
	m_b2d_body->SetAngularVelocity(m_dq.omega);
}

void Simulable::simul_post_timestep(const TSimulContext& context)
{
	if (m_b2d_body)
	{
		poses_mutex_lock();
		// m_simulable_parent->physical_objects_mtx(): already locked by caller

		// Pos:
		const b2Vec2& pos = m_b2d_body->GetPosition();
		const float angle = m_b2d_body->GetAngle();
		m_q.x = pos(0);
		m_q.y = pos(1);
		m_q.yaw = angle;
		// The rest (z,pitch,roll) will be always 0, unless other
		// world-element modifies them! (e.g. elevation map)

		// Vel:
		const b2Vec2& vel = m_b2d_body->GetLinearVelocity();
		const float w = m_b2d_body->GetAngularVelocity();
		m_dq.vx = vel(0);
		m_dq.vy = vel(1);
		m_dq.omega = w;

		// Instantaneous collision flag:
		m_isInCollision = false;
		if (b2ContactEdge* cl = m_b2d_body->GetContactList();
			cl != nullptr && cl->contact != nullptr &&
			cl->contact->IsTouching())
		{
			// We may store with which other bodies it's in collision...
			m_isInCollision = true;
		}
		// Reseteable collision flag:
		m_hadCollisionFlag = m_hadCollisionFlag || m_isInCollision;

		poses_mutex_unlock();
	}

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

bool Simulable::parseSimulable(const JointXMLnode<>& rootNode)
{
	MRPT_START

	using namespace rapidxml;
	using namespace std::string_literals;

	// -------------------------------------
	// (Mandatory) initial pose:
	// -------------------------------------
	if (const xml_node<>* nPose = rootNode.first_node("init_pose"); nPose)
	{
		mrpt::math::TPose3D p;
		if (3 != ::sscanf(nPose->value(), "%lf %lf %lf", &p.x, &p.y, &p.yaw))
			THROW_EXCEPTION_FMT(
				"Error parsing <init_pose>%s</init_pose>", nPose->value());
		p.yaw *= M_PI / 180.0;	// deg->rad

		this->setPose(p);
		m_initial_q = p;  // save it for later usage in some animations, etc.
	}
	else
	{
		THROW_EXCEPTION(
			"Missing required XML node <init_pose>x y phi</init_pose>");
	}

	// -------------------------------------
	// (Optional) initial vel:
	// -------------------------------------
	if (const xml_node<>* nInitVel = rootNode.first_node("init_vel"); nInitVel)
	{
		mrpt::math::TTwist2D dq;
		if (3 !=
			::sscanf(
				nInitVel->value(), "%lf %lf %lf", &dq.vx, &dq.vy, &dq.omega))
			THROW_EXCEPTION_FMT(
				"Error parsing <init_vel>%s</init_vel>", nInitVel->value());
		dq.omega *= M_PI / 180.0;  // deg->rad

		// Convert twist (velocity) from local -> global coords:
		dq.rotate(this->getPose().yaw);
		this->setTwist(dq);
	}

	// -------------------------------------
	// Parse <publish> XML tag
	// -------------------------------------
	if (auto node = rootNode.first_node("publish"); node)
	{
		TParameterDefinitions params;
		params["publish_pose_topic"] = TParamEntry("%s", &publishPoseTopic_);
		params["publish_pose_period"] = TParamEntry("%lf", &publishPosePeriod_);

		params["publish_relative_pose_topic"] =
			TParamEntry("%s", &publishRelativePoseTopic_);
		std::string listObjects;
		params["publish_relative_pose_objects"] =
			TParamEntry("%s", &listObjects);

		const std::map<std::string, std::string> varValues = {{"NAME", m_name}};

		parse_xmlnode_children_as_param(*node, params, varValues);

		// Parse the "enabled" attribute:
		{
			bool publishEnabled = true;
			TParameterDefinitions auxPar;
			auxPar["enabled"] = TParamEntry("%bool", &publishEnabled);
			parse_xmlnode_attribs(*node, auxPar, varValues);

			// Reset publish topic if enabled==false
			if (!publishEnabled) publishPoseTopic_.clear();
		}

		if (!listObjects.empty())
		{
			mrpt::system::tokenize(
				mrpt::system::trim(listObjects), " ,",
				publishRelativePoseOfOtherObjects_);

#if 0
		std::cout << "[DEBUG] "
					 "Publishing relative poses of "
				  << publishRelativePoseOfOtherObjects_.size() << " objects ("
				  << listObjects << ") to topic " << publishRelativePoseTopic_
				  << std::endl;
#endif
		}
		ASSERT_(
			(publishRelativePoseOfOtherObjects_.empty() &&
			 publishRelativePoseTopic_.empty()) ||
			(!publishRelativePoseOfOtherObjects_.empty() &&
			 !publishRelativePoseTopic_.empty()));

	}  // end <publish>

	// Parse animation effects:
	// ----------------------------
	if (auto nAnim = rootNode.first_node("animation"); nAnim)
	{
		auto aType = nAnim->first_attribute("type");
		ASSERTMSG_(aType, "<animation> tag requires a type=\"...\" attribute");
		const std::string sType = aType->value();

		if (sType == "keyframes")
		{
			auto& poseSeq = m_anim_keyframes_path.emplace();
			poseSeq.setInterpolationMethod(mrpt::poses::imLinearSlerp);

			for (auto n = nAnim->first_node(); n; n = n->next_sibling())
			{
				if (strcmp(n->name(), "time_pose") != 0)
					continue;  // ignore unknowns

				mrpt::math::TPose3D p;
				double t = 0;
				if (4 !=
					::sscanf(
						n->value(), "%lf %lf %lf %lf", &t, &p.x, &p.y, &p.yaw))
					THROW_EXCEPTION_FMT(
						"Error parsing <time_pose>:\n%s", n->value());
				p.yaw *= M_PI / 180.0;	// deg->rad

				poseSeq.insert(mrpt::Clock::fromDouble(t), p);
			}

			// m_anim_keyframes_path->saveToTextFile(m_name + "_path.txt"s);
		}
		else
		{
			THROW_EXCEPTION_FMT("Invalid animation type='%s'", sType.c_str());
		}
	}

	return true;
	MRPT_END
}

void Simulable::internalHandlePublish(const TSimulContext& context)
{
	std::shared_lock lck(m_q_mtx);

	MRPT_START

	if (publishPoseTopic_.empty() && publishRelativePoseTopic_.empty()) return;

	auto& client = context.world->commsClient();

	const double tNow = mrpt::Clock::toDouble(mrpt::Clock::now());
	if (tNow < publishPoseLastTime_ + publishPosePeriod_) return;

	publishPoseLastTime_ = tNow;

	if (!publishPoseTopic_.empty())
	{
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
	}

	if (!publishRelativePoseTopic_.empty())
	{
		mvsim_msgs::TimeStampedPose msg;
		msg.set_unixtimestamp(tNow);
		msg.set_relativetoobjectid(m_name);

		const auto& allObjects =
			getSimulableWorldObject()->getListOfSimulableObjects();

		// detect other objects and publish their relative poses wrt me:
		for (const auto& otherId : publishRelativePoseOfOtherObjects_)
		{
			msg.set_objectid(otherId);

			if (auto itObj = allObjects.find(otherId);
				itObj != allObjects.end())
			{
				const auto relPose = itObj->second->m_q - m_q;

				auto pose = msg.mutable_pose();
				pose->set_x(relPose.x);
				pose->set_y(relPose.y);
				pose->set_z(relPose.z);
				pose->set_yaw(relPose.yaw);
				pose->set_pitch(relPose.pitch);
				pose->set_roll(relPose.roll);

				client.publishTopic(publishRelativePoseTopic_, msg);
			}
			else
			{
				std::cerr
					<< "[WARNING] Trying to publish relative pose of '"
					<< otherId << "' wrt '" << m_name
					<< "' but could not find any object in the world with "
					   "the former name."
					<< std::endl;
			}
		}
	}

	MRPT_END
}

void Simulable::registerOnServer(mvsim::Client& c)
{
	MRPT_START

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	// Topic:
	if (!publishPoseTopic_.empty())
		c.advertiseTopic<mvsim_msgs::TimeStampedPose>(publishPoseTopic_);

	if (!publishRelativePoseTopic_.empty())
		c.advertiseTopic<mvsim_msgs::TimeStampedPose>(
			publishRelativePoseTopic_);
#endif

	MRPT_END
}

void Simulable::setPose(const mrpt::math::TPose3D& p) const
{
	m_q_mtx.lock();
	const_cast<mrpt::math::TPose3D&>(m_q) = p;
	m_q_mtx.unlock();
}
