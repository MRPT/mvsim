/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <box2d/b2_contact.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/Simulable.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/VisualObject.h>
#include <mvsim/World.h>

#include <cmath>  // fmod()

#include "JointXMLnode.h"
#include "parse_utils.h"
#include "xml_utils.h"

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <mvsim/mvsim-msgs/TimeStampedPose.pb.h>

#endif

using namespace mvsim;

void Simulable::simul_pre_timestep(	 //
	[[maybe_unused]] const TSimulContext& context)
{
	// Follow animation, if enabled:
	if (anim_keyframes_path_ && !anim_keyframes_path_->empty())
	{
		auto& poseSeq = anim_keyframes_path_.value();

		mrpt::math::TPose3D q;
		bool interOk = false;
		const double tMax = mrpt::Clock::toDouble(poseSeq.rbegin()->first);

		poseSeq.interpolate(
			mrpt::Clock::fromDouble(std::fmod(context.simul_time, tMax)), q,
			interOk);

		if (interOk) this->setPose(initial_q_ + q);
	}

	if (!b2dBody_) return;

	// Pos:
	b2dBody_->SetTransform(b2Vec2(q_.x, q_.y), q_.yaw);

	// Vel:
	b2dBody_->SetLinearVelocity(b2Vec2(dq_.vx, dq_.vy));
	b2dBody_->SetAngularVelocity(dq_.omega);
}

void Simulable::simul_post_timestep(const TSimulContext& context)
{
	if (b2dBody_)
	{
		q_mtx_.lock();
		// simulable_parent_->physical_objects_mtx(): already locked by caller

		// Pos:
		const b2Vec2& pos = b2dBody_->GetPosition();
		const float angle = b2dBody_->GetAngle();
		q_.x = pos(0);
		q_.y = pos(1);
		q_.yaw = angle;
		// The rest (z,pitch,roll) will be always 0, unless other
		// world-element modifies them! (e.g. elevation map)

		// Update the GUI element **poses** only:
		if (auto* vo = meAsVisualObject(); vo)
			vo->guiUpdate(std::nullopt, std::nullopt);

		// Vel:
		const b2Vec2& vel = b2dBody_->GetLinearVelocity();
		const float w = b2dBody_->GetAngularVelocity();
		dq_.vx = vel(0);
		dq_.vy = vel(1);
		dq_.omega = w;

		// Estimate acceleration from finite differences:
		ddq_lin_ =
			(q_.translation() - former_q_.translation()) * (1.0 / context.dt);
		former_q_ = q_;

		// Instantaneous collision flag:
		isInCollision_ = false;
		if (b2ContactEdge* cl = b2dBody_->GetContactList();
			cl != nullptr && cl->contact != nullptr &&
			cl->contact->IsTouching())
		{
			// We may store with which other bodies it's in collision...
			isInCollision_ = true;
		}
		// Reseteable collision flag:
		hadCollisionFlag_ = hadCollisionFlag_ || isInCollision_;

		q_mtx_.unlock();
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
	std::shared_lock lck(q_mtx_);

	mrpt::math::TTwist2D local_vel = dq_;
	local_vel.rotate(-q_.yaw);	// "-" means inverse pose
	return local_vel;
}

mrpt::poses::CPose2D Simulable::getCPose2D() const
{
	std::shared_lock lck(q_mtx_);

	return {q_.x, q_.y, q_.yaw};
}

mrpt::poses::CPose3D Simulable::getCPose3D() const
{
	std::shared_lock lck(q_mtx_);
	return mrpt::poses::CPose3D(q_);
}

bool Simulable::parseSimulable(
	const JointXMLnode<>& rootNode, const ParseSimulableParams& psp)
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
		if (3 != ::sscanf(
					 mvsim::parse(
						 nPose->value(),
						 getSimulableWorldObject()->user_defined_variables())
						 .c_str(),
					 "%lf %lf %lf", &p.x, &p.y, &p.yaw))
			THROW_EXCEPTION_FMT(
				"Error parsing <init_pose>%s</init_pose>", nPose->value());
		p.yaw *= M_PI / 180.0;	// deg->rad

		this->setPose(p);
		initial_q_ = p;	 // save it for later usage in some animations, etc.
	}
	else if (const xml_node<>* nPose3 = rootNode.first_node("init_pose3d");
			 nPose3)
	{
		mrpt::math::TPose3D p;
		if (6 != ::sscanf(
					 mvsim::parse(
						 nPose3->value(),
						 getSimulableWorldObject()->user_defined_variables())
						 .c_str(),
					 "%lf %lf %lf %lf %lf %lf ", &p.x, &p.y, &p.z, &p.yaw,
					 &p.pitch, &p.roll))
			THROW_EXCEPTION_FMT(
				"Error parsing <init_pose3d>%s</init_pose3d>", nPose3->value());
		p.yaw *= M_PI / 180.0;	// deg->rad
		p.pitch *= M_PI / 180.0;  // deg->rad
		p.roll *= M_PI / 180.0;	 // deg->rad

		this->setPose(p);
		initial_q_ = p;	 // save it for later usage in some animations, etc.
	}
	else if (psp.init_pose_mandatory)
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
		if (3 != ::sscanf(
					 mvsim::parse(
						 nInitVel->value(),
						 getSimulableWorldObject()->user_defined_variables())
						 .c_str(),
					 "%lf %lf %lf", &dq.vx, &dq.vy, &dq.omega))
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

		auto varValues = simulable_parent_->user_defined_variables();
		varValues["NAME"] = name_;

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
			auto& poseSeq = anim_keyframes_path_.emplace();
			poseSeq.setInterpolationMethod(mrpt::poses::imLinearSlerp);

			for (auto n = nAnim->first_node(); n; n = n->next_sibling())
			{
				if (strcmp(n->name(), "time_pose") == 0)
				{
					mrpt::math::TPose3D p;
					double t = 0;
					if (4 != ::sscanf(
								 mvsim::parse(
									 n->value(), getSimulableWorldObject()
													 ->user_defined_variables())
									 .c_str(),
								 "%lf %lf %lf %lf", &t, &p.x, &p.y, &p.yaw))
						THROW_EXCEPTION_FMT(
							"Error parsing <time_pose>:\n%s", n->value());
					p.yaw *= M_PI / 180.0;	// deg->rad

					poseSeq.insert(mrpt::Clock::fromDouble(t), p);
				}
				else if (strcmp(n->name(), "time_pose3d") == 0)
				{
					mrpt::math::TPose3D p;
					double t = 0;
					if (7 != ::sscanf(
								 mvsim::parse(
									 n->value(), getSimulableWorldObject()
													 ->user_defined_variables())
									 .c_str(),
								 "%lf %lf %lf %lf %lf %lf %lf", &t, &p.x, &p.y,
								 &p.z, &p.yaw, &p.pitch, &p.roll))
						THROW_EXCEPTION_FMT(
							"Error parsing <time_pose3d>:\n%s", n->value());
					p.yaw *= M_PI / 180.0;	// deg->rad
					p.pitch *= M_PI / 180.0;  // deg->rad
					p.roll *= M_PI / 180.0;	 // deg->rad

					poseSeq.insert(mrpt::Clock::fromDouble(t), p);
				}
			}

			// anim_keyframes_path_->saveToTextFile(name_ + "_path.txt"s);
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
	std::shared_lock lck(q_mtx_);

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
		msg.set_objectid(name_);

		auto pose = msg.mutable_pose();
		pose->set_x(q_.x);
		pose->set_y(q_.y);
		pose->set_z(.0);
		pose->set_yaw(q_.yaw);
		pose->set_pitch(q_.pitch);
		pose->set_roll(q_.roll);

		client.publishTopic(publishPoseTopic_, msg);
	}

	if (!publishRelativePoseTopic_.empty())
	{
		mvsim_msgs::TimeStampedPose msg;
		msg.set_unixtimestamp(tNow);
		msg.set_relativetoobjectid(name_);

		// Note: getSimulableWorldObjectMtx() is already hold by my caller.
		const auto& allObjects =
			getSimulableWorldObject()->getListOfSimulableObjects();

		// detect other objects and publish their relative poses wrt me:
		for (const auto& otherId : publishRelativePoseOfOtherObjects_)
		{
			if (auto itObj = allObjects.find(otherId);
				itObj != allObjects.end())
			{
				msg.set_objectid(otherId);

				const auto relPose = itObj->second->q_ - q_;

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
					<< otherId << "' wrt '" << name_
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
	q_mtx_.lock();

	Simulable& me = const_cast<Simulable&>(*this);

	me.q_ = p;

	// Update the GUI element poses only:
	if (auto* vo = me.meAsVisualObject(); vo)
		vo->guiUpdate(std::nullopt, std::nullopt);

	q_mtx_.unlock();
}

mrpt::math::TPose3D Simulable::getPose() const
{
	q_mtx_.lock_shared();
	mrpt::math::TPose3D ret = q_;
	q_mtx_.unlock_shared();
	return ret;
}

mrpt::math::TPose3D Simulable::getPoseNoLock() const { return q_; }

mrpt::math::TTwist2D Simulable::getTwist() const
{
	q_mtx_.lock_shared();
	mrpt::math::TTwist2D ret = dq_;
	q_mtx_.unlock_shared();
	return ret;
}

mrpt::math::TVector3D Simulable::getLinearAcceleration() const
{
	q_mtx_.lock_shared();
	auto ret = ddq_lin_;
	q_mtx_.unlock_shared();
	return ret;
}

void Simulable::setTwist(const mrpt::math::TTwist2D& dq) const
{
	q_mtx_.lock();
	const_cast<mrpt::math::TTwist2D&>(dq_) = dq;

	if (b2dBody_)
	{
		mrpt::math::TTwist2D local_dq = dq.rotated(q_.yaw);

		b2dBody_->SetLinearVelocity(b2Vec2(local_dq.vx, local_dq.vy));
		b2dBody_->SetAngularVelocity(dq.omega);
	}

	q_mtx_.unlock();
}
