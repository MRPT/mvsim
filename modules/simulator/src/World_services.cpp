/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mvsim/World.h>

#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF
#include <mvsim/mvsim-msgs/GenericAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPoseAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwist.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwistAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvSetPoseAnswer.pb.h>
#endif

#include <algorithm>  // count()
#include <map>
#include <stdexcept>

using namespace mvsim;

#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF

mvsim_msgs::SrvSetPoseAnswer World::srv_set_pose(
	const mvsim_msgs::SrvSetPose& req)
{
	mvsim_msgs::SrvSetPoseAnswer ans;
	ans.set_objectisincollision(false);

	const auto sId = req.objectid();

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());
	if (auto itV = m_simulableObjects.find(sId);
		itV != m_simulableObjects.end())
	{
		if (req.has_relativeincrement() && req.relativeincrement())
		{
			auto p = mrpt::poses::CPose3D(itV->second->getPose());
			p = p + mrpt::poses::CPose3D(
						req.pose().x(), req.pose().y(), req.pose().z(),
						req.pose().yaw(), req.pose().pitch(),
						req.pose().roll());
			itV->second->setPose(p.asTPose());

			auto* absPose = ans.mutable_objectglobalpose();
			absPose->set_x(p.x());
			absPose->set_y(p.y());
			absPose->set_z(p.z());
			absPose->set_yaw(p.yaw());
			absPose->set_pitch(p.pitch());
			absPose->set_roll(p.roll());
		}
		else
		{
			itV->second->setPose(
				{req.pose().x(), req.pose().y(), req.pose().z(),
				 req.pose().yaw(), req.pose().pitch(), req.pose().roll()});
		}
		ans.set_success(true);
		ans.set_objectisincollision(itV->second->hadCollision());
		itV->second->resetCollisionFlag();
	}
	else
	{
		ans.set_success(false);
	}
	return ans;
}

mvsim_msgs::SrvGetPoseAnswer World::srv_get_pose(
	const mvsim_msgs::SrvGetPose& req)
{
	auto lckCopy = mrpt::lockHelper(m_copy_of_objects_dynstate_mtx);

	mvsim_msgs::SrvGetPoseAnswer ans;
	const auto sId = req.objectid();
	ans.set_objectisincollision(false);

	if (auto itV = m_copy_of_objects_dynstate_pose.find(sId);
		itV != m_copy_of_objects_dynstate_pose.end())
	{
		ans.set_success(true);
		const mrpt::math::TPose3D p = itV->second;
		auto* po = ans.mutable_pose();
		po->set_x(p.x);
		po->set_y(p.y);
		po->set_z(p.z);
		po->set_yaw(p.yaw);
		po->set_pitch(p.pitch);
		po->set_roll(p.roll);

		const auto t = m_copy_of_objects_dynstate_twist.at(sId);
		auto* tw = ans.mutable_twist();
		tw->set_vx(t.vx);
		tw->set_vy(t.vy);
		tw->set_vz(0);
		tw->set_wx(0);
		tw->set_wy(0);
		tw->set_wz(t.omega);

		ans.set_objectisincollision(
			m_copy_of_objects_had_collision.count(sId) != 0);
	}
	else
	{
		ans.set_success(false);
	}

	lckCopy.unlock();

	{
		const auto lckPhys = mrpt::lockHelper(m_reset_collision_flags_mtx);
		m_reset_collision_flags.insert(sId);
	}
	return ans;
}

mvsim_msgs::SrvSetControllerTwistAnswer World::srv_set_controller_twist(
	const mvsim_msgs::SrvSetControllerTwist& req)
{
	std::lock_guard<std::mutex> lck(m_simulationStepRunningMtx);

	mvsim_msgs::SrvSetControllerTwistAnswer ans;
	ans.set_success(false);

	const auto sId = req.objectid();

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());

	auto itV = m_simulableObjects.find(sId);
	if (itV == m_simulableObjects.end())
	{
		ans.set_errormessage("objectId not found");
		return ans;
	}

	auto veh = std::dynamic_pointer_cast<VehicleBase>(itV->second);
	if (!veh)
	{
		ans.set_errormessage("objectId is not of VehicleBase type");
		return ans;
	}

	mvsim::ControllerBaseInterface* controller = veh->getControllerInterface();
	if (!controller)
	{
		ans.set_errormessage(
			"objectId vehicle seems not to have any controller");
		return ans;
	}

	const mrpt::math::TTwist2D t(
		req.twistsetpoint().vx(), req.twistsetpoint().vy(),
		req.twistsetpoint().wz());

	const bool ctrlAcceptTwist = controller->setTwistCommand(t);
	if (!ctrlAcceptTwist)
	{
		ans.set_errormessage(
			"objectId vehicle controller did not accept the twist "
			"command");
		return ans;
	}

	ans.set_success(true);
	return ans;
}

#endif	// MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF

void World::internal_advertiseServices()
{
#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF
	// global services:
	m_client
		.advertiseService<mvsim_msgs::SrvSetPose, mvsim_msgs::SrvSetPoseAnswer>(
			"set_pose", [this](const auto& req) { return srv_set_pose(req); });

	m_client
		.advertiseService<mvsim_msgs::SrvGetPose, mvsim_msgs::SrvGetPoseAnswer>(
			"get_pose", [this](const auto& req) { return srv_get_pose(req); });

	m_client.advertiseService<
		mvsim_msgs::SrvSetControllerTwist,
		mvsim_msgs::SrvSetControllerTwistAnswer>(
		"set_controller_twist",
		[this](const auto& req) { return srv_set_controller_twist(req); });

#endif
}
