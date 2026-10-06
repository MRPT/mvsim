/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mvsim/World.h>

#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF
#include <mvsim/mvsim-msgs/GenericAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvGetLightState.pb.h>
#include <mvsim/mvsim-msgs/SrvGetLightStateAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPoseAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwist.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwistAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetLightState.pb.h>
#include <mvsim/mvsim-msgs/SrvSetLightStateAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvSetPoseAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvShutdown.pb.h>
#include <mvsim/mvsim-msgs/SrvShutdownAnswer.pb.h>
#endif

#include <map>

using namespace mvsim;

#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF

mvsim_msgs::SrvSetPoseAnswer World::srv_set_pose(const mvsim_msgs::SrvSetPose& req)
{
	mvsim_msgs::SrvSetPoseAnswer ans;
	ans.set_objectisincollision(false);

	const auto sId = req.objectid();

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());
	if (auto itV = simulableObjects_.find(sId); itV != simulableObjects_.end())
	{
		if (req.has_relativeincrement() && req.relativeincrement())
		{
			auto p = mrpt::poses::CPose3D(itV->second->getPose());
			p = p + mrpt::poses::CPose3D(
						req.pose().x(), req.pose().y(), req.pose().z(), req.pose().yaw(),
						req.pose().pitch(), req.pose().roll());
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
				{req.pose().x(), req.pose().y(), req.pose().z(), req.pose().yaw(),
				 req.pose().pitch(), req.pose().roll()});
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

mvsim_msgs::SrvGetPoseAnswer World::srv_get_pose(const mvsim_msgs::SrvGetPose& req)
{
	auto lckCopy = mrpt::lockHelper(copy_of_objects_dynstate_mtx_);

	mvsim_msgs::SrvGetPoseAnswer ans;
	const auto sId = req.objectid();
	ans.set_objectisincollision(false);

	if (auto itV = copy_of_objects_dynstate_pose_.find(sId);
		itV != copy_of_objects_dynstate_pose_.end())
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

		const auto t = copy_of_objects_dynstate_twist_.at(sId);
		auto* tw = ans.mutable_twist();
		tw->set_vx(t.vx);
		tw->set_vy(t.vy);
		tw->set_vz(0);
		tw->set_wx(0);
		tw->set_wy(0);
		tw->set_wz(t.omega);

		ans.set_objectisincollision(copy_of_objects_had_collision_.count(sId) != 0);
	}
	else
	{
		ans.set_success(false);
	}

	lckCopy.unlock();

	{
		const auto lckPhys = mrpt::lockHelper(reset_collision_flags_mtx_);
		reset_collision_flags_.insert(sId);
	}
	return ans;
}

mvsim_msgs::SrvSetControllerTwistAnswer World::srv_set_controller_twist(
	const mvsim_msgs::SrvSetControllerTwist& req)
{
	std::lock_guard<std::mutex> lck(simulationStepRunningMtx_);

	mvsim_msgs::SrvSetControllerTwistAnswer ans;
	ans.set_success(false);

	const auto sId = req.objectid();

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());

	auto itV = simulableObjects_.find(sId);
	if (itV == simulableObjects_.end())
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
		ans.set_errormessage("objectId vehicle seems not to have any controller");
		return ans;
	}

	const mrpt::math::TTwist2D t(
		req.twistsetpoint().vx(), req.twistsetpoint().vy(), req.twistsetpoint().wz());

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

mvsim_msgs::SrvShutdownAnswer World::srv_shutdown(
	[[maybe_unused]] const mvsim_msgs::SrvShutdown& req)
{
	mvsim_msgs::SrvShutdownAnswer ans;
	ans.set_accepted(true);

	this->simulator_must_close(true);

	return ans;
}

namespace
{
/** Finds the light group of an object, or returns an error message */
std::string findLightGroup(
	const World::SimulableList& objs, const std::string& objectId, const std::string& group,
	CVisualObject*& obj)
{
	const auto it = objs.find(objectId);
	obj = it == objs.end() ? nullptr : dynamic_cast<CVisualObject*>(it->second.get());
	if (!obj)
	{
		return "objectId not found";
	}
	if (!obj->lightGroupState(group).has_value())
	{
		std::string available;
		for (const auto& name : obj->lightGroupNames())
		{
			available += (available.empty() ? "" : ", ") + name;
		}
		return "light group not found. Available ones: [" + available + "]";
	}
	return {};
}
}  // namespace

mvsim_msgs::SrvSetLightStateAnswer World::srv_set_light_state(
	const mvsim_msgs::SrvSetLightState& req)
{
	mvsim_msgs::SrvSetLightStateAnswer ans;

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());

	CVisualObject* obj = nullptr;
	if (const auto err = findLightGroup(simulableObjects_, req.objectid(), req.lightgroup(), obj);
		!err.empty())
	{
		ans.set_success(false);
		ans.set_errormessage(err);
		return ans;
	}
	ans.set_success(obj->setLightGroupState(req.lightgroup(), req.on()));
	return ans;
}

mvsim_msgs::SrvGetLightStateAnswer World::srv_get_light_state(
	const mvsim_msgs::SrvGetLightState& req)
{
	mvsim_msgs::SrvGetLightStateAnswer ans;

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());

	CVisualObject* obj = nullptr;
	if (const auto err = findLightGroup(simulableObjects_, req.objectid(), req.lightgroup(), obj);
		!err.empty())
	{
		ans.set_success(false);
		ans.set_errormessage(err);
		return ans;
	}
	ans.set_success(true);
	ans.set_on(obj->lightGroupState(req.lightgroup()).value_or(false));
	return ans;
}

#endif	// MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF

void World::internal_advertiseServices()
{
#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF
	// global services:
	client_.advertiseService<mvsim_msgs::SrvSetPose, mvsim_msgs::SrvSetPoseAnswer>(
		"set_pose", [this](const auto& req) { return srv_set_pose(req); });

	client_.advertiseService<mvsim_msgs::SrvGetPose, mvsim_msgs::SrvGetPoseAnswer>(
		"get_pose", [this](const auto& req) { return srv_get_pose(req); });

	client_.advertiseService<
		mvsim_msgs::SrvSetControllerTwist, mvsim_msgs::SrvSetControllerTwistAnswer>(
		"set_controller_twist", [this](const auto& req) { return srv_set_controller_twist(req); });

	client_.advertiseService<mvsim_msgs::SrvShutdown, mvsim_msgs::SrvShutdownAnswer>(
		"shutdown", [this](const auto& req) { return srv_shutdown(req); });

	client_.advertiseService<mvsim_msgs::SrvSetLightState, mvsim_msgs::SrvSetLightStateAnswer>(
		"set_light_state", [this](const auto& req) { return srv_set_light_state(req); });

	client_.advertiseService<mvsim_msgs::SrvGetLightState, mvsim_msgs::SrvGetLightStateAnswer>(
		"get_light_state", [this](const auto& req) { return srv_get_light_state(req); });

#endif
}
