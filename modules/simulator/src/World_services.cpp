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
#include <mvsim/mvsim-msgs/SrvGetAllPoses.pb.h>
#include <mvsim/mvsim-msgs/SrvGetAllPosesAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvGetLightState.pb.h>
#include <mvsim/mvsim-msgs/SrvGetLightStateAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPoseAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvRemoveObjects.pb.h>
#include <mvsim/mvsim-msgs/SrvRemoveObjectsAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwist.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwistAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetLightState.pb.h>
#include <mvsim/mvsim-msgs/SrvSetLightStateAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvSetPoseAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvShutdown.pb.h>
#include <mvsim/mvsim-msgs/SrvShutdownAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvInsertEntities.pb.h>
#include <mvsim/mvsim-msgs/SrvInsertEntitiesAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvRemoveEntities.pb.h>
#include <mvsim/mvsim-msgs/SrvRemoveEntitiesAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSpawnObjects.pb.h>
#include <mvsim/mvsim-msgs/SrvSpawnObjectsAnswer.pb.h>
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
	else if (auto rp = runtimeObjects_.getPose(sId); rp.has_value())
	{
		// Runtime (visual-only) objects:
		auto p = mrpt::math::TPose3D(
			req.pose().x(), req.pose().y(), req.pose().z(), req.pose().yaw(), req.pose().pitch(),
			req.pose().roll());
		if (req.has_relativeincrement() && req.relativeincrement())
		{
			p = (mrpt::poses::CPose3D(*rp) + mrpt::poses::CPose3D(p)).asTPose();
		}
		ans.set_success(runtimeObjects_.setPose(sId, p));
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
	else if (auto rp = runtimeObjects_.getPose(sId); rp.has_value())
	{
		// Runtime (visual-only) objects:
		ans.set_success(true);
		auto* po = ans.mutable_pose();
		po->set_x(rp->x);
		po->set_y(rp->y);
		po->set_z(rp->z);
		po->set_yaw(rp->yaw);
		po->set_pitch(rp->pitch);
		po->set_roll(rp->roll);
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

namespace
{
mrpt::img::TColor colorFromRGBA(uint32_t c)
{
	return mrpt::img::TColor(
		static_cast<uint8_t>(c >> 24), static_cast<uint8_t>(c >> 16), static_cast<uint8_t>(c >> 8),
		static_cast<uint8_t>(c));
}

RuntimeObjectDescription fromProto(const mvsim_msgs::RuntimeObject& o)
{
	RuntimeObjectDescription d;
	d.name = o.name();
	d.shape = static_cast<RuntimeObjectDescription::Shape>(o.shape());
	if (o.has_pose())
	{
		const auto& p = o.pose();
		d.pose = {p.x(), p.y(), p.z(), p.yaw(), p.pitch(), p.roll()};
	}
	d.size = {o.sizex(), o.sizey(), o.sizez()};
	d.color = colorFromRGBA(o.color());
	for (int i = 0; i + 1 < o.polygonxy_size(); i += 2)
	{
		d.polygon.emplace_back(o.polygonxy(i), o.polygonxy(i + 1));
	}
	for (int i = 0; i + 2 < o.pointsxyz_size(); i += 3)
	{
		d.points.emplace_back(o.pointsxyz(i), o.pointsxyz(i + 1), o.pointsxyz(i + 2));
	}
	for (const auto c : o.pointcolors())
	{
		d.point_colors.push_back(colorFromRGBA(c));
	}
	d.texture = o.texture();
	d.visible_to_sensors = o.visibletosensors();
	d.on_ground = o.onground();
	return d;
}
}  // namespace

mvsim_msgs::SrvSpawnObjectsAnswer World::srv_spawn_objects(const mvsim_msgs::SrvSpawnObjects& req)
{
	mvsim_msgs::SrvSpawnObjectsAnswer ans;
	try
	{
		std::vector<RuntimeObjectDescription> objs;
		objs.reserve(static_cast<size_t>(req.objects_size()));
		for (const auto& o : req.objects())
		{
			objs.push_back(fromProto(o));
		}
		runtimeObjects_.spawn(objs);
		ans.set_success(true);
	}
	catch (const std::exception& e)
	{
		ans.set_success(false);
		ans.set_errormessage(e.what());
	}
	return ans;
}

mvsim_msgs::SrvRemoveObjectsAnswer World::srv_remove_objects(
	const mvsim_msgs::SrvRemoveObjects& req)
{
	mvsim_msgs::SrvRemoveObjectsAnswer ans;
	size_t n = runtimeObjects_.remove({req.names().begin(), req.names().end()});
	if (req.has_prefix())
	{
		n += runtimeObjects_.removeByPrefix(req.prefix());
	}
	ans.set_success(true);
	ans.set_numremoved(static_cast<uint32_t>(n));
	return ans;
}

namespace
{
// Services run in the communications thread: entities are inserted or removed
// in the simulation thread.
constexpr auto kSimulationThreadTimeout = std::chrono::seconds(30);
}  // namespace

mvsim_msgs::SrvInsertEntitiesAnswer World::srv_insert_entities(
	const mvsim_msgs::SrvInsertEntities& req)
{
	mvsim_msgs::SrvInsertEntitiesAnswer ans;
	auto names = std::make_shared<std::vector<std::string>>();
	const std::string xml = req.xml();
	const std::string basePath = req.has_basepath() ? req.basepath() : std::string();
	try
	{
		auto fut = runInSimulationThread([this, names, xml, basePath]()
										 { *names = insertEntitiesFromXML(xml, basePath); });
		if (fut.wait_for(kSimulationThreadTimeout) != std::future_status::ready)
		{
			THROW_EXCEPTION("Timeout waiting for the simulation thread");
		}
		fut.get();
		ans.set_success(true);
		for (const auto& n : *names)
		{
			ans.add_names(n);
		}
	}
	catch (const std::exception& e)
	{
		ans.set_success(false);
		ans.set_errormessage(e.what());
	}
	return ans;
}

mvsim_msgs::SrvRemoveEntitiesAnswer World::srv_remove_entities(
	const mvsim_msgs::SrvRemoveEntities& req)
{
	mvsim_msgs::SrvRemoveEntitiesAnswer ans;
	auto notFound = std::make_shared<std::vector<std::string>>();
	const std::vector<std::string> names(req.names().begin(), req.names().end());
	try
	{
		auto fut = runInSimulationThread(
			[this, notFound, names]()
			{
				for (const auto& n : names)
				{
					if (!removeEntity(n))
					{
						notFound->push_back(n);
					}
				}
			});
		if (fut.wait_for(kSimulationThreadTimeout) != std::future_status::ready)
		{
			THROW_EXCEPTION("Timeout waiting for the simulation thread");
		}
		fut.get();
		ans.set_numremoved(static_cast<uint32_t>(names.size() - notFound->size()));
		ans.set_success(notFound->empty());
		if (!notFound->empty())
		{
			std::string msg = "Not found:";
			for (const auto& n : *notFound)
			{
				msg += " '" + n + "'";
			}
			ans.set_errormessage(msg);
		}
	}
	catch (const std::exception& e)
	{
		ans.set_success(false);
		ans.set_errormessage(e.what());
	}
	return ans;
}

mvsim_msgs::SrvGetAllPosesAnswer World::srv_get_all_poses(const mvsim_msgs::SrvGetAllPoses& req)
{
	mvsim_msgs::SrvGetAllPosesAnswer ans;
	const auto snap = getGroundTruthSnapshot(req.prefix());
	ans.set_success(true);
	ans.set_simultime(snap.simul_time);
	for (const auto& o : snap.objects)
	{
		auto* np = ans.add_objects();
		np->set_name(o.name);
		auto* po = np->mutable_pose();
		po->set_x(o.pose.x);
		po->set_y(o.pose.y);
		po->set_z(o.pose.z);
		po->set_yaw(o.pose.yaw);
		po->set_pitch(o.pose.pitch);
		po->set_roll(o.pose.roll);
		auto* tw = np->mutable_twist();
		tw->set_vx(o.twist.vx);
		tw->set_vy(o.twist.vy);
		tw->set_vz(0);
		tw->set_wx(0);
		tw->set_wy(0);
		tw->set_wz(o.twist.omega);
	}
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

	client_.advertiseService<mvsim_msgs::SrvSpawnObjects, mvsim_msgs::SrvSpawnObjectsAnswer>(
		"spawn_objects", [this](const auto& req) { return srv_spawn_objects(req); });

	client_.advertiseService<mvsim_msgs::SrvRemoveObjects, mvsim_msgs::SrvRemoveObjectsAnswer>(
		"remove_objects", [this](const auto& req) { return srv_remove_objects(req); });

	client_.advertiseService<mvsim_msgs::SrvInsertEntities, mvsim_msgs::SrvInsertEntitiesAnswer>(
		"insert_entities", [this](const auto& req) { return srv_insert_entities(req); });

	client_.advertiseService<mvsim_msgs::SrvRemoveEntities, mvsim_msgs::SrvRemoveEntitiesAnswer>(
		"remove_entities", [this](const auto& req) { return srv_remove_entities(req); });

	client_.advertiseService<mvsim_msgs::SrvGetAllPoses, mvsim_msgs::SrvGetAllPosesAnswer>(
		"get_all_poses", [this](const auto& req) { return srv_get_all_poses(req); });

#endif
}
