/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Standard simulator services of the "simulation_interfaces" package:
// entities are given as MVSim world XML.

#if defined(MVSIM_HAS_SIMULATION_INTERFACES)

#include <mrpt/ros2bridge/pose.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/WorldElements/WorldElementBase.h>
#include <mvsim/mvsim_node_core.h>

#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>

namespace
{
using simulation_interfaces::msg::Result;

void setResult(Result& r, uint8_t code, const std::string& msg = {})
{
	r.result = code;
	r.error_message = msg;
}

bool isIdentityAtOrigin(const geometry_msgs::msg::Pose& p)
{
	const auto& q = p.orientation;
	return p.position.x == 0 && p.position.y == 0 && p.position.z == 0 && q.x == 0 && q.y == 0 &&
		   q.z == 0 && (q.w == 1 || q.w == 0);
}

// Global (world frame) twist to the vehicle frame, and back:
mrpt::math::TTwist2D toLocal(const mrpt::math::TTwist2D& g, double yaw)
{
	const double c = std::cos(yaw);
	const double s = std::sin(yaw);
	return {c * g.vx + s * g.vy, -s * g.vx + c * g.vy, g.omega};
}
mrpt::math::TTwist2D toGlobal(const mrpt::math::TTwist2D& l, double yaw)
{
	const double c = std::cos(yaw);
	const double s = std::sin(yaw);
	return {c * l.vx - s * l.vy, s * l.vx + c * l.vy, l.omega};
}

// All named entities: vehicles, blocks, elements, actors and runtime objects.
std::vector<std::string> allEntityNames(const mvsim::World& w)
{
	std::vector<std::string> names;
	for (const auto& o : w.getGroundTruthSnapshot().objects)
	{
		names.push_back(o.name);
	}
	for (const auto& e : w.getListOfWorldElements())
	{
		if (!e->getName().empty())
		{
			names.push_back(e->getName());
		}
	}
	return names;
}

bool nameInUse(const mvsim::World& w, const std::string& name)
{
	const auto names = allEntityNames(w);
	return std::find(names.begin(), names.end(), name) != names.end();
}
}  // namespace

void MVSimNode::initSimulationInterfacesServices()
{
	using namespace simulation_interfaces::srv;
	using simulation_interfaces::msg::SimulatorFeatures;

	// These run in the ROS spin thread, the same that runs the simulation, so
	// they can insert and remove entities directly.

	simInterfacesServices_.spawn = n_->create_service<SpawnEntity>(
		"spawn_entity",
		[this](
			const std::shared_ptr<SpawnEntity::Request> req,
			std::shared_ptr<SpawnEntity::Response> res)
		{
#if defined(MVSIM_SIMULATION_INTERFACES_V2)
			const auto& uri = req->entity_resource.uri;
			const auto& resourceString = req->entity_resource.resource_string;
#else
			const auto& uri = req->uri;
			const auto& resourceString = req->resource_string;
#endif
			mvsim::World::InsertOptions opts;
			std::string xml;
			if (!uri.empty())
			{
				std::string path = uri;
				const std::string fileScheme = "file://";
				if (path.rfind(fileScheme, 0) == 0)
				{
					path = path.substr(fileScheme.size());
				}
				else if (path.find("://") != std::string::npos)
				{
					setResult(
						res->result, SpawnEntity::Response::UNSUPPORTED_FORMAT,
						"Only file paths or file:// URIs are supported");
					return;
				}
				std::ifstream f(path);
				std::stringstream ss;
				ss << f.rdbuf();
				xml = ss.str();
				if (!f.is_open() || xml.empty())
				{
					setResult(
						res->result, SpawnEntity::Response::RESOURCE_PARSE_ERROR,
						"Cannot read file: " + path);
					return;
				}
				opts.basePath = mrpt::system::extractFileDirectory(path);
			}
			else if (!resourceString.empty())
			{
				xml = resourceString;
			}
			else
			{
				setResult(res->result, SpawnEntity::Response::NO_RESOURCE);
				return;
			}

			if (!req->name.empty())
			{
				std::string name = req->name;
				for (int i = 1; nameInUse(*mvsim_world_, name); i++)
				{
					if (!req->allow_renaming)
					{
						setResult(
							res->result, SpawnEntity::Response::NAME_NOT_UNIQUE,
							"Name already in use: " + req->name);
						return;
					}
					name = req->name + "_" + std::to_string(i);
				}
				opts.name = name;
			}

			// Topics of vehicles are always under their name:
			const auto& ns = req->entity_namespace;
			if (!ns.empty() && ns != req->name && ns != "/" + req->name)
			{
				setResult(
					res->result, SpawnEntity::Response::NAMESPACE_INVALID,
					"The namespace must be empty or the entity name");
				return;
			}

			const auto& frame = req->initial_pose.header.frame_id;
			if (!frame.empty() && frame != world_frame_id_)
			{
				setResult(
					res->result, SpawnEntity::Response::INVALID_POSE,
					"initial_pose must be in frame '" + world_frame_id_ + "'");
				return;
			}
			if (!isIdentityAtOrigin(req->initial_pose.pose))
			{
				opts.pose = mrpt::ros2bridge::fromROS(req->initial_pose.pose).asTPose();
			}

			try
			{
				const auto names = mvsim_world_->insertEntitiesFromXML(xml, opts);
				std::string all;
				for (const auto& n : names)
				{
					all += (all.empty() ? "" : ",") + n;
				}
				res->entity_name = all;
				setResult(res->result, Result::RESULT_OK);
			}
			catch (const std::exception& e)
			{
				setResult(res->result, SpawnEntity::Response::RESOURCE_PARSE_ERROR, e.what());
			}
		});

	simInterfacesServices_.del = n_->create_service<DeleteEntity>(
		"delete_entity",
		[this](
			const std::shared_ptr<DeleteEntity::Request> req,
			std::shared_ptr<DeleteEntity::Response> res)
		{
			if (mvsim_world_->removeEntity(req->entity))
			{
				setResult(res->result, Result::RESULT_OK);
			}
			else
			{
				setResult(res->result, Result::RESULT_NOT_FOUND, "No entity: " + req->entity);
			}
		});

	simInterfacesServices_.getEntities = n_->create_service<GetEntities>(
		"get_entities",
		[this](
			const std::shared_ptr<GetEntities::Request> req,
			std::shared_ptr<GetEntities::Response> res)
		{
			const auto& f = req->filters;
			if (!f.categories.empty() || !f.tags.tags.empty() || !f.bounds.points.empty())
			{
				setResult(
					res->result, Result::RESULT_FEATURE_UNSUPPORTED,
					"Only filtering by name is supported");
				return;
			}
			try
			{
				const std::regex re(f.filter, std::regex::extended);
				for (const auto& n : allEntityNames(*mvsim_world_))
				{
					if (f.filter.empty() || std::regex_search(n, re))
					{
						res->entities.push_back(n);
					}
				}
				setResult(res->result, Result::RESULT_OK);
			}
			catch (const std::regex_error& e)
			{
				setResult(res->result, Result::RESULT_OPERATION_FAILED, e.what());
			}
		});

	simInterfacesServices_.getState = n_->create_service<GetEntityState>(
		"get_entity_state",
		[this](
			const std::shared_ptr<GetEntityState::Request> req,
			std::shared_ptr<GetEntityState::Response> res)
		{
			res->state.header.frame_id = world_frame_id_;
			res->state.header.stamp = myNow();
			for (const auto& o : mvsim_world_->getGroundTruthSnapshot().objects)
			{
				if (o.name != req->entity)
				{
					continue;
				}
				res->state.pose = mrpt::ros2bridge::toROS_Pose(o.pose);
				const auto g = toGlobal(o.twist, o.pose.yaw);
				res->state.twist.linear.x = g.vx;
				res->state.twist.linear.y = g.vy;
				res->state.twist.angular.z = g.omega;
				setResult(res->result, Result::RESULT_OK);
				return;
			}
			for (const auto& e : mvsim_world_->getListOfWorldElements())
			{
				if (e->getName() == req->entity)
				{
					res->state.pose = mrpt::ros2bridge::toROS_Pose(e->getPose());
					setResult(res->result, Result::RESULT_OK);
					return;
				}
			}
			setResult(res->result, Result::RESULT_NOT_FOUND, "No entity: " + req->entity);
		});

	simInterfacesServices_.setState = n_->create_service<SetEntityState>(
		"set_entity_state",
		[this](
			const std::shared_ptr<SetEntityState::Request> req,
			std::shared_ptr<SetEntityState::Response> res)
		{
#if defined(MVSIM_SIMULATION_INTERFACES_V2)
			const bool setPose = req->set_pose;
			const bool setTwist = req->set_twist;
			if (req->set_acceleration)
			{
				setResult(
					res->result, Result::RESULT_FEATURE_UNSUPPORTED,
					"Setting the acceleration is not supported");
				return;
			}
#else
			const bool setPose = true;
			const bool setTwist = true;
#endif
			const auto& frame = req->state.header.frame_id;
			if (!frame.empty() && frame != world_frame_id_)
			{
				setResult(
					res->result, Result::RESULT_OPERATION_FAILED,
					"The state must be in frame '" + world_frame_id_ + "'");
				return;
			}
			const auto pose = mrpt::ros2bridge::fromROS(req->state.pose).asTPose();
			const mrpt::math::TTwist2D twistGlobal(
				req->state.twist.linear.x, req->state.twist.linear.y, req->state.twist.angular.z);

			auto lck = mrpt::lockHelper(mvsim_world_->getListOfSimulableObjectsMtx());
			auto& objs = mvsim_world_->getListOfSimulableObjects();
			if (auto it = objs.find(req->entity); it != objs.end())
			{
				if (setPose)
				{
					it->second->setPose(pose);
				}
				if (setTwist)
				{
					const double yaw = setPose ? pose.yaw : it->second->getPose().yaw;
					it->second->setRefVelocityLocal(toLocal(twistGlobal, yaw));
				}
				setResult(res->result, Result::RESULT_OK);
				return;
			}
			lck.unlock();
			if (setPose && mvsim_world_->runtimeObjects().setPose(req->entity, pose))
			{
				setResult(res->result, Result::RESULT_OK);
				return;
			}
			setResult(res->result, Result::RESULT_NOT_FOUND, "No entity: " + req->entity);
		});

	simInterfacesServices_.features = n_->create_service<GetSimulatorFeatures>(
		"get_simulator_features",
		[](const std::shared_ptr<GetSimulatorFeatures::Request>,
		   std::shared_ptr<GetSimulatorFeatures::Response> res)
		{
			auto& f = res->features;
			f.features = {
				SimulatorFeatures::SPAWNING, SimulatorFeatures::DELETING,
				SimulatorFeatures::SPAWNING_RESOURCE_STRING,
				SimulatorFeatures::ENTITY_STATE_GETTING, SimulatorFeatures::ENTITY_STATE_SETTING};
			f.spawn_formats = {"mvsim_xml"};
			f.custom_info =
				"Entities are given as MVSim world XML (<vehicle>, <block>, <element>...). See "
				"https://mvsimulator.readthedocs.io";
		});
}

#endif	// MVSIM_HAS_SIMULATION_INTERFACES
