/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <box2d/b2_world.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/World.h>

#include <algorithm>
#include <set>

#include "xml_utils.h"

using namespace mvsim;

namespace
{
template <class LIST>
void eraseByPointer(LIST& lst, const Simulable* obj)
{
	for (auto it = lst.begin(); it != lst.end();)
	{
		if (dynamic_cast<const Simulable*>(it->second.get()) == obj)
		{
			it = lst.erase(it);
		}
		else
		{
			++it;
		}
	}
}
}  // namespace

void World::registerCallbackOnEntityChange(const on_entity_change_callback_t& f)
{
	auto lck = mrpt::lockHelper(callbacksOnEntityChangeMtx_);
	callbacksOnEntityChange_.push_back(f);
}

void World::internalNotifyEntityChange(const EntityChange& c)
{
	std::vector<on_entity_change_callback_t> cbs;
	{
		auto lck = mrpt::lockHelper(callbacksOnEntityChangeMtx_);
		cbs = callbacksOnEntityChange_;
	}
	for (const auto& cb : cbs)
	{
		cb(c);
	}
}

std::future<void> World::runInSimulationThread(const std::function<void()>& task)
{
	std::packaged_task<void()> pt(task);
	auto fut = pt.get_future();
	auto lck = mrpt::lockHelper(simulationThreadTasksMtx_);
	simulationThreadTasks_.push_back(std::move(pt));
	return fut;
}

void World::internalRunSimulationThreadTasks()
{
	std::vector<std::packaged_task<void()>> tasks;
	{
		auto lck = mrpt::lockHelper(simulationThreadTasksMtx_);
		tasks = std::move(simulationThreadTasks_);
		simulationThreadTasks_.clear();
	}
	for (auto& t : tasks)
	{
		t();  // exceptions are stored in the future
	}
}

void World::internalResetPerObjectCollisionCaches()
{
	for (auto& e : obstacles_for_each_obj_)
	{
		if (e.has_value() && e->collide_body)
		{
			box2d_world_->DestroyBody(e->collide_body);
		}
	}
	obstacles_for_each_obj_.clear();

	for (auto& e : worldElements_)
	{
		e->onSimulableObjectsChanged();
	}
	lut2d_objects_is_up_to_date_ = false;
}

void World::internalDestroyBox2DBodiesOf(Simulable& obj)
{
	// Joints attached to the body are destroyed by Box2D with it:
	const auto& name = obj.getName();
	joints_.erase(
		std::remove_if(
			joints_.begin(), joints_.end(),
			[&](const WorldJoint& j) { return j.bodyA_name == name || j.bodyB_name == name; }),
		joints_.end());

	obj.destroyBox2DBodies(*box2d_world_);
}

std::vector<std::string> World::insertEntitiesFromXML(
	const std::string& xmlText, const std::string& basePath)
{
	std::string text = xmlText;
	if (text.find("<mvsim_world") == std::string::npos)
	{
		text = "<mvsim_world version=\"1.0\">\n" + text + "\n</mvsim_world>";
	}
	const std::string base =
		basePath.empty() ? basePath_ : mrpt::system::toAbsolutePath(basePath, false);

	std::vector<EntityChange> changes;
	{
		std::lock_guard<std::mutex> lckStep(simulationStepRunningMtx_);
		auto lckWorld = mrpt::lockHelper(world_cs_);
		auto lckPhys = mrpt::lockHelper(physical_objects_mtx());

		// Existing entities, to tell the new ones apart:
		std::set<const Simulable*> existing;
		for (const auto& [n, v] : vehicles_)
		{
			existing.insert(v.get());
		}
		for (const auto& [n, b] : blocks_)
		{
			existing.insert(b.get());
		}
		for (const auto& e : worldElements_)
		{
			existing.insert(e.get());
		}

		const auto collectNew = [&]()
		{
			std::vector<EntityChange> lst;
			for (const auto& [n, v] : vehicles_)
			{
				if (!existing.count(v.get()))
				{
					lst.push_back({v->getName(), EntityKind::Vehicle, true, v});
				}
			}
			for (const auto& [n, b] : blocks_)
			{
				if (!existing.count(b.get()))
				{
					lst.push_back({b->getName(), EntityKind::Block, true, b});
				}
			}
			for (const auto& e : worldElements_)
			{
				if (!existing.count(e.get()))
				{
					lst.push_back({e->getName(), EntityKind::Element, true, e});
				}
			}
			return lst;
		};

		const auto prevFileDir = userDefinedVariables_["MVSIM_CURRENT_FILE_DIRECTORY"];
		userDefinedVariables_["MVSIM_CURRENT_FILE_DIRECTORY"] = base;

		try
		{
			const auto [xml, root] = readXmlTextAndGetRoot(text, base + "/runtime_entities.xml");
			(void)xml;
			if (std::string(root->name()) != "mvsim_world")
			{
				THROW_EXCEPTION_FMT(
					"XML root element is '%s' ('mvsim_world' expected)", root->name());
			}
			for (auto* node = root->first_node(); node; node = node->next_sibling(nullptr))
			{
				internal_recursive_parse_XML({node, base});
			}
			userDefinedVariables_["MVSIM_CURRENT_FILE_DIRECTORY"] = prevFileDir;

			changes = collectNew();

			// Unnamed elements get a unique name, so they can be removed:
			for (auto& c : changes)
			{
				if (c.kind == EntityKind::Element && c.name.empty())
				{
					c.name = "element_" + std::to_string(nextRuntimeElementId_++);
					c.object->setName(c.name);
					auto lckObjs = mrpt::lockHelper(simulableObjectsMtx_);
					eraseByPointer(simulableObjects_, c.object.get());
					simulableObjects_.emplace(c.name, c.object);
				}
			}

			// Names must be unique among all entities:
			std::map<std::string, size_t> nameCount;
			for (const auto& [n, v] : vehicles_)
			{
				nameCount[n]++;
			}
			for (const auto& [n, b] : blocks_)
			{
				nameCount[n]++;
			}
			for (const auto& e : worldElements_)
			{
				if (!e->getName().empty())
				{
					nameCount[e->getName()]++;
				}
			}
			for (const auto& c : changes)
			{
				if (c.name.empty())
				{
					THROW_EXCEPTION("A new entity has no name");
				}
				if (nameCount[c.name] > 1)
				{
					THROW_EXCEPTION_FMT("Name already in use: '%s'", c.name.c_str());
				}
			}
		}
		catch (...)
		{
			userDefinedVariables_["MVSIM_CURRENT_FILE_DIRECTORY"] = prevFileDir;
			// Undo partial insertions:
			for (const auto& c : collectNew())
			{
				internalRemoveEntityNoLock(c.object);
			}
			throw;
		}

		if (!changes.empty())
		{
			internalResetPerObjectCollisionCaches();
		}
		for (const auto& c : changes)
		{
			if (c.kind == EntityKind::Element)
			{
				invalidateElevationIndex();
			}
#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF
			if (client_.connected())
			{
				c.object->registerOnServer(client_);
			}
#endif
		}
	}

	std::vector<std::string> names;
	for (const auto& c : changes)
	{
		names.push_back(c.name);
		internalNotifyEntityChange(c);
	}
	return names;
}

bool World::removeEntity(const std::string& name)
{
	std::shared_ptr<Simulable> obj;
	EntityKind kind = EntityKind::Block;
	{
		std::lock_guard<std::mutex> lckStep(simulationStepRunningMtx_);
		auto lckWorld = mrpt::lockHelper(world_cs_);
		auto lckPhys = mrpt::lockHelper(physical_objects_mtx());

		if (auto it = vehicles_.find(name); it != vehicles_.end())
		{
			obj = it->second;
			kind = EntityKind::Vehicle;
		}
		else if (auto itB = blocks_.find(name); itB != blocks_.end())
		{
			obj = itB->second;
			kind = EntityKind::Block;
		}
		else
		{
			for (const auto& e : worldElements_)
			{
				if (e->getName() == name)
				{
					obj = e;
					kind = EntityKind::Element;
					break;
				}
			}
		}
		if (!obj)
		{
			// Visual-only runtime objects use the same namespace of names:
			return runtimeObjects_.remove({name}) > 0;
		}
		internalRemoveEntityNoLock(obj);
	}

	EntityChange c;
	c.name = name;
	c.kind = kind;
	c.added = false;
	c.object = obj;
	internalNotifyEntityChange(c);
	return true;
}

void World::internalRemoveEntityNoLock(const std::shared_ptr<Simulable>& obj)
{
	ASSERT_(obj);
	const std::string name = obj->getName();

	eraseByPointer(vehicles_, obj.get());
	eraseByPointer(blocks_, obj.get());
	worldElements_.remove_if([&](const WorldElementBase::Ptr& e)
							 { return static_cast<Simulable*>(e.get()) == obj.get(); });
	{
		auto lckObjs = mrpt::lockHelper(simulableObjectsMtx_);
		eraseByPointer(simulableObjects_, obj.get());
	}
	{
		auto lckCopy = mrpt::lockHelper(copy_of_objects_dynstate_mtx_);
		copy_of_objects_dynstate_pose_.erase(name);
		copy_of_objects_dynstate_twist_.erase(name);
		copy_of_objects_had_collision_.erase(name);
	}

	internalResetPerObjectCollisionCaches();
	internalDestroyBox2DBodiesOf(*obj);
	invalidateElevationIndex();

	// 3D objects and OpenGL resources are released by the thread that
	// renders the scenes:
	auto lck = mrpt::lockHelper(removedEntitiesPendingGuiMtx_);
	removedEntitiesPendingGui_.push_back(obj);
}

void World::internalProcessRemovedEntitiesInGui()
{
	std::vector<std::shared_ptr<Simulable>> lst;
	{
		auto lck = mrpt::lockHelper(removedEntitiesPendingGuiMtx_);
		lst = std::move(removedEntitiesPendingGui_);
		removedEntitiesPendingGui_.clear();
	}
	for (const auto& obj : lst)
	{
		if (auto* vo = dynamic_cast<CVisualObject*>(obj.get()); vo)
		{
			vo->removeFromScenes();
		}
		obj->freeOpenGLResources();
		gui_.forget_entity(obj->getName());
	}
}
