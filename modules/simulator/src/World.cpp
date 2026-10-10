/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/lock_helper.h>
#include <mrpt/imgui/CImGuiSceneView.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>	 // filePathSeparatorsToNative()
#include <mrpt/version.h>
#include <mvsim/World.h>

#include <map>

using namespace mvsim;
using namespace std;

// Default ctor: inits empty world.
World::World() : mrpt::system::COutputLogger("mvsim::World")
{  //
	this->clear_all();
}

// Dtor.
World::~World()
{
	if (gui_thread_.joinable())
	{
		MRPT_LOG_DEBUG("Dtor: Waiting for GUI thread to quit...");
		simulator_must_close(true);
		gui_thread_.join();
		MRPT_LOG_DEBUG("Dtor: GUI thread shut down successful.");
	}
	else
	{
		MRPT_LOG_DEBUG("Dtor: GUI thread already shut down.");
	}

	this->clear_all();
	box2d_world_.reset();
}

// Resets the entire simulation environment to an empty world.
void World::clear_all()
{
	auto lck = mrpt::lockHelper(world_cs_);

	// Reset params:
	force_set_simul_time(.0);

	// (B2D) World contents:
	// ---------------------------------------------
	box2d_world_ = std::make_unique<b2World>(b2Vec2_zero);

	// Define the ground body.
	b2BodyDef groundBodyDef;
	b2_ground_body_ = box2d_world_->CreateBody(&groundBodyDef);

	// Clear lists of objs:
	// ---------------------------------------------
	vehicles_.clear();
	worldElements_.clear();
	blocks_.clear();
	joints_.clear();
	actors_.clear();
	obstacles_for_each_obj_.clear();  // their bodies belonged to the old b2World
	nextVehicleIndex_ = 0;
	nextBlockIndex_ = 0;
	nextRuntimeElementId_ = 0;
}

void World::internal_initialize()
{
	ASSERT_(!initialized_);
	ASSERT_(worldVisual_);

	// Lights of both the visual and the physical worlds (the latter is the one
	// seen by sensors, with or without GUI):
	applyLightOptions();

	// Create group for sensor viz:
	{
		auto glVizSensors = mrpt::viz::CSetOfObjects::Create();
		glVizSensors->setName("group_sensors_viz");
		glVizSensors->setVisibility(guiOptions_.show_sensor_points);
		worldVisual_->insert(glVizSensors);
	}

	getTimeLogger().setMinLoggingLevel(this->getMinLoggingLevel());
	remoteResources_.setMinLoggingLevel(this->getMinLoggingLevel());

	initialized_ = true;
}

std::string World::xmlPathToActualPath(const std::string& modelURI) const
{
	std::string actualFileName = remoteResources_.resolve_path(modelURI);
	return local_to_abs_path(actualFileName);
}

/** Replace macros, prefix the base_path if input filename is relative, etc.
 */
std::string World::local_to_abs_path(const std::string& s_in) const
{
	std::string ret;
	const std::string s = mrpt::system::trim(s_in);

	// Relative path? It's not if:
	// "X:\*", "/*"
	// -------------------
	bool is_relative = true;
	if (s.size() > 2 && s[1] == ':' && (s[2] == '/' || s[2] == '\\'))
	{
		is_relative = false;
	}
	if (s.size() > 0 && (s[0] == '/' || s[0] == '\\'))
	{
		is_relative = false;
	}
	if (is_relative)
	{
		ret = mrpt::system::pathJoin({basePath_, s});
	}
	else
	{
		ret = s;
	}

	return mrpt::system::toAbsolutePath(ret);
}

void World::runVisitorOnVehicles(const vehicle_visitor_t& v)
{
	for (auto& veh : vehicles_)
	{
		if (veh.second)
		{
			v(*veh.second);
		}
	}
}

void World::runVisitorOnWorldElements(const world_element_visitor_t& v)
{
	for (auto& we : worldElements_)
		if (we) v(*we);
}

void World::runVisitorOnBlocks(const block_visitor_t& v)
{
	for (auto& b : blocks_)
	{
		if (b.second)
		{
			v(*b.second);
		}
	}
}

void World::connectToServer()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	//
	client_.setVerbosityLevel(this->getMinLoggingLevel());
	client_.serverHostAddress(serverAddress_);
	client_.connect();

	// Let objects register topics / services:
	auto lckListObjs = mrpt::lockHelper(simulableObjectsMtx_);

	for (auto& o : simulableObjects_)
	{
		ASSERT_(o.second);
		o.second->registerOnServer(client_);
	}
	lckListObjs.unlock();

	// global services:
	internal_advertiseServices();
#endif
}

void World::insertBlock(const Block::Ptr& block)
{
	// Assign each block an "index" number
	block->setBlockIndex(nextBlockIndex_++);

	// make sure the name is not duplicated:
	blocks_.insert(BlockList::value_type(block->getName(), block));

	auto lckListObjs = mrpt::lockHelper(simulableObjectsMtx_);

	simulableObjects_.insert(
		simulableObjects_.end(),
		std::make_pair(block->getName(), std::dynamic_pointer_cast<Simulable>(block)));
}

void World::free_opengl_resources()
{
	// The GUI thread renders these scenes: stop it first. It frees its own
	// OpenGL resources before exiting.
	if (gui_thread_.joinable() && gui_thread_.get_id() != std::this_thread::get_id())
	{
		simulator_must_close(true);
		gui_thread_.join();
	}

	auto lck = mrpt::lockHelper(worldPhysicalMtx_);

	worldPhysical_.clear();
	worldVisual_->clear();

	CVisualObject::FreeOpenGLResources();
}

std::optional<double> World::next_opengl_sensor_time() const
{
	std::optional<double> t;
	for (const auto& v : vehicles_)
	{
		for (const auto& s : v.second->getSensors())
		{
			if (s && s->rendersWithOpenGL())
			{
				const double ts = s->next_sensor_time();
				t = t.has_value() ? std::min(*t, ts) : ts;
			}
		}
	}
	return t;
}

bool World::sensor_has_to_create_egl_context()
{
	// If we have a GUI, reuse that context:
	if (!headless())
	{
		return false;
	}

	// otherwise, just the first time for this world (each world renders its
	// sensors from its own thread):
	const bool ret = !eglContextCreated_;
	eglContextCreated_ = true;
	return ret;
}

std::optional<mvsim::TJoyStickEvent> World::getJoystickState() const
{
	if (!joystickEnabled_)
	{
		return {};
	}

	if (!joystick_)
	{
		joystick_.emplace();
		const auto nJoy = joystick_->getJoysticksCount();
		if (!nJoy)
		{
			MRPT_LOG_WARN(
				"[World::getJoystickState()] No Joystick found, disabling "
				"joystick-based controllers.");
			joystickEnabled_ = false;
			joystick_.reset();
			return {};
		}
	}

	const int nJoy = 0;	 // TODO: Expose param for multiple joysticks?
	mvsim::Joystick::State joyState;

	joystick_->getJoystickPosition(nJoy, joyState);

	mvsim::TJoyStickEvent js;
	js.axes = joyState.axes;
	js.buttons = joyState.buttons;

	const size_t JOY_AXIS_AZIMUTH = 3;

	if (js.axes.size() > JOY_AXIS_AZIMUTH && is_GUI_open())
	{
		const float dAzimuth = js.axes[JOY_AXIS_AZIMUTH];
		enqueue_task_to_run_in_gui_thread(
			[this, dAzimuth]()
			{
				if (!gui_.sceneView)
				{
					return;
				}
				auto& cam = gui_.sceneView->cameraController;
				cam.setAzimuthDegrees(cam.getAzimuthDegrees() - dAzimuth);
			});
	}

	return js;
}

void World::dispatchOnObservation(const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs)
{
	internalOnObservation(veh, obs);
	for (const auto& cb : callbacksOnObservation_) cb(veh, obs);
}

void World::internalOnObservation(const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs)
{
	using namespace std::string_literals;

	// Save to .rawlog, if enabled:
	if (save_to_rawlog_.empty() || vehicles_.empty())
	{
		return;
	}
	auto lck = mrpt::lockHelper(rawlog_io_mtx_);
	if (rawlog_io_per_veh_.empty())
	{
		for (const auto& v : vehicles_)
		{
			const std::string fileName =
				mrpt::system::fileNameChangeExtension(save_to_rawlog_, v.first + ".rawlog"s);

			MRPT_LOG_INFO_STREAM("Creating dataset file: " << fileName);

#if MRPT_VERSION >= 0x020f07
			rawlog_io_per_veh_[v.first] =
				std::make_shared<mrpt::io::CCompressedOutputStream>(fileName);
#else
			rawlog_io_per_veh_[v.first] = std::make_shared<mrpt::io::CFileGZOutputStream>(fileName);
#endif
		}
	}

	// Store:
	auto arch = mrpt::serialization::archiveFrom(*rawlog_io_per_veh_.at(veh.getName()));
	arch << *obs;
}

void World::internal_update_elevation_index() const
{
	// Marked first, so a change during the rebuild invalidates it again:
	elevationIndexIsUpToDate_ = true;

	elevationIndex_.clear();
	elevationIndexUnbounded_.clear();
	for (const auto& obj : worldElements_)
	{
		const auto bb = obj->elevationBoundingBox();
		if (!bb)
		{
			elevationIndexUnbounded_.push_back(obj.get());
			continue;
		}
		const auto c0 = xy_to_lut_coords(mrpt::math::TPoint2Df(bb->min.x, bb->min.y));
		const auto c1 = xy_to_lut_coords(mrpt::math::TPoint2Df(bb->max.x, bb->max.y));
		const auto nCells =
			static_cast<std::size_t>(c1.x - c0.x + 1) * static_cast<std::size_t>(c1.y - c0.y + 1);
		if (nCells > MAX_LUT_CELLS_PER_OBJECT)
		{
			elevationIndexUnbounded_.push_back(obj.get());
			continue;
		}
		for (int32_t cx = c0.x; cx <= c1.x; cx++)
		{
			for (int32_t cy = c0.y; cy <= c1.y; cy++)
			{
				elevationIndex_[{cx, cy}].push_back(obj.get());
			}
		}
	}
}

template <typename Functor>
void World::forEachElevationAt(const mrpt::math::TPoint2D& worldXY, const Functor& f) const
{
	// Assumption: getListOfSimulableObjectsMtx() is already acquired by all possible call paths?
	const auto lutCoord = xy_to_lut_coords(mrpt::math::TPoint2Df(worldXY.x, worldXY.y));

	// 1) world elements: by their 2D spatial index.
	if (!elevationIndexIsUpToDate_)
	{
		std::unique_lock lck(elevationIndexMtx_);
		if (!elevationIndexIsUpToDate_)
		{
			internal_update_elevation_index();
		}
	}
	{
		std::shared_lock lck(elevationIndexMtx_);
		for (const auto* obj : elevationIndexUnbounded_)
		{
			if (const auto optZ = obj->getElevationAt(worldXY); optZ)
			{
				f(*optZ);
			}
		}
		if (auto it = elevationIndex_.find(lutCoord); it != elevationIndex_.end())
		{
			for (const auto* obj : it->second)
			{
				if (const auto optZ = obj->getElevationAt(worldXY); optZ)
				{
					f(*optZ);
				}
			}
		}
	}

	// 2) blocks: by hashed 2D LUT, plus those too large for it.
	const auto visitBlocks = [&](const std::vector<Simulable::Ptr>& objs)
	{
		for (const auto& obj : objs)
		{
			if (!obj)
			{
				continue;
			}
			if (const auto optZ = obj->getElevationAt(worldXY); optZ)
			{
				f(*optZ);
			}
		}
	};
	const World::LUTCache& lut = getLUTCacheOfObjects();
	visitBlocks(lut2d_oversized_objects_);
	if (auto it = lut.find(lutCoord); it != lut.end())
	{
		visitBlocks(it->second);
	}
}

std::set<float> World::getElevationsAt(const mrpt::math::TPoint2D& worldXY) const
{
	std::set<float> ret;
	forEachElevationAt(worldXY, [&ret](float z) { ret.insert(z); });

	// if none:
	if (ret.empty())
	{
		ret.insert(.0f);
	}
	return ret;
}

std::optional<std::any> World::getPropertyAt(
	const std::string& propertyName, const mrpt::math::TPoint3D& worldXYZ) const
{
	// 1) world elements: visit all
	for (const auto& obj : worldElements_)
	{
		const auto optProp = obj->queryProperty(propertyName, worldXYZ);
		if (optProp)
		{
			return optProp;
		}
	}
	return {};
}

float World::getHighestElevationUnder(const mrpt::math::TPoint3Df& pt) const
{
	// The highest elevation not above the query point, or 0 if none:
	std::optional<float> highest;
	forEachElevationAt(
		{pt.x, pt.y},
		[&](float z)
		{
			if (z <= pt.z && (!highest || z > *highest))
			{
				highest = z;
			}
		});
	return highest.value_or(.0f);
}

namespace
{
CVisualObject* findVisualObject(const World::SimulableList& objs, const std::string& name)
{
	const auto it = objs.find(name);
	if (it == objs.end())
	{
		return nullptr;
	}
	return dynamic_cast<CVisualObject*>(it->second.get());
}
}  // namespace

bool World::setLightGroupState(const std::string& objectName, const std::string& groupName, bool on)
{
	auto lck = mrpt::lockHelper(simulableObjectsMtx_);
	auto* obj = findVisualObject(simulableObjects_, objectName);
	return obj && obj->setLightGroupState(groupName, on);
}

std::optional<bool> World::lightGroupState(
	const std::string& objectName, const std::string& groupName) const
{
	auto lck = mrpt::lockHelper(simulableObjectsMtx_);
	const auto* obj = findVisualObject(simulableObjects_, objectName);
	if (!obj)
	{
		return {};
	}
	return obj->lightGroupState(groupName);
}

World::GroundTruthSnapshot World::getGroundTruthSnapshot(const std::string& prefix) const
{
	// Skip unnamed and internal ("__"-prefixed) objects:
	const auto startsWithPrefix = [&prefix](const std::string& name)
	{
		return !name.empty() && name.compare(0, 2, "__") != 0 &&
			   name.compare(0, prefix.size(), prefix) == 0;
	};

	GroundTruthSnapshot snap;
	{
		auto lckCopy = mrpt::lockHelper(copy_of_objects_dynstate_mtx_);
		snap.simul_time = copy_of_objects_dynstate_time_;
		for (const auto& [name, pose] : copy_of_objects_dynstate_pose_)
		{
			if (!startsWithPrefix(name))
			{
				continue;
			}
			ObjectGroundTruth o;
			o.name = name;
			o.pose = pose;
			if (auto it = copy_of_objects_dynstate_twist_.find(name);
				it != copy_of_objects_dynstate_twist_.end())
			{
				o.twist = it->second;
			}
			snap.objects.push_back(std::move(o));
		}
	}
	for (const auto& [name, pose] : runtimeObjects_.poses())
	{
		if (startsWithPrefix(name))
		{
			snap.objects.push_back({name, pose, {}});
		}
	}
	return snap;
}
