/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/lock_helper.h>
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
}

void World::internal_initialize()
{
	ASSERT_(!initialized_);
	ASSERT_(worldVisual_);

	worldVisual_->getViewport()->lightParameters().ambient = lightOptions_.light_ambient;

	// Physical world light = visual world lights:
	worldPhysical_.getViewport()->lightParameters() =
		worldVisual_->getViewport()->lightParameters();

	// Create group for sensor viz:
	{
		auto glVizSensors = mrpt::opengl::CSetOfObjects::Create();
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
	block->setBlockIndex(blocks_.size());

	// make sure the name is not duplicated:
	blocks_.insert(BlockList::value_type(block->getName(), block));

	auto lckListObjs = mrpt::lockHelper(simulableObjectsMtx_);

	simulableObjects_.insert(
		simulableObjects_.end(),
		std::make_pair(block->getName(), std::dynamic_pointer_cast<Simulable>(block)));
}

void World::free_opengl_resources()
{
	auto lck = mrpt::lockHelper(worldPhysicalMtx_);

	worldPhysical_.clear();
	worldVisual_->clear();

	VisualObject::FreeOpenGLResources();
}

bool World::sensor_has_to_create_egl_context()
{
	// If we have a GUI, reuse that context:
	if (!headless())
	{
		return false;
	}

	// otherwise, just the first time:
	static bool first = true;
	bool ret = first;
	first = false;
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

	if (js.axes.size() > JOY_AXIS_AZIMUTH && gui_.gui_win)
	{
		auto lck = mrpt::lockHelper(gui_.gui_win->background_scene_mtx);
		auto& cam = gui_.gui_win->camera();
		cam.setAzimuthDegrees(cam.getAzimuthDegrees() - js.axes[JOY_AXIS_AZIMUTH]);
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

			rawlog_io_per_veh_[v.first] = std::make_shared<mrpt::io::CFileGZOutputStream>(fileName);
		}
	}

	// Store:
	auto arch = mrpt::serialization::archiveFrom(*rawlog_io_per_veh_.at(veh.getName()));
	arch << *obs;
}

std::set<float> World::getElevationsAt(const mrpt::math::TPoint2D& worldXY) const
{
	// Assumption: getListOfSimulableObjectsMtx() is already acquired by all possible call paths?
	std::set<float> ret;

	// Optimized search for potential objects that influence this query:
	// 1) world elements: assuming they are few, visit them all.
	for (const auto& obj : worldElements_)
	{
		const auto optZ = obj->getElevationAt(worldXY);
		if (optZ)
		{
			ret.insert(*optZ);
		}
	}

	// 2) blocks: by hashed 2D LUT.
	const World::LUTCache& lut = getLUTCacheOfObjects();
	const auto lutCoord = xy_to_lut_coords(worldXY);
	if (auto it = lut.find(lutCoord); it != lut.end())
	{
		for (const auto& obj : it->second)
		{
			if (!obj)
			{
				continue;
			}
			const auto optZ = obj->getElevationAt(worldXY);
			if (optZ)
			{
				ret.insert(*optZ);
			}
		}
	}

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
	const auto zs = getElevationsAt({pt.x, pt.y});

	float prevZ = .0f;
	for (float z : zs)
	{
		if (z > pt.z)
		{
			break;
		}
		prevZ = z;
	}
	return prevZ;
}
