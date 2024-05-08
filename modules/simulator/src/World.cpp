/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/system/filesystem.h>	 // filePathSeparatorsToNative()
#include <mrpt/version.h>
#include <mvsim/World.h>

#include <algorithm>  // count()
#include <iostream>
#include <map>
#include <stdexcept>

using namespace mvsim;
using namespace std;

// Default ctor: inits empty world.
World::World() : mrpt::system::COutputLogger("mvsim::World")
{
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
}

void World::internal_initialize()
{
	ASSERT_(!initialized_);
	ASSERT_(worldVisual_);

#if MRPT_VERSION >= 0x270
	worldVisual_->getViewport()->lightParameters().ambient =
		lightOptions_.light_ambient;
#else
	worldVisual_->getViewport()->lightParameters().ambient = {
		lightOptions_.light_ambient, lightOptions_.light_ambient,
		lightOptions_.light_ambient, 1.0f};
#endif
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

/** Runs the simulation for a given time interval (in seconds) */
void World::run_simulation(double dt)
{
	ASSERT_(initialized_);

	const double t0 = mrpt::Clock::nowDouble();

	// Define start of simulation time:
	if (!simul_start_wallclock_time_.has_value())
		simul_start_wallclock_time_ = t0 - dt;

	timlogger_.registerUserMeasure("run_simulation.dt", dt);

	const double simulTimestep = get_simul_timestep();

	// sanity checks:
	ASSERT_(dt > 0);
	ASSERT_(simulTimestep > 0);

	// Run in time steps:
	const double end_time = get_simul_time() + dt;
	// tolerance for rounding errors summing time steps
	const double timetol = 1e-4;
	while (get_simul_time() < (end_time - timetol))
	{
		// Timestep: always "simul_step" for the sake of repeatibility,
		// except if requested to run a shorter step:
		const double remainingTime = end_time - get_simul_time();
		if (remainingTime <= 0) break;

		internal_one_timestep(
			remainingTime >= simulTimestep ? simulTimestep : remainingTime);

		// IMPORTANT: This must be inside the loop to allow breaking if we are
		// closing the app and simulatedTime is not ticking anymore.
		if (simulator_must_close()) break;
	}

	const double t1 = mrpt::Clock::toDouble(mrpt::Clock::now());

	timlogger_.registerUserMeasure("run_simulation.cpu_dt", t1 - t0);
}

/** Runs one individual time step */
void World::internal_one_timestep(double dt)
{
	if (simulator_must_close()) return;

	std::lock_guard<std::mutex> lck(simulationStepRunningMtx_);

	timer_iteration_.Tic();

	TSimulContext context;
	context.world = this;
	context.b2_world = box2d_world_.get();
	context.simul_time = get_simul_time();
	context.dt = dt;

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());

	// 1) Pre-step
	{
		mrpt::system::CTimeLoggerEntry tle(timlogger_, "timestep.0.prestep");

		for (auto& e : simulableObjects_)
			if (e.second) e.second->simul_pre_timestep(context);
	}

	// 2) Run dynamics
	{
		mrpt::system::CTimeLoggerEntry tle(
			timlogger_, "timestep.1.dynamics_integrator");

		box2d_world_->Step(dt, b2dVelIters_, b2dPosIters_);

		// Move time forward:
		auto lckSimTim = mrpt::lockHelper(simul_time_mtx_);
		simulTime_ += dt;
	}

	// 3) Save dynamical state and post-step processing:
	// This also makes a copy of all objects dynamical state, so service calls
	// can be answered straight away without waiting for the main simulation
	// mutex:
	{
		mrpt::system::CTimeLoggerEntry tle(
			timlogger_, "timestep.3.save_dynstate");

		const auto lckPhys = mrpt::lockHelper(physical_objects_mtx());
		const auto lckCopy = mrpt::lockHelper(copy_of_objects_dynstate_mtx_);

		for (auto& e : simulableObjects_)
		{
			if (!e.second) continue;
			// process:
			e.second->simul_post_timestep(context);

			// save our own copy of the kinematic state:
			copy_of_objects_dynstate_pose_[e.first] = e.second->getPose();
			copy_of_objects_dynstate_twist_[e.first] = e.second->getTwist();

			if (e.second->hadCollision())
				copy_of_objects_had_collision_.insert(e.first);
		}
	}
	{
		const auto lckCollis = mrpt::lockHelper(reset_collision_flags_mtx_);
		for (const auto& sId : reset_collision_flags_)
		{
			if (auto itV = simulableObjects_.find(sId);
				itV != simulableObjects_.end())
				itV->second->resetCollisionFlag();
		}
		reset_collision_flags_.clear();
	}
	lckListObjs.unlock();  // for simulableObjects_

	// 4) Wait for 3D sensors (OpenGL raytrace) to get executed on its thread:
	mrpt::system::CTimeLoggerEntry tle4(
		timlogger_, "timestep.4.wait_3D_sensors");
	if (pending_running_sensors_on_3D_scene())
	{
		// Use a huge timeout here to avoid timing out in build farms / cloud
		// containers:
		for (int i = 0; i < 20000 && pending_running_sensors_on_3D_scene(); i++)
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

		if (pending_running_sensors_on_3D_scene())
		{
			MRPT_LOG_WARN(
				"Timeout waiting for async sensors to be simulated in opengl "
				"thread.");
			timlogger_.registerUserMeasure("timestep.timeout_3D_sensors", 1.0);
		}
	}
	tle4.stop();

	// 5) If we have .rawlog generation enabled, process odometry, etc.
	mrpt::system::CTimeLoggerEntry tle5(timlogger_, "timestep.5.post_rawlog");

	internalPostSimulStepForRawlog();
	internalPostSimulStepForTrajectory();

	tle5.stop();

	const double ts = timer_iteration_.Tac();
	timlogger_.registerUserMeasure("timestep", ts);
	if (ts > dt) timlogger_.registerUserMeasure("timestep.too_slow_alert", ts);
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
		is_relative = false;
	if (s.size() > 0 && (s[0] == '/' || s[0] == '\\')) is_relative = false;
	if (is_relative)
		ret = mrpt::system::pathJoin({basePath_, s});
	else
		ret = s;

	return mrpt::system::toAbsolutePath(ret);
}

void World::runVisitorOnVehicles(const vehicle_visitor_t& v)
{
	for (auto& veh : vehicles_)
		if (veh.second) v(*veh.second);
}

void World::runVisitorOnWorldElements(const world_element_visitor_t& v)
{
	for (auto& we : worldElements_)
		if (we) v(*we);
}

void World::runVisitorOnBlocks(const block_visitor_t& v)
{
	for (auto& b : blocks_)
		if (b.second) v(*b.second);
}

void World::connectToServer()
{
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
		std::make_pair(
			block->getName(), std::dynamic_pointer_cast<Simulable>(block)));
}

double World::get_simul_timestep() const
{
	ASSERT_GE_(simulTimestep_, .0);
	static bool firstTimeCheck = true;

	auto lambdaMinimumSensorPeriod = [this]() -> std::optional<double>
	{
		std::optional<double> ret;
		for (const auto& veh : vehicles_)
		{
			if (!veh.second) continue;
			for (const auto& s : veh.second->getSensors())
			{
				const double T = s->sensor_period();
				if (ret)
					mrpt::keep_min(*ret, T);
				else
					ret = T;
			}
		}
		return ret;
	};

	if (simulTimestep_ == 0)
	{
		// `0` means auto-determine as the minimum of 5 ms and the shortest
		// sensor sample period.
		simulTimestep_ = 5e-3;

		auto sensorMinPeriod = lambdaMinimumSensorPeriod();
		if (sensorMinPeriod) mrpt::keep_min(simulTimestep_, *sensorMinPeriod);

		MRPT_LOG_INFO_FMT(
			"Physics simulation timestep automatically determined as: %.02f ms",
			1e3 * simulTimestep_);

		firstTimeCheck = false;	 // no need to check.
	}
	else if (firstTimeCheck)
	{
		firstTimeCheck = false;
		auto sensorMinPeriod = lambdaMinimumSensorPeriod();
		if (sensorMinPeriod && simulTimestep_ > *sensorMinPeriod)
		{
			MRPT_LOG_WARN_FMT(
				"Physics simulation timestep (%.02f ms) should be >= than the "
				"minimum sensor period (%.02f ms) to avoid missing sensor "
				"readings!!",
				1e3 * simulTimestep_, 1e3 * *sensorMinPeriod);
		}
	}

	return simulTimestep_;
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
	if (!headless()) return false;

	// otherwise, just the first time:
	static bool first = true;
	bool ret = first;
	first = false;
	return ret;
}

std::optional<mvsim::TJoyStickEvent> World::getJoystickState() const
{
	if (!joystickEnabled_) return {};

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
		cam.setAzimuthDegrees(
			cam.getAzimuthDegrees() - js.axes[JOY_AXIS_AZIMUTH]);
	}

	return js;
}

void World::dispatchOnObservation(
	const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs)
{
	internalOnObservation(veh, obs);
	for (const auto& cb : callbacksOnObservation_) cb(veh, obs);
}

void World::internalOnObservation(
	const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs)
{
	using namespace std::string_literals;

	// Save to .rawlog, if enabled:
	if (save_to_rawlog_.empty() || vehicles_.empty()) return;

	auto lck = mrpt::lockHelper(rawlog_io_mtx_);
	if (rawlog_io_per_veh_.empty())
	{
		for (const auto& v : vehicles_)
		{
			const std::string fileName = mrpt::system::fileNameChangeExtension(
				save_to_rawlog_, v.first + ".rawlog"s);

			MRPT_LOG_INFO_STREAM("Creating dataset file: " << fileName);

			rawlog_io_per_veh_[v.first] =
				std::make_shared<mrpt::io::CFileGZOutputStream>(fileName);
		}
	}

	// Store:
	auto arch =
		mrpt::serialization::archiveFrom(*rawlog_io_per_veh_.at(veh.getName()));
	arch << *obs;
}

void World::internalPostSimulStepForRawlog()
{
	if (save_to_rawlog_.empty()) return;

	ASSERT_GT_(rawlog_odometry_rate_, 0.0);

	const double now = get_simul_time();
	const double T_odom = 1.0 / rawlog_odometry_rate_;

	if (rawlog_last_odom_time_.has_value() &&
		(now - *rawlog_last_odom_time_) < T_odom)
	{
		return;	 // not yet
	}

	rawlog_last_odom_time_ = now;

	// Create one observation per robot:
	for (const auto& veh : vehicles_)
	{
		auto obs = mrpt::obs::CObservationOdometry::Create();
		obs->timestamp = get_simul_timestamp();
		obs->sensorLabel = "odom";
		obs->odometry = veh.second->getCPose2D();

		obs->hasVelocities = true;
		obs->velocityLocal = veh.second->getVelocityLocal();

		// TODO: Simul noisy odometry and odom velocity

		internalOnObservation(*veh.second, obs);
	}
}

void World::internalPostSimulStepForTrajectory()
{
	using namespace std::string_literals;

	if (save_ground_truth_trajectory_.empty()) return;

	ASSERT_GT_(ground_truth_rate_, 0.0);

	const double now = get_simul_time();
	const double T_odom = 1.0 / ground_truth_rate_;

	if (gt_last_time_.has_value() && (now - *gt_last_time_) < T_odom)
	{
		return;	 // not yet
	}

	gt_last_time_ = now;

	// Create one entry per robot.
	// First, create the output files if this is the first time:
	auto lck = mrpt::lockHelper(gt_io_mtx_);
	if (gt_io_per_veh_.empty())
	{
		for (const auto& [vehName, veh] : vehicles_)
		{
			const std::string fileName = mrpt::system::fileNameChangeExtension(
				save_ground_truth_trajectory_, vehName + ".txt"s);

			MRPT_LOG_INFO_STREAM("Creating ground truth file: " << fileName);

			gt_io_per_veh_[vehName] = std::fstream(fileName, std::ios::out);
		}
	}

	const double t = mrpt::Clock::toDouble(get_simul_timestamp());

	for (const auto& [vehName, veh] : vehicles_)
	{
		const auto p = mrpt::poses::CPose3DQuat(veh->getCPose3D());

		// each row contains these elements separated by spaces:
		// timestamp x y z q_x q_y q_z q_w

		gt_io_per_veh_.at(vehName) << mrpt::format(
			"%f %f %f %f %f %f %f %f\n", t, p.x(), p.y(), p.z(), p.quat().x(),
			p.quat().y(), p.quat().z(), p.quat().w());
	}
}
