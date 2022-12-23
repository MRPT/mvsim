/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/system/filesystem.h>	 // filePathSeparatorsToNative()
#include <mvsim/World.h>

#include <algorithm>  // count()
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
	if (m_gui_thread.joinable())
	{
		MRPT_LOG_DEBUG("Dtor: Waiting for GUI thread to quit...");
		gui_thread_must_close(true);
		m_gui_thread.join();
		MRPT_LOG_DEBUG("Dtor: GUI thread shut down successful.");
	}
	else
	{
		MRPT_LOG_DEBUG("Dtor: GUI thread already shut down.");
	}

	this->clear_all();
	m_box2d_world.reset();
}

// Resets the entire simulation environment to an empty world.
void World::clear_all()
{
	auto lck = mrpt::lockHelper(m_world_cs);

	// Reset params:
	force_set_simul_time(.0);

	// (B2D) World contents:
	// ---------------------------------------------
	m_box2d_world = std::make_unique<b2World>(b2Vec2_zero);

	// Define the ground body.
	b2BodyDef groundBodyDef;
	m_b2_ground_body = m_box2d_world->CreateBody(&groundBodyDef);

	// Clear lists of objs:
	// ---------------------------------------------
	m_vehicles.clear();
	m_world_elements.clear();
	m_blocks.clear();
}

void World::internal_initialize()
{
	worldVisual_->getViewport()->lightParameters().ambient = {
		0.5f, 0.5f, 0.5f, 1.0f};

	initialized_ = true;
}

/** Runs the simulation for a given time interval (in seconds) */
void World::run_simulation(double dt)
{
	ASSERT_(initialized_);

	const double t0 = mrpt::Clock::nowDouble();

	// Define start of simulation time:
	if (!m_simul_start_wallclock_time.has_value())
		m_simul_start_wallclock_time = t0 - dt;

	m_timlogger.registerUserMeasure("run_simulation.dt", dt);

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
		if (gui_thread_must_close()) break;
	}

	const double t1 = mrpt::Clock::toDouble(mrpt::Clock::now());

	m_timlogger.registerUserMeasure("run_simulation.cpu_dt", t1 - t0);
}

/** Runs one individual time step */
void World::internal_one_timestep(double dt)
{
	if (gui_thread_must_close()) return;

	std::lock_guard<std::mutex> lck(m_simulationStepRunningMtx);

	m_timer_iteration.Tic();

	TSimulContext context;
	context.world = this;
	context.b2_world = m_box2d_world.get();
	context.simul_time = get_simul_time();
	context.dt = dt;

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());

	// 1) Pre-step
	{
		mrpt::system::CTimeLoggerEntry tle(m_timlogger, "timestep.0.prestep");

		for (auto& e : m_simulableObjects)
			if (e.second) e.second->simul_pre_timestep(context);
	}

	// 2) Run dynamics
	{
		mrpt::system::CTimeLoggerEntry tle(
			m_timlogger, "timestep.1.dynamics_integrator");

		m_box2d_world->Step(dt, m_b2d_vel_iters, m_b2d_pos_iters);

		// Move time forward:
		auto lckSimTim = mrpt::lockHelper(m_simul_time_mtx);
		m_simul_time += dt;
	}

	// 3) Save dynamical state and post-step processing:
	// This also makes a copy of all objects dynamical state, so service calls
	// can be answered straight away without waiting for the main simulation
	// mutex:
	{
		mrpt::system::CTimeLoggerEntry tle(
			m_timlogger, "timestep.3.save_dynstate");

		const auto lckPhys = mrpt::lockHelper(physical_objects_mtx());
		const auto lckCopy = mrpt::lockHelper(m_copy_of_objects_dynstate_mtx);

		for (auto& e : m_simulableObjects)
		{
			if (!e.second) continue;
			// process:
			e.second->simul_post_timestep(context);

			// save our own copy of the kinematic state:
			m_copy_of_objects_dynstate_pose[e.first] = e.second->getPose();
			m_copy_of_objects_dynstate_twist[e.first] = e.second->getTwist();

			if (e.second->hadCollision())
				m_copy_of_objects_had_collision.insert(e.first);
		}
	}
	{
		const auto lckCollis = mrpt::lockHelper(m_reset_collision_flags_mtx);
		for (const auto& sId : m_reset_collision_flags)
		{
			if (auto itV = m_simulableObjects.find(sId);
				itV != m_simulableObjects.end())
				itV->second->resetCollisionFlag();
		}
		m_reset_collision_flags.clear();
	}
	lckListObjs.unlock();  // for m_simulableObjects

	// 4) Wait for 3D sensors (OpenGL raytrace) to get executed on its thread:
	mrpt::system::CTimeLoggerEntry tle4(
		m_timlogger, "timestep.4.wait_3D_sensors");
	if (pending_running_sensors_on_3D_scene())
	{
		for (int i = 0; i < 1000 && pending_running_sensors_on_3D_scene(); i++)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		if (pending_running_sensors_on_3D_scene())
		{
			MRPT_LOG_WARN(
				"Timeout waiting for async sensors to be simulated in opengl "
				"thread.");
			m_timlogger.registerUserMeasure("timestep.timeout_3D_sensors", 1.0);
		}
	}
	tle4.stop();

	const double ts = m_timer_iteration.Tac();
	m_timlogger.registerUserMeasure("timestep", ts);
	if (ts > dt) m_timlogger.registerUserMeasure("timestep.too_slow_alert", ts);
}

std::string World::xmlPathToActualPath(const std::string& modelURI) const
{
	std::string localFileName;
	if (modelURI.substr(0, 7) == "http://" ||
		modelURI.substr(0, 8) == "https://")
	{
		// MRPT_TODO("Retrieve models from online sources");
		THROW_EXCEPTION("To do: online models");
		// localFileName = xx;
	}
	else if (modelURI.substr(0, 7) == "file://")
	{
		localFileName = modelURI.substr(7);
	}
	else
		localFileName = modelURI;

	return resolvePath(localFileName);
}

/** Replace macros, prefix the base_path if input filename is relative, etc.
 */
std::string World::resolvePath(const std::string& s_in) const
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
	{
		ret = m_base_path;
		if (!ret.empty() && ret[ret.size() - 1] != '/' &&
			ret[ret.size() - 1] != '\\')
			ret += string("/");
		ret += s;
	}
	else
		ret = s;

	return mrpt::system::filePathSeparatorsToNative(ret);
}

void World::runVisitorOnVehicles(const vehicle_visitor_t& v)
{
	for (auto& veh : m_vehicles)
		if (veh.second) v(*veh.second);
}

void World::runVisitorOnWorldElements(const world_element_visitor_t& v)
{
	for (auto& we : m_world_elements)
		if (we) v(*we);
}

void World::runVisitorOnBlocks(const block_visitor_t& v)
{
	for (auto& b : m_blocks)
		if (b.second) v(*b.second);
}

void World::connectToServer()
{
	//
	m_client.setVerbosityLevel(this->getMinLoggingLevel());
	m_client.serverHostAddress(m_server_address);
	m_client.connect();

	// Let objects register topics / services:
	auto lckListObjs = mrpt::lockHelper(m_simulableObjectsMtx);

	for (auto& o : m_simulableObjects)
	{
		ASSERT_(o.second);
		o.second->registerOnServer(m_client);
	}
	lckListObjs.unlock();

	// global services:
	internal_advertiseServices();
}

void World::insertBlock(const Block::Ptr& block)
{
	// Assign each block an "index" number
	block->setBlockIndex(m_blocks.size());

	// make sure the name is not duplicated:
	m_blocks.insert(BlockList::value_type(block->getName(), block));

	auto lckListObjs = mrpt::lockHelper(m_simulableObjectsMtx);

	m_simulableObjects.insert(
		m_simulableObjects.end(),
		std::make_pair(
			block->getName(), std::dynamic_pointer_cast<Simulable>(block)));
}

double World::get_simul_timestep() const
{
	ASSERT_GE_(m_simul_timestep, .0);
	static bool firstTimeCheck = true;

	auto lambdaMinimumSensorPeriod = [this]() -> std::optional<double> {
		std::optional<double> ret;
		for (const auto& veh : m_vehicles)
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

	if (m_simul_timestep == 0)
	{
		// `0` means auto-determine as the minimum of 50 ms and the shortest
		// sensor sample period.
		m_simul_timestep = 50e-3;

		auto sensorMinPeriod = lambdaMinimumSensorPeriod();
		if (sensorMinPeriod) mrpt::keep_min(m_simul_timestep, *sensorMinPeriod);

		MRPT_LOG_INFO_FMT(
			"Physics simulation timestep automatically determined as: %.02f ms",
			1e3 * m_simul_timestep);

		firstTimeCheck = false;	 // no need to check.
	}
	else if (firstTimeCheck)
	{
		firstTimeCheck = false;
		auto sensorMinPeriod = lambdaMinimumSensorPeriod();
		if (sensorMinPeriod && m_simul_timestep > *sensorMinPeriod)
		{
			MRPT_LOG_WARN_FMT(
				"Physics simulation timestep (%.02f ms) should be >= than the "
				"minimum sensor period (%.02f ms) to avoid missing sensor "
				"readings!!",
				1e3 * m_simul_timestep, 1e3 * *sensorMinPeriod);
		}
	}

	return m_simul_timestep;
}

void World::free_opengl_resources()
{
	auto lck = mrpt::lockHelper(worldPhysicalMtx_);

	worldPhysical_.clear();
	worldVisual_->clear();

	VisualObject::FreeOpenGLResources();
}
