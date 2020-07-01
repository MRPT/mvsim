/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/filesystem.h>  // filePathSeparatorsToNative()
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
		MRPT_LOG_DEBUG("Waiting for GUI thread to quit...");
		m_gui_thread_must_close = true;
		m_gui_thread.join();
		MRPT_LOG_DEBUG("GUI thread shut down successful.");
	}

	this->clear_all();
	m_box2d_world.reset();
}

// Resets the entire simulation environment to an empty world.
void World::clear_all()
{
	auto lck = mrpt::lockHelper(m_world_cs);

	// Reset params:
	m_simul_time = 0.0;

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

/** Runs the simulation for a given time interval (in seconds) */
void World::run_simulation(double dt)
{
	m_timlogger.registerUserMeasure("run_simulation.dt", dt);

	// sanity checks:
	ASSERT_(dt > 0);
	ASSERT_(m_simul_timestep > 0);

	// Run in time steps:
	const double end_time = m_simul_time + dt;
	const double timetol =
		1e-6;  // tolerance for rounding errors summing time steps
	while (m_simul_time < (end_time - timetol))
	{
		// Timestep: always "simul_step" for the sake of repeatibility
		internal_one_timestep(m_simul_timestep);
	}
}

/** Runs one individual time step */
void World::internal_one_timestep(double dt)
{
	m_timer_iteration.Tic();

	TSimulContext context;
	context.world = this;
	context.b2_world = m_box2d_world.get();
	context.simul_time = m_simul_time;
	context.dt = dt;

	// 1) Pre-step
	{
		mrpt::system::CTimeLoggerEntry tle(m_timlogger, "timestep.0.prestep");

		for (auto& e : m_simulableObjects)
			if (e) e->simul_pre_timestep(context);
	}

	// 2) Run dynamics
	{
		mrpt::system::CTimeLoggerEntry tle(
			m_timlogger, "timestep.1.dynamics_integrator");

		m_box2d_world->Step(dt, m_b2d_vel_iters, m_b2d_pos_iters);
		m_simul_time += dt;  // Avance time
	}

	// 3) Save dynamical state and post-step processing:
	{
		mrpt::system::CTimeLoggerEntry tle(
			m_timlogger, "timestep.3.save_dynstate");

		for (auto& e : m_simulableObjects)
			if (e) e->simul_post_timestep(context);
	}

	const double ts = m_timer_iteration.Tac();
	m_timlogger.registerUserMeasure(
		(ts > dt ? "timestep_too_slow_alert" : "timestep"), ts);
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

	// Expand macros: (TODO)
	// -------------------

	return mrpt::system::filePathSeparatorsToNative(ret);
}

/** Run the user-provided visitor on each vehicle */
void World::runVisitorOnVehicles(const vehicle_visitor_t& v)
{
	for (auto& veh : m_vehicles)
		if (veh.second) v(*veh.second);
}

/** Run the user-provided visitor on each world element */
void World::runVisitorOnWorldElements(const world_element_visitor_t& v)
{
	for (auto& we : m_world_elements)
		if (we) v(*we);
}

void World::connectToServer()
{
	//
	MRPT_TODO("Allow changing parameters... from xml?");
	m_client.setVerbosityLevel(this->getMinLoggingLevel());
	m_client.connect();
}
