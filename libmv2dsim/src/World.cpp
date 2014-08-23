/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#include <mv2dsim/World.h>

#include <mrpt/utils/utils_defs.h>  // mrpt::format()
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/system/filesystem.h> // filePathSeparatorsToNative()

#include <iostream> // for debugging
#include <algorithm> // count()
#include <stdexcept>
#include <map>

using namespace mv2dsim;
using namespace std;

MRPT_TODO("Create global obj for params registry; export them to ROS, etc.")

// Default ctor: inits empty world.
World::World() :
	m_simul_time(0.0),
	m_simul_timestep(0.010),
	m_b2d_vel_iters(6),
	m_b2d_pos_iters(3),
	m_base_path("."),
	m_box2d_world( NULL )
{
	this->clear_all();
}

// Dtor.
World::~World()
{
	this->clear_all();
	delete m_box2d_world; m_box2d_world=NULL;
}

// Resets the entire simulation environment to an empty world.
void World::clear_all(bool acquire_mt_lock)
{
	try
	{
		if (acquire_mt_lock) m_world_cs.enter();

		// Reset params:
		m_simul_time = 0.0;

		// (B2D) World contents:
		// ---------------------------------------------
		delete m_box2d_world;
		m_box2d_world = new b2World( b2Vec2_zero );

		// Define the ground body.
		b2BodyDef groundBodyDef;
		m_b2_ground_body = m_box2d_world->CreateBody(&groundBodyDef);

		// Clear m_vehicles & other lists of objs:
		// ---------------------------------------------
		for(TListVehicles::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it) delete *it;
		m_vehicles.clear();

		for(std::list<WorldElementBase*>::iterator it=m_world_elements.begin();it!=m_world_elements.end();++it) delete *it;
		m_world_elements.clear();


		if (acquire_mt_lock) m_world_cs.leave();
	}
	catch (std::exception &)
	{
		if (acquire_mt_lock) m_world_cs.leave();
		throw; // re-throw
	}
}

/** Runs the simulation for a given time interval (in seconds) */
void World::run_simulation(double dt)
{
	m_timlogger.registerUserMeasure("run_simulation.dt",dt);

	// sanity checks:
	ASSERT_(dt>0)
	ASSERT_(m_simul_timestep>0)

	// Run in time steps:
	const double end_time = m_simul_time+dt;
	const double timetol = 1e-6; // tolerance for rounding errors summing time steps
	while (m_simul_time<(end_time-timetol))
	{
		// Timestep: always "simul_step" for the sake of repeatibility
		internal_one_timestep(m_simul_timestep);
	}
}

/** Runs one individual time step */
void World::internal_one_timestep(double dt)
{
	mrpt::utils::CTimeLoggerEntry tlegl(m_timlogger,"timestep");

	TSimulContext context;
	context.b2_world   = m_box2d_world;
	context.simul_time = m_simul_time;

	// 1) Pre-step
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timlogger,"timestep.0.prestep.veh");
		for(TListVehicles::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it)
			(*it)->simul_pre_timestep(context);
	}
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timlogger,"timestep.0.prestep.sensors");
		for(TListVehicles::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it) {
			VehicleBase::TListSensors &sensors = (*it)->getSensors();
			for (VehicleBase::TListSensors::iterator itSen=sensors.begin();itSen!=sensors.end();++itSen) {
				(*itSen)->simul_pre_timestep(context);
			}
		}
	}
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timlogger,"timestep.0.prestep.world");
		for(std::list<WorldElementBase*>::iterator it=m_world_elements.begin();it!=m_world_elements.end();++it)
			(*it)->simul_pre_timestep(context);
	}


	// 2) Run dynamics
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timlogger,"timestep.1.dynamics_integrator");

		m_box2d_world->Step(dt, m_b2d_vel_iters, m_b2d_pos_iters);
		m_simul_time+= dt;  // Avance time
	}


	// 3) Save dynamical state into vehicles classes
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timlogger,"timestep.3.save_dynstate");

		context.simul_time = m_simul_time;
		for(TListVehicles::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it)
		{
			(*it)->simul_post_timestep_common(context);
			(*it)->simul_post_timestep(context);
		}
	}

	// 4) Post-step:
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timlogger,"timestep.0.poststep.veh");
		for(TListVehicles::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it)
			(*it)->simul_post_timestep(context);
	}
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timlogger,"timestep.0.poststep.sensors");
		for(TListVehicles::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it) {
			VehicleBase::TListSensors &sensors = (*it)->getSensors();
			for (VehicleBase::TListSensors::iterator itSen=sensors.begin();itSen!=sensors.end();++itSen) {
				(*itSen)->simul_post_timestep(context);
			}
		}
	}
	{
		mrpt::utils::CTimeLoggerEntry tle(m_timlogger,"timestep.0.poststep.world");
		for(std::list<WorldElementBase*>::iterator it=m_world_elements.begin();it!=m_world_elements.end();++it)
			(*it)->simul_post_timestep(context);
	}


	m_timlogger.leave("timestep");
}

/** Updates (or sets-up upon first call) the GUI visualization of the scene.
	* \note This method is prepared to be called concurrently with the simulation, and doing so is recommended to assure a smooth multi-threading simulation.
	*/
size_t ID_GLTEXT_CLOCK = 0;

void World::update_GUI()
{
	// First call?
	// -----------------------
	if (!m_gui_win)
	{
		m_timlogger.enter("update_GUI_init");

		m_gui_win = mrpt::gui::CDisplayWindow3D::Create("mv2dsim",800,600);
		m_gui_win->setCameraZoom(80);
		mrpt::opengl::COpenGLScenePtr gl_scene = m_gui_win->get3DSceneAndLock();

		gl_scene->insert( mrpt::opengl::CGridPlaneXY::Create() );

		m_gui_win->unlockAccess3DScene();
		m_timlogger.leave("update_GUI_init");
	}

	m_timlogger.enter("update_GUI"); // Don't count initialization, since that is a total outlier and lacks interest!

	m_timlogger.enter("update_GUI.1.get-lock");
	mrpt::opengl::COpenGLScenePtr gl_scene = m_gui_win->get3DSceneAndLock(); // ** LOCK **
	m_timlogger.leave("update_GUI.1.get-lock");

	// Update view of map elements
	// -----------------------------
	m_timlogger.enter("update_GUI.2.map-elements");

	for(std::list<WorldElementBase*>::iterator it=m_world_elements.begin();it!=m_world_elements.end();++it)
		(*it)->gui_update(*gl_scene);

	m_timlogger.leave("update_GUI.2.map-elements");

	// Update view of vehicles
	// -----------------------------
	m_timlogger.enter("update_GUI.3.vehicles");

	for(TListVehicles::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it)
		(*it)->gui_update(*gl_scene);

	m_timlogger.leave("update_GUI.3.vehicles");

	// Update view of sensors
	// -----------------------------
	m_timlogger.enter("update_GUI.4.sensors");
	m_timlogger.leave("update_GUI.4.sensors");

	// Other messages
	// -----------------------------
	m_timlogger.enter("update_GUI.5.text-msgs");

	m_gui_win->addTextMessage(2,2, mrpt::format("Time: %s", mrpt::system::formatTimeInterval(this->m_simul_time).c_str()), mrpt::utils::TColorf(1,1,1,0.5), "serif", 10, mrpt::opengl::NICE, ID_GLTEXT_CLOCK );

	m_timlogger.leave("update_GUI.5.text-msgs");

	// Force refresh view
	// -----------------------
	m_gui_win->unlockAccess3DScene(); // ** UNLOCK **
	m_gui_win->repaint();

	m_timlogger.leave("update_GUI");
}

/** Replace macros, prefix the base_path if input filename is relative, etc.
  */
std::string World::resolvePath(const std::string &s_in) const
{
	std::string ret;
	const std::string s = mrpt::system::trim(s_in);

	// Relative path? It's not if:
	// "X:\*", "/*"
	// -------------------
	bool is_relative = true;
	if (s.size()>2 && s[1]==':' && (s[2]=='/' || s[2]=='\\') ) is_relative=false;
	if (s.size()>0 && (s[0]=='/' || s[0]=='\\') ) is_relative=false;
	if (is_relative)
	{
		ret = m_base_path;
		if (!ret.empty() && ret.back()!='/' && ret.back()!='\\') 
			ret+= string("/");
		ret += s;
	}
	else
		ret = s;

	// Expand macros: (TODO)
	// -------------------

	return mrpt::system::filePathSeparatorsToNative(ret);
}
