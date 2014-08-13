/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/VehicleBase.h>
#include <mv2dsim/WorldElements/WorldElementBase.h>

#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <Box2D/Dynamics/b2World.h>
#include <list>

namespace mv2dsim
{
	/** Simulation happens inside a World object. 
	  * This is the central class for usage from user code, running the simulation, 
	  * loading XML models, managing GUI visualization, etc.
	  * The ROS node acts as a bridge between this class and the ROS subsystem.
	  */
	class World
	{
	public:
		/** \name Initialization, simulation set-up
		  @{*/
		World(); //!< Default ctor: inits an empty world
		~World(); //!< Dtor.

		void clear_all(bool acquire_mt_lock=true); //!< Resets the entire simulation environment to an empty world. \a acquire_mt_lock determines whether to lock the multithreading semaphore before (set to false only if it's already acquired).

		/** Load an entire world description into this object from a specification in XML format.
		  * \exception std::exception On any error, with what() giving a descriptive error message
		  */
		void load_from_XML(const std::string &xml_text); 
		/** @} */

		/** \name Simulation execution
		  @{*/
		
		double get_simul_timestep() const { return m_simul_timestep; } //!< Simulation fixed-time interval for numerical integration
		void   set_simul_timestep(double timestep) { m_simul_timestep=timestep; } //!< Simulation fixed-time interval for numerical integration
		
		/** Runs the simulation for a given time interval (in seconds) */
		void run_simulation(double dt);

		/** Updates (or sets-up upon first call) the GUI visualization of the scene. 
		  * \note This method is prepared to be called concurrently with the simulation, and doing so is recommended to assure a smooth multi-threading simulation.
		  */
		void update_GUI();

		/** @} */

	private:
		// -------- World Params ----------
		double m_simul_time;    //!< In seconds, real simulation time since beginning (may be different than wall-clock time because of time warp, etc.)
		double m_simul_timestep; //!< Simulation fixed-time interval for numerical integration.

		// -------- World contents ----------
		mrpt::synch::CCriticalSection m_world_cs; //!< The main semaphore to protect simulation objects from multithreading access.
		b2World  m_box2d_world; //!< Box2D dynamic simulator instance

		std::list<VehicleBase*> m_vehicles;
		std::list<WorldElementBase*> m_world_elements; 

		/** Runs one individual time step */
		void internal_one_timestep(double dt);


		// -------- GUI stuff ----------
		mrpt::gui::CDisplayWindow3DPtr  m_gui_win;

	};


}