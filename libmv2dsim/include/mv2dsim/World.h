/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mrpt/synch/CCriticalSection.h>
#include <Box2D/Dynamics/b2World.h>

#include <mv2dsim/VehicleBase.h>

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
		World(); //!< Default ctor: inits an empty world
		~World(); //!< Dtor.

		void clear_all(); //!< Resets the entire simulation environment to an empty world.

	private:
		mrpt::synch::CCriticalSection m_world_cs; //!< The main semaphore to protect simulation objects from multithreading access.
		b2World  m_box2d_world; //!< Box2D dynamic simulator instance

		std::list<VehicleBase*> m_vehicles;

	};


}