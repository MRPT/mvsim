/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/World.h>

using namespace mv2dsim;
using namespace std;

// Default ctor: inits empty world.
World::World() : 
	m_box2d_world( b2Vec2_zero )
{
	this->clear_all();
}

// Dtor.
World::~World()
{
	this->clear_all();
}

// Resets the entire simulation environment to an empty world.
void World::clear_all()
{
	mrpt::synch::CCriticalSectionLocker csl( &m_world_cs ); // Protect multithread access

	// Clear m_vehicles:
	for(std::list<VehicleBase*>::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it)
		delete *it;
	m_vehicles.clear();
}


