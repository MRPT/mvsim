/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/World.h>
#include <mv2dsim/VehicleBase.h>
#include <mv2dsim/FrictionModels/LinearFriction.h>

#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>

using namespace mv2dsim;

LinearFriction::LinearFriction() :
	m_world(NULL),
	m_vehicle(NULL)
{
}

void LinearFriction::init(World* world, VehicleBase * my_vehicle)
{
	//b2FrictionJoint
//	for (size_t i=0;i<getNumWheels();i++)
//	{
//		const TInfoPerWheel & ipw = getWheelInfo(i);
//
//
//	}

}

void LinearFriction::update_step(const TSimulContext &context)
{
//	for (size_t i=0;i<getNumWheels();i++)
//	{
//		const TInfoPerWheel & ipw = getWheelInfo(i);
//
//
//	}

}


