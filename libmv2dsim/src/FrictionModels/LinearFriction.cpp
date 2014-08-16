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
	m_world   = world;
	m_vehicle = my_vehicle;

	b2FrictionJointDef fjd;

	b2Body * veh = my_vehicle->getBox2DChassisBody();

	fjd.bodyA = world->getBox2DGroundBody();
	fjd.bodyB = veh;

	for (size_t i=0;i<my_vehicle->getNumWheels();i++)
	{
		const VehicleBase::TInfoPerWheel & ipw = my_vehicle->getWheelInfo(i);

		const b2Vec2 local_pt = b2Vec2( ipw.x,ipw.y );

		fjd.localAnchorA = veh->GetWorldPoint( local_pt );
		fjd.localAnchorB = local_pt;
		fjd.maxForce = 0;
		fjd.maxTorque = 0;

		b2FrictionJoint* b2_friction = dynamic_cast<b2FrictionJoint*>( world->getBox2DWorld()->CreateJoint( &fjd ) );
		m_joints.push_back(b2_friction);
	}
}

void LinearFriction::update_step(const TSimulContext &context)
{
	// Update them:
	b2Body * veh = m_vehicle->getBox2DChassisBody();
	for (size_t i=0;i<m_vehicle->getNumWheels();i++)
	{
		b2FrictionJoint* b2_friction = dynamic_cast<b2FrictionJoint*>( m_joints[i] );
		b2_friction->ShiftOrigin( veh->GetWorldPoint( b2Vec2_zero ) );
		b2_friction->SetMaxForce(5);
		b2_friction->SetMaxTorque(2);
	}

}


