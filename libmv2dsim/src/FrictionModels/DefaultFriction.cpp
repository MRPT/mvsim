/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/World.h>
#include <mv2dsim/VehicleBase.h>
#include <mv2dsim/FrictionModels/DefaultFriction.h>

#include <rapidxml.hpp>
#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>
#include "xml_utils.h"

using namespace mv2dsim;


DefaultFriction::DefaultFriction(VehicleBase & my_vehicle, const rapidxml::xml_node<char> *node) :
	FrictionBase(my_vehicle),
	m_max_torque(5), 
	m_max_force(2)
{
	// Sanity: we can tolerate node==NULL (=> means use default params).
	if (node && 0!=strcmp(node->name(),"friction"))
		throw std::runtime_error("<friction>...</friction> XML node was expected!!");

	if (node)
	{
		// Parse params:
		std::map<std::string,TParamEntry> params;
		params["max_torque"] = TParamEntry("%lf",&m_max_torque);
		params["max_force"] = TParamEntry("%lf",&m_max_force);

		// Parse XML params:
		parse_xmlnode_children_as_param(*node,params);
	}	

	// Create a "friction joint" for each wheel:
	b2FrictionJointDef fjd;

	b2Body * veh = m_my_vehicle.getBox2DChassisBody();

	fjd.bodyA = m_world->getBox2DGroundBody();
	fjd.bodyB = veh;

	for (size_t i=0;i<m_my_vehicle.getNumWheels();i++)
	{
		const Wheel & ipw = m_my_vehicle.getWheelInfo(i);

		const b2Vec2 local_pt = b2Vec2( ipw.x,ipw.y );

		fjd.localAnchorA = veh->GetWorldPoint( local_pt );
		fjd.localAnchorB = local_pt;
		fjd.maxForce = m_max_torque;
		fjd.maxTorque = m_max_force;

		b2FrictionJoint* b2_friction = dynamic_cast<b2FrictionJoint*>( m_world->getBox2DWorld()->CreateJoint( &fjd ) );
		m_joints.push_back(b2_friction);
	}
}

void DefaultFriction::update_step(const TSimulContext &context)
{
	// Update them:
	b2Body * veh = m_my_vehicle.getBox2DChassisBody();
	for (size_t i=0;i<m_my_vehicle.getNumWheels();i++)
	{
		b2FrictionJoint* b2_friction = dynamic_cast<b2FrictionJoint*>( m_joints[i] );
		b2_friction->ShiftOrigin( veh->GetWorldPoint( b2Vec2_zero ) );

		b2_friction->SetMaxForce(m_max_torque);
		b2_friction->SetMaxTorque(m_max_force);
	}

}


