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
#include "xml_utils.h"

using namespace mv2dsim;


DefaultFriction::DefaultFriction(VehicleBase & my_vehicle, const rapidxml::xml_node<char> *node) :
	FrictionBase(my_vehicle)
{
	// Sanity: we can tolerate node==NULL (=> means use default params).
	if (node && 0!=strcmp(node->name(),"friction"))
		throw std::runtime_error("<friction>...</friction> XML node was expected!!");

	if (node)
	{
		// Parse params:
		m_max_torque = 0.0;
		m_max_force = 30;
		//std::map<std::string,TParamEntry> params;
		//XXX

		//// Parse XML params:
		//parse_xmlnode_children_as_param(*node,params);
	}

	// Create a "friction joint" for each wheel:
	b2FrictionWheelDef fjd;

	b2Body * veh = m_my_vehicle.getBox2DChassisBody();

	fjd.bodyA = veh;
	fjd.bodyB = m_world->getBox2DGroundBody();

	for (size_t i=0;i<m_my_vehicle.getNumWheels();i++)
	{
		const Wheel & ipw = m_my_vehicle.getWheelInfo(i);

		const b2Vec2 local_pt = b2Vec2( ipw.x,ipw.y );

		fjd.localWheelPt = local_pt;
		fjd.localWheelAngle = ipw.yaw;
		fjd.maxForce = m_max_torque;
		fjd.maxTorque = m_max_force;

		b2FrictionWheel* b2_friction = dynamic_cast<b2FrictionWheel*>( m_world->getBox2DWorld()->CreateJoint( &fjd ) );
		ASSERT_(b2_friction)
		m_joints.push_back(b2_friction);
	}

}

// See docs in base class.
void DefaultFriction::evaluate_friction(const FrictionBase::TFrictionInput &input, mrpt::math::TPoint2D &out_result_force_local) const
{
	b2Body *b2Veh = m_my_vehicle.getBox2DChassisBody();

	// Update the friction "joints":
	// ---------------------------------
	for (size_t i=0;i<m_my_vehicle.getNumWheels();i++)
	{
		b2FrictionWheel* b2_friction = dynamic_cast<b2FrictionWheel*>( m_joints[i] );
		ASSERT_(b2_friction)
		b2_friction->ShiftOrigin( b2Veh->GetWorldPoint( b2Vec2_zero ) );

		b2_friction->SetMaxForce(m_max_torque);
		b2_friction->SetMaxTorque(m_max_force);
	}

	// Action/Reaction, slippage, etc:
	// --------------------------------------
	const double mu = 0.65;
	const double max_friction = mu * input.weight;

	// Contribution from motors:
	const mrpt::math::TPoint2D propulsion(0,0); // (-input.motor_force, 0.0);
	// TODO: Clamp motor propulsion according to max_friction!!

	// "Damping" / internal friction of the wheel's shaft, etc.
	//const double C_damping = 5.0;
	//const mrpt::math::TPoint2D wheel_damping(- C_damping * input.wheel_speed.x, 0.0);

	//// Lateral friction
	//mrpt::math::TPoint2D wheel_lat_friction(0.0, 0.0);
	//{

	//	// Impulse required to step the lateral slippage:
	//	wheel_lat_friction.y = -input.wheel_speed.y / input.context.dt;

	//	wheel_lat_friction.y = b2Clamp(wheel_lat_friction.y, -max_friction,max_friction);
	//}

	// Apply force: In local (x,y) coordinates (Newtons)
	// --------------------------------------
	out_result_force_local = propulsion;
}


