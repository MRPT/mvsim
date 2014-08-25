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

#if 0
	// Create a "friction joint" for each wheel:
	b2FrictionJointDef fjd;

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

		b2FrictionJoint* b2_friction = dynamic_cast<b2FrictionJoint*>( m_world->getBox2DWorld()->CreateJoint( &fjd ) );
		m_joints.push_back(b2_friction);
	}
#endif
}

// See docs in base class.
void DefaultFriction::evaluate_friction(const FrictionBase::TFrictionInput &input, mrpt::math::TPoint2D &out_result_force_local) const
{
	b2Body *b2Veh = m_my_vehicle.getBox2DChassisBody();

	// Update the friction "joints":
	// ---------------------------------
#if 0
	for (size_t i=0;i<m_my_vehicle.getNumWheels();i++)
	{
		b2FrictionJoint* b2_friction = dynamic_cast<b2FrictionJoint*>( m_joints[i] );
		b2_friction->ShiftOrigin( b2Veh->GetWorldPoint( b2Vec2_zero ) );

		b2_friction->SetMaxForce(m_max_torque);
		b2_friction->SetMaxTorque(m_max_force);
	}
#endif

	// Rotate wheel velocity vector from veh. frame => wheel frame
	mrpt::math::TPoint2D vel_w( input.wheel_speed );


	// Action/Reaction, slippage, etc:
	// --------------------------------------
	const double mu = 0.65;
	const double max_friction = mu * input.weight;

	// 1) Lateral friction (decoupled sub-problem)
	// --------------------------------------------
	double wheel_lat_friction=0.0;  // direction: +y local wrt the wheel
	{
		// Impulse required to step the lateral slippage:
		wheel_lat_friction = -vel_w.y/ input.context.dt;

		wheel_lat_friction = b2Clamp(wheel_lat_friction, -max_friction,max_friction);
	}

	// 2) Longitudinal friction (decoupled sub-problem)
	// -------------------------------------------------
	double wheel_long_friction=0.0; // direction: +x local wrt the wheel
	
	// (eq. 1)==> desired impulse in wheel spinning speed.
	//double wheel_C_lon_vel = vel_w.x - input.wheel.w * 0.5*input.wheel.diameter;

	// It should be = 0 for no slippage (nonholonomic constraint): find out required wheel \omega:
	const double R = 0.5*input.wheel.diameter; // Wheel radius
	const double lon_constraint_desired_wheel_w = vel_w.x / R;
	const double desired_wheel_w_impulse = (lon_constraint_desired_wheel_w-input.wheel.w);
	const double desired_wheel_alpha = desired_wheel_w_impulse / input.context.dt;
	
	// (eq. 3)==> Find out F_r
	// Iyy_w * \Delta\omega_w = dt*\tau-  R*dt*Fri    -C_damp * \omega_w * dt
	// "Damping" / internal friction of the wheel's shaft, etc.
	const double C_damping = 1.0;
	//const mrpt::math::TPoint2D wheel_damping(- C_damping * input.wheel_speed.x, 0.0);

	const double I_yy = 0.1;
	double F_friction_lon = ( input.motor_torque - I_yy*desired_wheel_alpha - C_damping*input.wheel.w )/R;

	// Slippage: The friction with the ground is not infinite:
	F_friction_lon = b2Clamp(F_friction_lon, -max_friction,max_friction);
	
	// Recalc wheel ang. velocity impulse with this reduced force:
	const double actual_wheel_alpha = ( input.motor_torque - R * F_friction_lon - C_damping*input.wheel.w )/I_yy;
	
	// Apply impulse to wheel's spinning:
	input.wheel.w += actual_wheel_alpha * input.context.dt;

	// Slippage??

	wheel_long_friction = F_friction_lon;

	
	// Resultant force: In local (x,y) coordinates (Newtons) wrt the Wheel
	// -----------------------------------------------------------------------
	const mrpt::math::TPoint2D result_force_wrt_wheel(wheel_long_friction ,wheel_lat_friction);

	// Rotate to put in vehicle local framework:
	MRPT_TODO("rotate! (for ackermann,etc.)")
	const mrpt::math::TPoint2D result_force_wrt_veh = result_force_wrt_wheel;


	out_result_force_local = result_force_wrt_veh;
}


