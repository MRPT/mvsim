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
	FrictionBase(my_vehicle)
{
	// Sanity: we can tolerate node==NULL (=> means use default params).
	if (node && 0!=strcmp(node->name(),"friction"))
		throw std::runtime_error("<friction>...</friction> XML node was expected!!");

	if (node)
	{
		// Parse params:
		//std::map<std::string,TParamEntry> params;
		//XXX

		//// Parse XML params:
		//parse_xmlnode_children_as_param(*node,params);
	}	

}

// See docs in base class.
void DefaultFriction::evaluate_friction(const FrictionBase::TFrictionInput &input, mrpt::math::TPoint2D &out_result_force_local) const
{
	b2Body *b2Veh = m_my_vehicle.getBox2DChassisBody();

	// Action/Reaction, slippage, etc:
	// --------------------------------------
	// Contribution from motors:
	const mrpt::math::TPoint2D propulsion(-input.motor_force, 0.0);
		
	// "Damping" / internal friction of the wheel's shaft, etc.
	const double C_damping = 5.0;
	const mrpt::math::TPoint2D wheel_damping(- C_damping * input.wheel_speed.x, 0.0);
		
	// Lateral friction
	mrpt::math::TPoint2D wheel_lat_friction(0.0, 0.0);
	{
		const double clamp_norm = 3.0;
		const double f = std::max( std::min(input.wheel_speed.y,clamp_norm), -clamp_norm);
		const double mu = 0.65;
		wheel_lat_friction.y = -f * mu * input.weight * 9.81;
	}

	// Apply force: In local (x,y) coordinates (Newtons)
	// --------------------------------------
	out_result_force_local = propulsion; // + wheel_damping + wheel_lat_friction;
}


