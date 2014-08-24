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
void DefaultFriction::evaluate_friction(const FrictionBase::TFrictionInput &input, mv2dsim::vec2 &out_result_force_local) const
{
	b2Body *b2Veh = m_my_vehicle.getBox2DChassisBody();

	for (int wheel=0;wheel<2;wheel++)
	{
		// Action/Reaction, slippage, etc:
		// --------------------------------------
		const double C_damping = 5.0;
		double net_g2v_force = -input.motor_force - C_damping * input.wheel_speed.x;

		// Apply force: In local (x,y) coordinates (Newtons)
		// --------------------------------------
		out_result_force_local.vals[0] = net_g2v_force;
		out_result_force_local.vals[1] = 0.0;
	}
}


