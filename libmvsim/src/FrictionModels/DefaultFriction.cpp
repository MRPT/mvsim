/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#include <mvsim/World.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/FrictionModels/DefaultFriction.h>

#include <rapidxml.hpp>
#include "xml_utils.h"

using namespace mvsim;

DefaultFriction::DefaultFriction(
	VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node)
	: FrictionBase(my_vehicle), m_mu(0.8), m_C_damping(1.0)
{
	// Sanity: we can tolerate node==NULL (=> means use default params).
	if (node && 0 != strcmp(node->name(), "friction"))
		throw std::runtime_error(
			"<friction>...</friction> XML node was expected!!");

	if (node)
	{
		// Parse params:
		std::map<std::string, TParamEntry> params;
		params["mu"] = TParamEntry("%lf", &m_mu);
		params["C_damping"] = TParamEntry("%lf", &m_C_damping);

		// Parse XML params:
		parse_xmlnode_children_as_param(*node, params);
	}
}

// See docs in base class.
void DefaultFriction::evaluate_friction(
	const FrictionBase::TFrictionInput& input,
	mrpt::math::TPoint2D& out_result_force_local) const
{
	// Rotate wheel velocity vector from veh. frame => wheel frame
	const mrpt::poses::CPose2D wRot(0, 0, input.wheel.yaw);
	const mrpt::poses::CPose2D wRotInv(0, 0, -input.wheel.yaw);
	mrpt::math::TPoint2D vel_w;
	wRotInv.composePoint(input.wheel_speed, vel_w);

	// Action/Reaction, slippage, etc:
	// --------------------------------------
	const double mu = m_mu;
	const double gravity = m_my_vehicle.getWorldObject()->get_gravity();
	const double partial_mass = input.weight / gravity + input.wheel.mass;
	const double max_friction = mu * partial_mass * gravity;

	// 1) Lateral friction (decoupled sub-problem)
	// --------------------------------------------
	double wheel_lat_friction = 0.0;  // direction: +y local wrt the wheel
	{
		// Impulse required to step the lateral slippage:
		wheel_lat_friction = -vel_w.y * partial_mass / input.context.dt;

		wheel_lat_friction =
			b2Clamp(wheel_lat_friction, -max_friction, max_friction);
	}

	// 2) Longitudinal friction (decoupled sub-problem)
	// -------------------------------------------------
	double wheel_long_friction = 0.0;  // direction: +x local wrt the wheel

	// (eq. 1)==> desired impulse in wheel spinning speed.
	// wheel_C_lon_vel = vel_w.x - input.wheel.w * 0.5*input.wheel.diameter

	// It should be = 0 for no slippage (nonholonomic constraint): find out
	// required wheel \omega:case '4':
	const double R = 0.5 * input.wheel.diameter;  // Wheel radius
	const double lon_constraint_desired_wheel_w = vel_w.x / R;
	const double desired_wheel_w_impulse =
		(lon_constraint_desired_wheel_w - input.wheel.getW());
	const double desired_wheel_alpha =
		desired_wheel_w_impulse / input.context.dt;

	// (eq. 3)==> Find out F_r
	// Iyy_w * \Delta\omega_w = dt*\tau-  R*dt*Fri    -C_damp * \omega_w * dt
	// "Damping" / internal friction of the wheel's shaft, etc.
	const double C_damping = m_C_damping;
	// const mrpt::math::TPoint2D wheel_damping(- C_damping *
	// input.wheel_speed.x, 0.0);

	const double I_yy = input.wheel.Iyy;
	double F_friction_lon = (input.motor_torque - I_yy * desired_wheel_alpha -
							 C_damping * input.wheel.getW()) /
							R;

	// Slippage: The friction with the ground is not infinite:
	F_friction_lon = b2Clamp(F_friction_lon, -max_friction, max_friction);

	// Recalc wheel ang. velocity impulse with this reduced force:
	const double actual_wheel_alpha = (input.motor_torque - R * F_friction_lon -
									   C_damping * input.wheel.getW()) /
									  I_yy;

	// Apply impulse to wheel's spinning:
	input.wheel.setW(
		input.wheel.getW() + actual_wheel_alpha * input.context.dt);

	wheel_long_friction = F_friction_lon;

	// Resultant force: In local (x,y) coordinates (Newtons) wrt the Wheel
	// -----------------------------------------------------------------------
	const mrpt::math::TPoint2D result_force_wrt_wheel(
		wheel_long_friction, wheel_lat_friction);

	// Rotate to put: Wheel frame ==> vehicle local framework:
	wRot.composePoint(result_force_wrt_wheel, out_result_force_local);
}
