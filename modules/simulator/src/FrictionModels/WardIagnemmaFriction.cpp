/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/FrictionModels/WardIagnemmaFriction.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;

static double sign(double x) { return (double)((x > 0) - (x < 0)); }
WardIagnemmaFriction::WardIagnemmaFriction(
	VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node)
	: FrictionBase(my_vehicle),
	  mu_(0.8),
	  C_damping_(1.0),
	  A_roll_(50),
	  R1_(0.08),
	  R2_(0.05)
{
	// Sanity: we can tolerate node==nullptr (=> means use default params).
	if (node && 0 != strcmp(node->name(), "friction"))
		throw std::runtime_error(
			"<friction>...</friction> XML node was expected!!");

	if (node)
	{
		// Parse params:
		TParameterDefinitions params;
		params["mu"] = TParamEntry("%lf", &mu_);
		params["C_damping"] = TParamEntry("%lf", &C_damping_);
		params["A_roll"] = TParamEntry("%lf", &A_roll_);
		params["R1"] = TParamEntry("%lf", &R1_);
		params["R2"] = TParamEntry("%lf", &R2_);
		// Parse XML params:
		parse_xmlnode_children_as_param(
			*node, params, world_->user_defined_variables());
	}

	MRPT_UNSCOPED_LOGGER_START;
	MRPT_LOG_DEBUG("WardIagnemma Creates!");
	MRPT_UNSCOPED_LOGGER_END;
}

// See docs in base class.
void WardIagnemmaFriction::evaluate_friction(
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
	const double mu = mu_;
	const double gravity = my_vehicle_.getWorldObject()->get_gravity();
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
	const double C_damping = C_damping_;
	// const mrpt::math::TPoint2D wheel_damping(- C_damping *
	// input.wheel_speed.x, 0.0);

	// Actually, Ward-Iagnemma rolling resistance is here (longitudal one):

	const double F_rr =
		-sign(vel_w.x) * partial_mass * gravity *
		(R1_ * (1 - exp(-A_roll_ * fabs(vel_w.x))) + R2_ * fabs(vel_w.x));

	if (!logger_.expired())
	{
		logger_.lock()->updateColumn("F_rr", F_rr);
	}

	const double I_yy = input.wheel.Iyy;
	//                                  There are torques this is force   v
	double F_friction_lon = (input.motor_torque - I_yy * desired_wheel_alpha -
							 C_damping * input.wheel.getW()) /
								R +
							F_rr;

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
