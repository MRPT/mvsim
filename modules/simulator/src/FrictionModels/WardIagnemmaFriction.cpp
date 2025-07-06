/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
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

namespace
{
double sign(double x) { return (double)((x > 0) - (x < 0)); }
}  // namespace

WardIagnemmaFriction::WardIagnemmaFriction(
	VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node)
	: FrictionBase(my_vehicle)
{
	// Sanity: we can tolerate node==nullptr (=> means use default params).
	if (node && 0 != strcmp(node->name(), "friction"))
	{
		throw std::runtime_error("<friction>...</friction> XML node was expected!!");
	}

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
			*node, params, world_->user_defined_variables(), "WardIagnemmaFriction",
			my_vehicle.parent() /*for logger*/);
	}
}

// See docs in base class.
mrpt::math::TVector2D WardIagnemmaFriction::evaluate_friction(
	const FrictionBase::TFrictionInput& input) const
{
	// Rotate wheel velocity vector from veh. frame => wheel frame
	const mrpt::poses::CPose2D wRot(0, 0, input.wheel.yaw);
	const mrpt::poses::CPose2D wRotInv(0, 0, -input.wheel.yaw);
	mrpt::math::TPoint2D vel_w;
	wRotInv.composePoint(input.wheelCogLocalVel, vel_w);

	// Action/Reaction, slippage, etc:
	// --------------------------------------
	const double mu = mu_;
	const double gravity = myVehicle_.parent()->get_gravity();
	const double partial_mass = input.Fz / gravity + input.wheel.mass;
	const double max_friction = mu * partial_mass * gravity;

	// 1) Lateral friction (decoupled sub-problem)
	// --------------------------------------------
	double wheel_lateral_friction = 0.0;  // direction: +y local wrt the wheel
	{
		// Impulse required to step the lateral slippage:
		wheel_lateral_friction = -vel_w.y * partial_mass / input.context.dt;

		wheel_lateral_friction = std::clamp(wheel_lateral_friction, -max_friction, max_friction);
	}

	// 2) Longitudinal friction (decoupled sub-problem)
	// -------------------------------------------------
	double wheel_long_friction = 0.0;  // direction: +x local wrt the wheel

	// (eq. 1)==> desired impulse in wheel spinning speed.
	// wheel_C_lon_vel = vel_w.x - input.wheel.w * 0.5*input.wheel.diameter

	// It should be = 0 for no slippage (non-holonomic constraint): find out
	// required wheel \omega:case '4':
	const double R = 0.5 * input.wheel.diameter;  // Wheel radius
	const double lon_constraint_desired_wheel_w = vel_w.x / R;
	const double desired_wheel_w_impulse = (lon_constraint_desired_wheel_w - input.wheel.getW());
	const double desired_wheel_alpha = desired_wheel_w_impulse / input.context.dt;

	// (eq. 3)==> Find out F_r
	// Iyy_w * \Delta\omega_w = dt*\tau-  R*dt*Fri    -C_damp * \omega_w * dt
	// "Damping" / internal friction of the wheel's shaft, etc.
	const double C_damping = C_damping_;

	// const mrpt::math::TPoint2D wheel_damping(- C_damping *
	// input.wheel_speed.x, 0.0);

	// Actually, Ward-Iagnemma rolling resistance is here (longitudinal one):

	const double F_rr = -sign(vel_w.x) * partial_mass * gravity *
						(R1_ * (1 - exp(-A_roll_ * fabs(vel_w.x))) + R2_ * fabs(vel_w.x));

	const double I_yy = input.wheel.Iyy;
	//                                  There are torques this is force   v
	double F_friction_lon =
		(input.motorTorque - I_yy * desired_wheel_alpha - C_damping * input.wheel.getW()) / R +
		F_rr;

	// Slippage: The friction with the ground is not infinite:
	F_friction_lon = std::clamp(F_friction_lon, -max_friction, max_friction);

	// Recalc wheel ang. velocity impulse with this reduced force:
	const double actual_wheel_alpha =
		(input.motorTorque - R * F_friction_lon - C_damping * input.wheel.getW()) / I_yy;

	// Apply impulse to wheel's spinning:
	input.wheel.setW(input.wheel.getW() + actual_wheel_alpha * input.context.dt);

	wheel_long_friction = F_friction_lon;

	// Resultant force: In local (x,y) coordinates (Newtons) wrt the Wheel
	// -----------------------------------------------------------------------
	const mrpt::math::TPoint2D result_force_wrt_wheel(wheel_long_friction, wheel_lateral_friction);

	// Rotate to put: Wheel frame ==> vehicle local framework:
	mrpt::math::TVector2D res;
	wRot.composePoint(result_force_wrt_wheel, res);

	if (logger_ && !logger_->expired())
	{
		auto l = logger_->lock();

		l->updateColumn("F_rr", F_rr);
		l->updateColumn("desired_wheel_alpha", desired_wheel_alpha);
		l->updateColumn("actual_wheel_alpha", actual_wheel_alpha);
		l->updateColumn("motor_torque", input.motorTorque);
		l->updateColumn("wheel_long_friction", wheel_long_friction);
	}

	return res;
}
