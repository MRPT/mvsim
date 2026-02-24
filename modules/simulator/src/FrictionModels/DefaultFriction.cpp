/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/FrictionModels/DefaultFriction.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;

DefaultFriction::DefaultFriction(VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node)
	: FrictionBase(my_vehicle)
{
	// Sanity: we can tolerate node==nullptr (=> means use default params).
	if (node && 0 != strcmp(node->name(), "friction"))
	{
		throw std::runtime_error("<friction>...</friction> XML node was expected!!");
	}

	// Parse XML params:
	if (node)
	{
		parse_xmlnode_children_as_param(*node, params_, world_->user_defined_variables());
	}
}

// See docs in base class.
mrpt::math::TVector2D DefaultFriction::evaluate_friction(
	const FrictionBase::TFrictionInput& input) const
{
	// Rotate wheel velocity vector from veh. frame => wheel frame
	const mrpt::poses::CPose2D wRot(0, 0, input.wheel.yaw);

	// Velocity of the wheel cog in the frame of the wheel itself:
	const mrpt::math::TVector2D vel_w = wRot.inverseComposePoint(input.wheelCogLocalVel);

	// Gravity slope force in wheel-local frame:
	const mrpt::math::TVector2D gravSlope_w = wRot.inverseComposePoint(input.gravSlopeForce);

	// Action/Reaction, slippage, etc:
	// --------------------------------------
	const double mu = input.mu_override.value_or(mu_);
	const double gravity = myVehicle_.parent()->get_gravity();
	const double partial_mass = input.Fz / gravity + input.wheel.mass;
	const double max_friction = mu * partial_mass * gravity;

	// 1) Lateral friction (decoupled sub-problem)
	// --------------------------------------------
	double wheel_lateral_friction = 0.0;  // direction: +y local wrt the wheel
	{
		// Impulse to cancel lateral velocity + counteract gravity slope:
		wheel_lateral_friction = -vel_w.y * partial_mass / input.context.dt - gravSlope_w.y;

		wheel_lateral_friction = std::clamp(wheel_lateral_friction, -max_friction, max_friction);
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

	const double desired_wheel_w_impulse = (lon_constraint_desired_wheel_w - input.wheel.getW());

	const double desired_wheel_alpha = desired_wheel_w_impulse / input.context.dt;

	// (eq. 3)==> Find out F_r
	// Iyy_w * \Delta\omega_w = dt*\tau-  R*dt*Fri    -C_damp * \omega_w * dt
	// "Damping" / internal friction of the wheel's shaft, etc.
	const double C_damping = C_damping_;

	// Rolling resistance: constant-magnitude torque opposing wheel rotation,
	// proportional to normal force: T_rr = C_rr * F_normal * R
	// Uses a smooth tanh approximation near zero to avoid sign discontinuity.
	const double F_normal = partial_mass * gravity;
	const double C_rr = input.C_rr_override.value_or(C_rr_);
	const double T_rolling_resistance =
		(C_rr > 0 && std::abs(input.wheel.getW()) > 1e-4)
			? C_rr * F_normal * R * std::tanh(input.wheel.getW() * 100.0)
			: 0.0;

	const double I_yy = input.wheel.Iyy;

	// Effective motor torque after subtracting bearing damping and rolling resistance:
	const double effectiveTorque =
		input.motorTorque - C_damping * input.wheel.getW() - T_rolling_resistance;

	double F_friction_lon = (effectiveTorque - I_yy * desired_wheel_alpha) / R;

	// Slippage: The friction with the ground is not infinite:
	F_friction_lon = std::clamp(F_friction_lon, -max_friction, max_friction);

	// Recalc wheel ang. velocity impulse with this reduced force:
	const double actual_wheel_alpha = (effectiveTorque - R * F_friction_lon) / I_yy;

	// Add slope gravity force to the contact-patch friction (acts on chassis,
	// not on wheel spin). Clamped by remaining friction capacity.
	const double remaining_lon = max_friction - std::abs(F_friction_lon);
	F_friction_lon -= std::clamp(gravSlope_w.x, -remaining_lon, remaining_lon);

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
		auto logger = logger_->lock();

		logger->updateColumn("desired_wheel_alpha", desired_wheel_alpha);
		logger->updateColumn("wheel_lateral_friction", wheel_lateral_friction);
		logger->updateColumn("desired_wheel_w_impulse", desired_wheel_w_impulse);
		logger->updateColumn("F_friction_lon", F_friction_lon);
		logger->updateColumn("actual_wheel_alpha", actual_wheel_alpha);
		logger->updateColumn("motor_torque", input.motorTorque);
		logger->updateColumn("wheel_long_friction", wheel_long_friction);
	}

	return res;
}
