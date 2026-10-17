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

	const double R = 0.5 * input.wheel.diameter;  // Wheel radius
	const double dt = input.context.dt;
	const double w = input.wheel.getW();

	// Wheel spin acceleration for rolling without slipping within the step:
	const double desired_wheel_alpha = (vel_w.x / R - w) / dt;

	// Lateral force to cancel the lateral velocity + counteract gravity slope:
	const double latForceToStick =
		-vel_w.y * (input.lateralMass > 0 ? input.lateralMass : partial_mass) / dt - gravSlope_w.y;

	// Bearing damping and rolling resistance (both integrated implicitly):
	double T_rolling_resistance = 0;
	double dTrr_dw = 0;
	rollingResistance(
		input.C_rr_override.value_or(C_rr_), partial_mass * gravity, R, w, T_rolling_resistance,
		dTrr_dw);
	const double otherTorque = -C_damping_ * w - T_rolling_resistance;

	// Coulomb friction (friction circle):
	double wheel_long_friction = 0;	 // direction: +x local wrt the wheel
	double wheel_lateral_friction = 0;	// direction: +y local wrt the wheel
	double actual_wheel_alpha = 0;
	solveContact(
		input, otherTorque, C_damping_ + dTrr_dw, desired_wheel_alpha, R, latForceToStick,
		{vel_w.x - w * R, vel_w.y}, max_friction, wheel_long_friction, wheel_lateral_friction,
		actual_wheel_alpha);

	// Add slope gravity force to the contact-patch friction (acts on chassis,
	// not on wheel spin). Clamped by remaining friction capacity.
	const double remaining_lon =
		remainingLongitudinalForce(max_friction, wheel_long_friction, wheel_lateral_friction);
	wheel_long_friction -= std::clamp(gravSlope_w.x, -remaining_lon, remaining_lon);

	// Apply impulse to wheel's spinning:
	input.wheel.setW(w + actual_wheel_alpha * dt);

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
		logger->updateColumn("actual_wheel_alpha", actual_wheel_alpha);
		logger->updateColumn("motor_torque", input.motorTorque);
		logger->updateColumn("wheel_long_friction", wheel_long_friction);
	}

	return res;
}
