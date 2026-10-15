/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
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
		params["C_rr"] = TParamEntry("%lf", &C_rr_);
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

	// Gravity slope force in wheel-local frame:
	mrpt::math::TPoint2D gravSlope_w;
	wRotInv.composePoint(input.gravSlopeForce, gravSlope_w);

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

	// Two independent rolling-resistance models coexist here:
	//
	// 1) F_rr (ground-contact): Ward-Iagnemma terrain interaction model using
	//    params R1_, R2_, A_roll_. Velocity-dependent resistance, applied as an
	//    equivalent torque (F_rr * R) opposing the wheel spin: adding it as a
	//    contact force instead would spin the wheel up, and the wheel would give
	//    that momentum back to the chassis in the next steps.
	//
	// 2) T_rolling_resistance (tire deformation): torque-based model using
	//    C_rr_. Opposes wheel spin as a shaft-level torque subtracted from
	//    the effective motor torque.
	//
	// Both may be enabled simultaneously for off-road scenarios where terrain
	// drag (F_rr) and tire hysteresis (T_rolling_resistance) are distinct
	// effects. If only one source of rolling resistance is desired, configure
	// either {R1_, R2_, A_roll_} or C_rr_, not both.
	const double F_normal = partial_mass * gravity;
	const double F_rr = -sign(vel_w.x) * F_normal *
						(R1_ * (1 - exp(-A_roll_ * fabs(vel_w.x))) + R2_ * fabs(vel_w.x));

	double T_rolling_resistance = 0;
	double dTrr_dw = 0;
	rollingResistance(
		input.C_rr_override.value_or(C_rr_), F_normal, R, w, T_rolling_resistance, dTrr_dw);

	// Bearing damping and rolling resistance (both models):
	const double otherTorque = -C_damping_ * w - T_rolling_resistance + F_rr * R;

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
		auto l = logger_->lock();

		l->updateColumn("F_rr", F_rr);
		l->updateColumn("desired_wheel_alpha", desired_wheel_alpha);
		l->updateColumn("actual_wheel_alpha", actual_wheel_alpha);
		l->updateColumn("motor_torque", input.motorTorque);
		l->updateColumn("wheel_long_friction", wheel_long_friction);
	}

	return res;
}
