/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2025 Francisco Pérez Ibañez                               |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/FrictionModels/DefaultFriction.h>  // For use as default model
#include <mvsim/FrictionModels/EllipseCurveMethod.h>
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/Simulable.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/Wheel.h>
#include <mvsim/World.h>

#include <cmath>
#include <rapidxml.hpp>

#include "parse_utils.h"
#include "xml_utils.h"

using namespace mvsim;

EllipseCurveMethod::EllipseCurveMethod(
	VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node)
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

namespace
{
// Saturation function
double saturation(double x, double x_abs_max)
{
	return (std::abs(x) < x_abs_max) ? x : (x > 0 ? x_abs_max : -x_abs_max);
}
}  // namespace

// See docs in base class.
mrpt::math::TVector2D EllipseCurveMethod::evaluate_friction(
	const FrictionBase::TFrictionInput& input) const
{
	// This function is called once for each wheel of the vehicle, for each simulation time step.

	// Rotate wheel velocity vector from veh. frame => wheel frame
	const mrpt::poses::CPose2D wRot(0, 0, input.wheel.yaw);
	const mrpt::poses::CPose2D wRotInv(0, 0, -input.wheel.yaw);

	// Velocity of the wheel cog in the frame of the vehicle:
	const mrpt::math::TPoint3D vel_v = {input.wheelCogLocalVel.x, input.wheelCogLocalVel.y, 0};

	// Velocity of the wheel cog in the frame of the wheel itself:
	const mrpt::math::TPoint3D vel_w = wRotInv.composePoint(vel_v);

	// Gravity slope force in wheel-local frame:
	mrpt::math::TPoint2D gravSlope_w;
	wRotInv.composePoint(input.gravSlopeForce, gravSlope_w);

	// wheel angle around the vertical axis (wrt the vehicle frame):
	const double wheel_delta = input.wheel.yaw;

	const double gravity = myVehicle_.parent()->get_gravity();
	const double partial_mass = input.Fz / gravity + input.wheel.mass;

	const double Fz = partial_mass * gravity;  // overall vertical force wheel-ground

	const double R = 0.5 * input.wheel.diameter;  // Wheel radius

	// Slip angle (α)
	const double wheel_velocity_angle =
		(std::abs(vel_w.y) > 1e-3 || std::abs(vel_w.x) > 1e-3) ? std::atan2(vel_w.y, vel_w.x) : .0;
	double slip_angle = mrpt::math::angDistance(wheel_velocity_angle, wheel_delta);
	if (std::abs(slip_angle) > M_PI_2)
	{
		// We could check for wheel vx sign, but that would not include some cases like sliding
		// sideways on a slope, etc.
		slip_angle = mrpt::math::angDistance(wheel_velocity_angle, wheel_delta + M_PI);
	}

	const double wheel_ground_point_vel =
		std::abs(input.wheel.getW()) > 1e-5 ? input.wheel.getW() * R : 1e-5;

	// Slip ratio (s):
	const double slip_ratio = std::clamp(
		(wheel_ground_point_vel - vel_w.x) / std::abs(wheel_ground_point_vel), -1.0, 1.0);

	// Maximum friction forces from the ellipse curve model:
	// ------------------------------------------------------
	const double alpha_s = slip_angle_saturation_;
	const double ss = slip_ratio_saturation_;

	const double max_friction_longitudinal_Fx =
		Fz * C_s_ * saturation(slip_ratio, ss) *
		sqrt(1.0 - C_s_alpha_ * mrpt::square(saturation(slip_angle, alpha_s) / alpha_s));

	const double max_friction_lateral_Fy =
		Fz * C_alpha_ * saturation(slip_angle, alpha_s) *
		sqrt(1.0 - C_alpha_s_ * mrpt::square(saturation(slip_ratio, ss) / ss));

	// 1) Lateral friction (decoupled sub-problem)
	// --------------------------------------------
	double wheel_lateral_friction = 0.0;  // direction: +y local wrt the wheel
	{
		// Impulse to cancel lateral velocity + counteract gravity slope:
		wheel_lateral_friction = -vel_w.y * partial_mass / input.context.dt - gravSlope_w.y;

		wheel_lateral_friction =
			std::clamp(wheel_lateral_friction, -max_friction_lateral_Fy, max_friction_lateral_Fy);
	}

	// 2) Longitudinal friction (decoupled sub-problem)
	// direction: +x local wrt the wheel
	// -------------------------------------------------

	const double lon_constraint_desired_wheel_w = vel_w.x / R;
	const double desired_wheel_w_impulse = (lon_constraint_desired_wheel_w - input.wheel.getW());
	const double desired_wheel_alpha = desired_wheel_w_impulse / input.context.dt;

	// Rolling resistance: constant-magnitude torque opposing wheel rotation,
	// proportional to normal force: T_rr = C_rr * F_normal * R
	// Uses a smooth tanh approximation near zero to avoid sign discontinuity.
	const double C_rr = input.C_rr_override.value_or(C_rr_);
	const double T_rolling_resistance = (C_rr > 0 && std::abs(input.wheel.getW()) > 1e-4)
											? C_rr * Fz * R * std::tanh(input.wheel.getW() * 100.0)
											: 0.0;

	const double I_yy = input.wheel.Iyy;

	// Effective motor torque after subtracting bearing damping and rolling resistance:
	const double effectiveTorque =
		input.motorTorque - C_damping_ * input.wheel.getW() - T_rolling_resistance;

	double wheel_long_friction = (effectiveTorque - I_yy * desired_wheel_alpha) / R;

	// Slippage: The friction with the ground is not infinite:
	wheel_long_friction = std::clamp(
		wheel_long_friction, -max_friction_longitudinal_Fx, max_friction_longitudinal_Fx);

	// Recalc wheel ang. velocity impulse with this reduced force:
	const double actual_wheel_alpha = (effectiveTorque - R * wheel_long_friction) / I_yy;

	// Add slope gravity force to the contact-patch friction (acts on chassis,
	// not on wheel spin). Clamped by remaining friction capacity.
	const double remaining_lon = max_friction_longitudinal_Fx - std::abs(wheel_long_friction);
	wheel_long_friction -= std::clamp(gravSlope_w.x, -remaining_lon, remaining_lon);

	// Apply impulse to wheel's spinning:
	input.wheel.setW(input.wheel.getW() + actual_wheel_alpha * input.context.dt);

	// Resultant force: In local (x,y) coordinates (Newtons) wrt the Wheel
	// -----------------------------------------------------------------------
	const mrpt::math::TPoint2D result_force_wrt_wheel(wheel_long_friction, wheel_lateral_friction);

	// Rotate to put: Wheel frame ==> vehicle local framework:
	mrpt::math::TVector2D res;
	wRot.composePoint(result_force_wrt_wheel, res);

	// Logger:
	if (logger_ && !logger_->expired())
	{
		auto logger = logger_->lock();

		logger->updateColumn("actual_wheel_alpha", actual_wheel_alpha);
		logger->updateColumn("motor_torque", input.motorTorque);
		logger->updateColumn("wheel_long_friction", wheel_long_friction);
		logger->updateColumn("wheel_lateral_friction", wheel_lateral_friction);

		logger->updateColumn("slip_angle", slip_angle);
		logger->updateColumn("slip_ratio", slip_ratio);

		logger->updateColumn("wheel_ground_point_vel", wheel_ground_point_vel);
		logger->updateColumn("vel_w_x", vel_w.x);
		logger->updateColumn("vel_w_y", vel_w.y);
	}

	return res;
}
