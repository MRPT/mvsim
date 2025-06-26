/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2025 Francisco Pérez Ibañez                               |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
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

	// wheel angle around the vertical axis (wrt the vehicle frame):
	const double wheel_delta = input.wheel.yaw;

	const double gravity = myVehicle_.parent()->get_gravity();
	const double partial_mass = input.Fz / gravity + input.wheel.mass;

	const double Fz = partial_mass * gravity;  // overall vertical force wheel-ground

	const double R = 0.5 * input.wheel.diameter;  // Wheel radius

	// Slip angle (α)
	const double slip_angle = std::atan2(vel_v.y, vel_v.x) - wheel_delta;

	const double wheel_ground_point_vel =
		std::abs(input.wheel.getW()) > 1e-5 ? input.wheel.getW() * R : 1e-5;

	// Slip ratio (s):
	const double slip_ratio = (wheel_ground_point_vel - vel_w.x) / wheel_ground_point_vel;

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
		// Impulse required to step the lateral slippage:
		wheel_lateral_friction = -vel_w.y * partial_mass / input.context.dt;

		wheel_lateral_friction =
			std::clamp(wheel_lateral_friction, -max_friction_lateral_Fy, max_friction_lateral_Fy);
	}

	// 2) Longitudinal friction (decoupled sub-problem)
	// direction: +x local wrt the wheel
	// -------------------------------------------------

	const double lon_constraint_desired_wheel_w = vel_w.x / R;
	const double desired_wheel_w_impulse = (lon_constraint_desired_wheel_w - input.wheel.getW());
	const double desired_wheel_alpha = desired_wheel_w_impulse / input.context.dt;

	// Force for rolling resistance:
	const double F_rr = 0;	// -sign(vel_w.x) * partial_mass * gravity *  (R1_ * (1 - exp(-A_roll_ *
							// fabs(vel_w.x))) + R2_ * fabs(vel_w.x));

	const double I_yy = input.wheel.Iyy;

	double wheel_long_friction =
		(input.motorTorque - I_yy * desired_wheel_alpha - C_damping_ * input.wheel.getW()) / R +
		F_rr;

	// Slippage: The friction with the ground is not infinite:
	wheel_long_friction = std::clamp(
		wheel_long_friction, -max_friction_longitudinal_Fx, max_friction_longitudinal_Fx);

	// Recalc wheel ang. velocity impulse with this reduced force:
	const double actual_wheel_alpha =
		(input.motorTorque - R * wheel_long_friction - C_damping_ * input.wheel.getW()) / I_yy;

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
		logger->updateColumn("motorTorque", input.motorTorque);
		logger->updateColumn("wheel_long_friction", wheel_long_friction);
		logger->updateColumn("wheel_lateral_friction", wheel_lateral_friction);

		logger->updateColumn("slip_angle", slip_angle);
		logger->updateColumn("slip_ratio", slip_ratio);
	}

	return res;
}
