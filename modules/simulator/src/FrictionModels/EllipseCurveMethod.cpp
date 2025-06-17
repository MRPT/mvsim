/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
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
	: FrictionBase(my_vehicle), CA_(8), Caf_(8.5), Cs_(7.5), ss_(0.1), Cafs_(0.5), Csaf_(0.5)
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
// Heaviside function
double miH(double x, double x0) { return (x > x0) ? 1.0 : 0.0; }
// Saturation function
double miS(double x, double x0) { return x * miH(x0, std::abs(x)) + x0 * miH(std::abs(x), x0); }
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

	// Slip angle:
	const double slip_angle = std::atan2(vel_v.y, vel_v.x) - wheel_delta;

	const double gravity = myVehicle_.parent()->get_gravity();
	const double partial_mass = input.Fz / gravity + input.wheel.mass;

	const double Fz = partial_mass * gravity;  // overall vertical force wheel-ground

	// 1) Lateral friction (decoupled sub-problem)
	// --------------------------------------------
	double wheel_lateral_friction = 0.0;  // direction: +y local wrt the wheel
	{
		// Impulse required to step the lateral slippage:
		wheel_lateral_friction = -vel_w.y * partial_mass / input.context.dt;

		wheel_lateral_friction = b2Clamp(wheel_lateral_friction, -max_friction, max_friction);
	}

	// 2) Longitudinal friction (decoupled sub-problem)
	// -------------------------------------------------
	double wheel_long_friction = 0.0;  // direction: +x local wrt the wheel

	// Eq. (21) CNIM paper
	const double max_friction_longitudinal = Fz * (std::abs(s) < ss ? Cs * s : Cs * ss);

	// (eq. 1)==> desired impulse in wheel spinning speed.
	// wheel_C_lon_vel = vel_w.x - input.wheel.w * 0.5*input.wheel.diameter

	// It should be = 0 for no slippage (non-holonomic constraint): find out
	// required wheel \omega:case '4':
	const double R = 0.5 * input.wheel.diameter;  // Wheel radius
	const double lon_constraint_desired_wheel_w = vel_w.x / R;
	const double desired_wheel_w_impulse = (lon_constraint_desired_wheel_w - input.wheel.getW());
	const double desired_wheel_alpha = desired_wheel_w_impulse / input.context.dt;

	// Force for rolling resistance:
	const double F_rr = 0;	// -sign(vel_w.x) * partial_mass * gravity *  (R1_ * (1 - exp(-A_roll_ *
							// fabs(vel_w.x))) + R2_ * fabs(vel_w.x));

	const double I_yy = input.wheel.Iyy;

	double F_friction_lon =
		(input.motorTorque - I_yy * desired_wheel_alpha - C_damping_ * input.wheel.getW()) / R +
		F_rr;

	// Slippage: The friction with the ground is not infinite:
	F_friction_lon = b2Clamp(F_friction_lon, -max_friction_longitudinal, max_friction_longitudinal);

	// Recalc wheel ang. velocity impulse with this reduced force:
	const double actual_wheel_alpha =
		(input.motorTorque - R * F_friction_lon - C_damping_ * input.wheel.getW()) / I_yy;

	// Apply impulse to wheel's spinning:
	input.wheel.setW(input.wheel.getW() + actual_wheel_alpha * input.context.dt);

	wheel_long_friction = F_friction_lon;

	// const double afs = 5.0 * M_PI / 180.0;
	//  const double CA = CA_;

#if 0
	// distancia centro de gravedad a ejes
	const double a1 = std::abs(pos[3].x), a2 = std::abs(pos[0].x);
	const double l = a1 + a2;  // distancia entre ejes
	// longitud ejes
	const double Axf = std::abs(pos[2].y) + std::abs(pos[3].y),
				 Axr = std::abs(pos[0].y) + std::abs(pos[1].y);
	ASSERT_(Axf > 0);  // comprobar si la distancia de los ejes son mayores a 0
	ASSERT_(Axr > 0);

	// 2) Wheels velocity at Tire SR (decoupled sub-problem)
	// -------------------------------------------------
	// duda de cambiar el codigo o no) VehicleBase.cpp line 575 calcula esto pero distinto
	const double vxT = (vel.vx - w * pos[wheel_index].y) * cos(wheel_yaw) +
					   (vel.vy + w * pos[wheel_index].x) * sin(wheel_yaw);

	// 3) Longitudinal slip (decoupled sub-problem)
	// -------------------------------------------------
	// w= velocidad angular
	double s = (wheel_radius * input.wheel.getW() - vxT) /
			   (wheel_radius * input.wheel.getW() * miH(wheel_radius * input.wheel.getW(), vxT) +
				vxT * miH(vxT, wheel_radius * input.wheel.getW()));

	// 5) Longitudinal friction (decoupled sub-problem)
	// -------------------------------------------------
	double wheel_long_friction = 0.0;
	wheel_long_friction =
		max_friction * Cs_ * miS(s, ss_) * sqrt(1 - Csaf_ * pow((miS(slip_angle, afs) / afs), 2));
	// wheel_long_friction = b2Clamp(wheel_long_friction, -1.0, 1.0);

	// 6) Lateral friction (decoupled sub-problem)
	// --------------------------------------------
	double wheel_lateral_friction = 0.0;
	wheel_lateral_friction =
		-max_friction * Caf_ * miS(slip_angle, afs) * sqrt(1 - Cafs_ * pow((miS(s, ss_) / ss_), 2));
	// wheel_lateral_friction = b2Clamp(wheel_lateral_friction, -1.0, 1.0);

	// Recalc wheel ang. velocity impulse with this reduced force:
	const double I_yy = input.wheel.Iyy;
	const double actual_wheel_alpha =
		(input.motorTorque - wheel_radius * wheel_long_friction) / I_yy;
#endif

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
	}

	return res;
}
