/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2025 Francisco Pérez Ibañez                               |
  | Copyright (C) 2014-2026 Jose Luis Blanco Claraco                        |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_friction_joint.h>
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/TParameterDefinitions.h>
#include <mvsim/Wheel.h>

namespace mvsim
{
/** Friction model based on the "Ellipse Curve Method".
 *
 *  This model is used to compute the friction forces acting on the wheels of a vehicle based on
 *  the slip angle and longitudinal slip.
 *
 * \note Contributed by Francisco Pérez Ibañez, reviewed by J.L. Blanco-Claraco, J.L. Torres-Moreno.
 */
class EllipseCurveMethod : public FrictionBase
{
	DECLARES_REGISTER_FRICTION(EllipseCurveMethod)
   public:
	EllipseCurveMethod(VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node);

	// See docs in base class.
	virtual mrpt::math::TVector2D evaluate_friction(
		const FrictionBase::TFrictionInput& input) const override;

   private:
	// TODO:  Move to VehicleBase
	// double CA_ = 8;	 //!< aerodynamic force coefficient (non-dimensional)

	double C_damping_ = 0.01;  //!< For wheels "internal friction" (N*m*s/rad)
	double C_rr_ = 0.0;	 //!< Rolling resistance coefficient (non-dimensional)

	// Ellipse curve parameters:
	double C_alpha_ = 8.5;	//!< Coefficient for lateral slip angle (non-dimensional)
	double C_s_ = 7.5;	//!< Coefficient for longitudinal slip (non-dimensional)

	double slip_angle_saturation_ = 0.1;  //!< Saturation value for slip angle α_s (rad)
	double slip_ratio_saturation_ = 0.1;  //!< Saturation value for slip ratio s_s (unitless)

	double C_alpha_s_ = 0.5;  //!< Slip coefficient (non-dimensional)
	double C_s_alpha_ = 0.5;  //!< Slip coefficient (non-dimensional)

   public:
	const TParameterDefinitions params_ = {
		{"C_damping", {"%lf", &C_damping_}},
		{"C_rr", {"%lf", &C_rr_}},
		{"C_s", {"%lf", &C_s_}},
		{"slip_angle_saturation", {"%lf", &slip_angle_saturation_}},
		{"slip_ratio_saturation", {"%lf", &slip_ratio_saturation_}},
		{"C_alpha", {"%lf", &C_alpha_}},
		{"C_alpha_s", {"%lf", &C_alpha_s_}},
		{"C_s_alpha", {"%lf", &C_s_alpha_}}};
};
}  // namespace mvsim
