/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_friction_joint.h>
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/TParameterDefinitions.h>

namespace mvsim
{
/** The default friction model for interaction between each wheel-ground contact
 * point.
 * \ingroup friction_module
 */
class DefaultFriction : public FrictionBase
{
	DECLARES_REGISTER_FRICTION(DefaultFriction)
   public:
	DefaultFriction(VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node);

	// See docs in base class.
	virtual mrpt::math::TVector2D evaluate_friction(
		const FrictionBase::TFrictionInput& input) const override;

   private:
	double mu_ = 0.8;  //!< friction coefficient (non-dimensional)
	double C_damping_ = 0.01;  //!< For wheels "internal friction" (N*m*s/rad)
	double C_rr_ = 0.0;	 //!< Rolling resistance coefficient (non-dimensional)

   public:
	const TParameterDefinitions params_ = {
		{"mu", {"%lf", &mu_}}, {"C_damping", {"%lf", &C_damping_}}, {"C_rr", {"%lf", &C_rr_}}};
};
}  // namespace mvsim
