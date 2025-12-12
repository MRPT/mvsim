/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_friction_joint.h>
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/TParameterDefinitions.h>

#include <vector>

namespace mvsim
{
/** The default friction model for interaction between each wheel-ground contact
 * point. No rolling resistance.
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

	/// TEMPORARY
	double getMu() const { return mu_; }
	double getCdamping() const { return C_damping_; }
	double getCrr() const { return Crr_; }

   private:
	double mu_;	 //!< friction coeficient (non-dimensional)
	double C_damping_;	//!< For wheels "internal friction" (N*m*s/rad)

	/// TEMPORARY
	double Crr_;  //!< rolling resistance coefficient (non-dimensional). Only for Feedforward
				  //!< controller

   public:
	const TParameterDefinitions params_ = {
		{"mu", {"%lf", &mu_}}, {"C_damping", {"%lf", &C_damping_}}, {"Crr", {"%lf", &Crr_}}};
};
}  // namespace mvsim
