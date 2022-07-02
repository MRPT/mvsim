/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
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
 * point
 */
class DefaultFriction : public FrictionBase
{
	DECLARES_REGISTER_FRICTION(DefaultFriction)
   public:
	DefaultFriction(
		VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node);

	// See docs in base class.
	virtual void evaluate_friction(
		const FrictionBase::TFrictionInput& input,
		mrpt::math::TPoint2D& out_result_force_local) const override;

   private:
	double m_mu;  //!< friction coeficient (non-dimensional)
	double m_C_damping;	 //!< For wheels "internal friction" (N*m*s/rad)

   public:
	const TParameterDefinitions m_params = {
		{"mu", {"%lf", &m_mu}}, {"C_damping", {"%lf", &m_C_damping}}};
};
}  // namespace mvsim
