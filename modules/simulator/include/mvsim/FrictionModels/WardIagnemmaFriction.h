/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2017  Borys Tymchenko                                     |
  | Odessa National Polytechnic University                                  |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_friction_joint.h>
#include <mvsim/FrictionModels/FrictionBase.h>

#include <vector>

namespace mvsim
{
/** Friction model implemented with respect to
 * http://web.mit.edu/mobility/publications/Iagnemma_TRO_07.pdf
 * A Dynamic-Model-Based Wheel Slip Detector for Mobile Robots on Outdoor
 * Terrain
 * Chris C. Ward and Karl Iagnemma
 */
class WardIagnemmaFriction : public FrictionBase
{
	DECLARES_REGISTER_FRICTION(WardIagnemmaFriction)
   public:
	WardIagnemmaFriction(
		VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node);

	// See docs in base class.
	virtual void evaluate_friction(
		const FrictionBase::TFrictionInput& input,
		mrpt::math::TPoint2D& out_result_force_local) const override;

   private:
	double mu_;	 //!< friction coeficient (non-dimensional)
	double C_damping_;	//!< For wheels "internal friction" (N*m*s/rad)
	double A_roll_, R1_,
		R2_;  //!< Ward-Iagnemma rolling resistance coefficient
};
}  // namespace mvsim
