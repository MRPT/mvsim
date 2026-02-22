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
 *
 * \ingroup friction_module
 */
class WardIagnemmaFriction : public FrictionBase
{
	DECLARES_REGISTER_FRICTION(WardIagnemmaFriction)
   public:
	WardIagnemmaFriction(VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node);

	// See docs in base class.
	virtual mrpt::math::TVector2D evaluate_friction(
		const FrictionBase::TFrictionInput& input) const override;

   private:
	double mu_ = 0.8;  //!< friction coefficient (non-dimensional)
	double C_damping_ = 0.01;  //!< For wheels "internal friction" (N*m*s/rad)
	double C_rr_ = 0.0;	 //!< Rolling resistance coefficient (non-dimensional)
	/**  Ward-Iagnemma rolling resistance coefficient */
	double A_roll_ = 50.0, R1_ = 0.0075, R2_ = 0.02;
};
}  // namespace mvsim
