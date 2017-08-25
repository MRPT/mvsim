/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2017  Borys Tymchenko                                     |
  | Odessa National Polytechnic University                                  |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/FrictionModels/FrictionBase.h>
#include <vector>
#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>

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
		mrpt::math::TPoint2D& out_result_force_local) const;

   private:
	double m_mu;  //!< friction coeficient (non-dimensional)
	double m_C_damping;  //!< For wheels "internal friction" (N*m*s/rad)
	double m_A_roll, m_R1,
		m_R2;  //!< Ward-Iagnemma rolling resistance coefficient
};
}
