/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/FrictionModels/FrictionBase.h>
#include <vector>
#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>

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
		mrpt::math::TPoint2D& out_result_force_local) const;

   private:
	double m_mu;  //!< friction coeficient (non-dimensional)
	double m_C_damping;  //!< For wheels "internal friction" (N*m*s/rad)
};
}
