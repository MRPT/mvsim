/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/FrictionModels/FrictionBase.h>
#include <vector>
#include <Box2D/Dynamics/Joints/b2FrictionJoint.h>

namespace mv2dsim
{
	/** The default friction model for interaction between each wheel-ground contact point
	  */
	class DefaultFriction : public FrictionBase
	{
		DECLARES_REGISTER_FRICTION(DefaultFriction)
	public:
		DefaultFriction(VehicleBase &my_vehicle, const rapidxml::xml_node<char> *node);

		// See docs in base class.
		virtual void evaluate_friction(const FrictionBase::TFrictionInput &input, mrpt::math::TPoint2D &out_result_force_local) const;

	private:

	};
}
