/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/FrictionModels/FrictionBase.h>

namespace mv2dsim
{
	/**
	  */
	class LinearFriction : public FrictionBase
	{
	public:
		LinearFriction();

		virtual void init(World* world, VehicleBase * my_vehicle);
		virtual void update_step(const TSimulContext &context);

	protected:
		World       * m_world;
		VehicleBase * m_vehicle;


	private:

	};
}
