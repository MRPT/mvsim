/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/basic_types.h>
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

namespace mv2dsim
{
	/** Virtual base class for all friction models */
	class FrictionBase
	{
	public:
		virtual void init(World* world, VehicleBase * my_vehicle)=0;
		virtual void update_step(const TSimulContext &context)=0;

		virtual ~FrictionBase() { }

	protected:

	};

	typedef stlplus::smart_ptr<FrictionBase> FrictionBasePtr;

}
