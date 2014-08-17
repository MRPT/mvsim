/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/basic_types.h>

namespace mv2dsim
{
	class Simulable
	{
	public:
		/** Process right before the integration of dynamic equations for each timestep: set action forces from motors, update friction models, etc. */
		virtual void simul_pre_timestep(const TSimulContext &context) = 0;

		/** Override to do any required process right after the integration of dynamic equations for each timestep */
		virtual void simul_post_timestep(const TSimulContext &context) = 0;
	};
}
