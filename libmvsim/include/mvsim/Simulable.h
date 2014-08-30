/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mvsim/basic_types.h>

namespace mvsim
{
	class Simulable
	{
	public:
		/** Process right before the integration of dynamic equations for each timestep: set action forces from motors, update friction models, etc. */
		virtual void simul_pre_timestep(const TSimulContext &context)  { /* default: do nothing*/ }

		/** Override to do any required process right after the integration of dynamic equations for each timestep */
		virtual void simul_post_timestep(const TSimulContext &context)  { /* default: do nothing*/ }

		/** Override to register external forces exerted by other WorldElements. 
		  * Force is (fx,fy) in global coordinates. Application point is (local_ptx,local_pty) in the body local frame */
		virtual void apply_force(double fx, double fy, double local_ptx = 0.0, double local_pty = 0.0)  { /* default: do nothing*/ }
	};
}
