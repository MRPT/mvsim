/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#pragma once

namespace mv2dsim
{
	struct PID_Controller
	{
		PID_Controller();

		double KP,KI,KD;
		double I_MAX_ABS; //!< Max abs value for I part.
		double max_out; //!< For clamping (0=no clamp)

		/** err = desired-actual, dt=ellapsed time in secs */
		double compute(double err, double dt);

	private:
		double m_i_term; 
	};
}
