/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#pragma once

namespace mvsim
{
	struct PID_Controller
	{
		PID_Controller();

    double KP,KI,KD;
		double max_out; //!< For clamping (0=no clamp)

		/** err = desired-actual, dt=ellapsed time in secs */
		double compute(double err, double dt);

  private:
    double lastOutput;
    double e_n, e_n_1, e_n_2;
	};
}
