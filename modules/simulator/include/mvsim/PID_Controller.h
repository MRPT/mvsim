/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

namespace mvsim
{
struct PID_Controller
{
	PID_Controller();

	double KP, KI, KD;
	double max_out;	 //!< For clamping (0=no clamp)

	/** err = desired-actual, dt=ellapsed time in secs */
	double compute(double err, double dt);

   private:
	double lastOutput;
	double e_n, e_n_1, e_n_2;
};
}  // namespace mvsim
