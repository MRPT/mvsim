/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

namespace mvsim
{
/** \ingroup mvsim_simulator_module */
struct PID_Controller
{
	PID_Controller() = default;

	double KP = 1.0, KI = 0, KD = 0;
	double max_out = 0;	 //!< For clamping (0=no clamp)

	/** err = desired-actual, dt=ellapsed time in secs */
	double compute(double err, double dt);

	/** Reset internal status to all zeros (KP, KI,DP, max_out remain
	 * unmodified) */
	void reset();

   private:
	double lastOutput = 0;
	double e_n = 0, e_n_1 = 0, e_n_2 = 0;
};
}  // namespace mvsim
