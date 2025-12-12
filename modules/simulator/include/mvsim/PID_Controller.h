/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
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

	double KP, KI, KD, N;  //!< PID controller parameters
	double max_torque;	//!< Maximum abs. value torque (for clamp) [Nm]

	// TEMPORARY!
	double tau_ff = 0.0, tau_ff1 = 0, tau_ff2 = 0, tau_ff3 = 0, tau_ff4 = 0, tau_ff5 = 0,
		   tau_ff6 = 0;	 //!< Feedforward time constant (in seconds)
	double K_ff = 0.0, K_ff1 = 0, K_ff2 = 0, K_ff3 = 0, K_ff4 = 0, K_ff5 = 0,
		   K_ff6 = 0;  //!< Feedforward gain

	double new_vel_max = 0.0, new_w_max = 0.0, dist_obst = 0.0;

	double tau_f = 0.0;	 //!< Time constant for reference filter
	int n_f = 1;  //!< Order of the reference filter

	double max_out = 0.0;  //!< Maximum absolute output value

	bool enable_antiwindup = true;
	bool enable_feedforward = false;
	bool enable_referencefilter = false;
	bool enable_adaptative = false;

	bool full_payload = false;

	/** err = desired-actual, dt=ellapsed time in secs */
	double compute(double spVel, double actVel, double torque_slope, double dt);

	/** Reset internal status to all zeros (KP, KI,DP, max_out remain
	 * unmodified) */
	void reset();

   private:
	double spVel_f_y1 = 0.0;
	double spVel_f_y2 = 0.0;
	double d_state = 0.0;
	double ff_state = 0.0;
	double lastOutput = 0;
	double e_n = 0, e_n_1 = 0, e_n_2 = 0, i_state = 0.0;
};
}  // namespace mvsim
