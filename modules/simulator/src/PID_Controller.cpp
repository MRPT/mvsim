/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/PID_Controller.h>

using namespace mvsim;

/** err = desired-actual, dt=ellapsed time in secs */
double PID_Controller::compute(double err, double dt)
{
	e_n_2 = e_n_1;
	e_n_1 = e_n;
	e_n = err;

	double output = lastOutput + KP * (e_n - e_n_1) + KI * e_n * dt +
					KD * (e_n - 2 * e_n_1 + e_n_2) / dt;

	// prevent integral windup
	if (max_out != 0.0 && (output < -max_out || output > max_out))
	{
		output -= KI * e_n * dt;
	}

	lastOutput = output;

	if (max_out != 0.0)
	{
		if (output < -max_out) output = -max_out;
		if (output > max_out) output = max_out;
	}

	return output;
}

void PID_Controller::reset()
{
	lastOutput = 0;
	e_n = 0, e_n_1 = 0, e_n_2 = 0;
}
