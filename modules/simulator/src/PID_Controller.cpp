/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/PID_Controller.h>

#include <algorithm>
#include <cmath>

using namespace mvsim;

double PID_Controller::filterReference(double setpoint, double dt)
{
	if (!enable_reference_filter || dt <= 0.0)
	{
		return setpoint;
	}

	const double tau = std::max(1e-6, reference_filter_tau);
	const double a = std::exp(-dt / tau);

	if (reference_filter_order <= 1)
	{
		refFilter_y1_ = a * refFilter_y1_ + (1.0 - a) * setpoint;
		return refFilter_y1_;
	}

	// Two cascaded 1st-order filters => 2nd order
	refFilter_y1_ = a * refFilter_y1_ + (1.0 - a) * setpoint;
	refFilter_y2_ = a * refFilter_y2_ + (1.0 - a) * refFilter_y1_;
	return refFilter_y2_;
}

double PID_Controller::compute(double err, double dt, double feedforward)
{
	if (dt <= 0.0)
	{
		dt = 1e-3;
	}

	// Proportional:
	const double P = KP * err;

	// Integral:
	i_state_ += KI * err * dt;

	// Derivative (with optional filtering):
	double D = 0;
	if (N > 0)
	{
		// Filtered derivative: d_state tracks the error through a first-order
		// filter, then D = KD * N * (e - d_state). Equivalent to:
		//   d_state += dt * N * (err - d_state)
		//   D = KD * d_state   [where d_state ≈ de/dt when converged]
		// But the standard formulation is:
		//   D_dot = N * (KD * e - D) => D[k] = D[k-1] + dt*N*(KD*err - D[k-1])
		d_state_ += dt * N * (KD * err - d_state_);
		D = d_state_;
	}
	else if (!firstCall_)
	{
		// Standard unfiltered derivative:
		D = KD * (err - e_prev_) / dt;
	}

	e_prev_ = err;
	firstCall_ = false;

	// Unsaturated output:
	double u_unsat = P + i_state_ + D + feedforward;

	// Saturate:
	double u_sat = u_unsat;
	if (max_out > 0.0)
	{
		u_sat = std::clamp(u_sat, -max_out, max_out);
	}

	// Anti-windup via back-calculation:
	if (enable_antiwindup && KP > 1e-12 && max_out > 0.0)
	{
		const double Kaw = KI / KP;
		const double sat_error = u_sat - u_unsat;
		i_state_ += Kaw * sat_error * dt;

		// Recompute with corrected integrator:
		u_unsat = P + i_state_ + D + feedforward;
		u_sat = std::clamp(u_unsat, -max_out, max_out);
	}

	lastOutput_ = u_sat;
	return u_sat;
}

void PID_Controller::reset()
{
	lastOutput_ = 0.0;
	i_state_ = 0.0;
	d_state_ = 0.0;
	e_prev_ = 0.0;
	refFilter_y1_ = 0.0;
	refFilter_y2_ = 0.0;
	firstCall_ = true;
}
