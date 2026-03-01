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
/** PID controller with optional features:
 *  - Filtered derivative (coefficient N)
 *  - Anti-windup via back-calculation
 *  - Optional reference setpoint filter (1st or 2nd order)
 *
 * \ingroup mvsim_simulator_module
 */
struct PID_Controller
{
	PID_Controller() = default;

	double KP = 1.0, KI = 0, KD = 0;

	/** Derivative filter coefficient. Higher values = less filtering.
	 *  Typical range: 2-20. Set to 0 to disable filtered derivative
	 *  (falls back to standard difference). */
	double N = 0;

	double max_out = 0;	 //!< For clamping (0=no clamp)

	bool enable_antiwindup = false;

	/** Enable low-pass filter on reference (setpoint) before error
	 *  computation. */
	bool enable_reference_filter = false;

	/** Time constant for the reference filter [seconds]. */
	double reference_filter_tau = 0.1;

	/** Order of the reference filter: 1 or 2. */
	int reference_filter_order = 1;

	/** Compute PID output.
	 *  \param[in] err  Error = desired - actual
	 *  \param[in] dt   Elapsed time in seconds
	 *  \param[in] feedforward  Optional feedforward term added to the output
	 *  \return Control output (torque, force, etc.)
	 */
	double compute(double err, double dt, double feedforward = 0);

	/** Filter a setpoint through the reference filter.
	 *  Call this on the setpoint *before* computing the error, if
	 *  enable_reference_filter is true.
	 *  \return Filtered setpoint value.
	 */
	double filterReference(double setpoint, double dt);

	/** Reset internal status to all zeros (gains and settings remain
	 * unmodified) */
	void reset();

   private:
	double lastOutput_ = 0;
	double i_state_ = 0;  //!< Integrator accumulator
	double d_state_ = 0;  //!< Filtered derivative state
	double e_prev_ = 0;	 //!< Previous error (for unfiltered derivative)
	double refFilter_y1_ = 0;  //!< Reference filter 1st stage state
	double refFilter_y2_ = 0;  //!< Reference filter 2nd stage state
	bool firstCall_ = true;
};
}  // namespace mvsim
