/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mvsim/PID_Controller.h>

using namespace mvsim;


PID_Controller::PID_Controller() :
  KP(1.0), KI(.0), KD(.0),
  max_out(0),
  lastOutput(0),
  e_n(0),
  e_n_1(0),
  e_n_2(0)
{
}

/** err = desired-actual, dt=ellapsed time in secs */
double PID_Controller::compute(double err, double dt)
{
  e_n_2 = e_n_1;
  e_n_1 = e_n;
  e_n = err;

  double output = lastOutput
             + KP * (e_n - e_n_1)
             + KI *  e_n * dt
             + KD * (e_n - 2 * e_n_1 + e_n_2) / dt;

  // prevent integral windup
  if(max_out != 0.0 && (output < -max_out || output > max_out))
  {
    output -= KI * e_n * dt;
  }

  lastOutput = output;

  if(max_out != 0.0)
  {
    if(output < -max_out) output = -max_out;
    if(output >  max_out) output =  max_out;
  }

  return output;
}
