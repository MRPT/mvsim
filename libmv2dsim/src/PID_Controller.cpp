/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/PID_Controller.h>
#include <algorithm> // max

using namespace mv2dsim;


PID_Controller::PID_Controller() :
	KP(1.0),KI(.0),KD(.0),
	I_MAX_ABS(10.), max_out(0),m_i_term(0),m_last_err(0)
{
}

/** err = desired-actual, dt=ellapsed time in secs */
double PID_Controller::compute(double err, double dt)
{
	// P:
	const double p_term = KP * err;
	// I:
	m_i_term += dt*KI*err;
	m_i_term = std::max( -I_MAX_ABS, std::min(I_MAX_ABS,m_i_term) );
	// D:
	const double d_term = dt>0 ? KD * (err-m_last_err)/dt : 0.0;
	m_last_err = err;

	// PID:
	double ret = p_term + m_i_term + d_term;

	// Clamp:
	if (ret>max_out) ret = max_out; else if (ret<-max_out) ret = -max_out;

	//printf("I=%.04f err=%.03f => ret:%.02f\n",m_i_term, err, ret);
	return ret;
}
