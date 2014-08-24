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
	I_MAX_ABS(1e3), m_i_term(0) 
{ 
}

/** err = desired-actual, dt=ellapsed time in secs */
double PID_Controller::compute(double err, double dt)
{
	if (dt<=0.0) return 0.0;
	const double p_term = KP * err;
	m_i_term += dt*KI*err;
	m_i_term = std::max( -I_MAX_ABS, std::min(I_MAX_ABS,m_i_term) );
	printf("I=%.04f err=%.03f\n",m_i_term, err);
	return p_term + m_i_term;
}
