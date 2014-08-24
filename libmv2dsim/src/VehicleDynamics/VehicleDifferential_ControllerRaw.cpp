/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleDynamics/VehicleDifferential.h>
//#include <mv2dsim/World.h>
//#include <rapidxml.hpp>

using namespace mv2dsim;
using namespace std;

// See base class docs
void DynamicsDifferential::ControllerRawForces::control_step(
	const DynamicsDifferential::TControllerInput &ci, 
	DynamicsDifferential::TControllerOutput &co)
{
	co.wheel_force_l = this->setpoint_wheel_force_l;
	co.wheel_force_r = this->setpoint_wheel_force_r;
}
