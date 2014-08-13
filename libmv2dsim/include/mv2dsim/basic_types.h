/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#pragma once

// Misc. types & forwards declarations

#include <string>

class b2World; 

namespace rapidxml
{
    // Forward declarations
    template<class Ch> class xml_node;
    template<class Ch> class xml_attribute;
    template<class Ch> class xml_document;
}

namespace mv2dsim
{
	/** Vector to store a pose (x,y,yaw), vel (dx,dy,omega) or acc (ddx,ddy,alpha) */
	struct vec3
	{
		double vals[3];
		vec3() {}
		vec3(double x,double y,double th) { vals[0]=x;vals[1]=y;vals[2]=th; }
	};

}
