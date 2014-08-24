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
class b2Body;
class b2Fixture;

#define INVISIBLE_FIXTURE_USER_DATA (reinterpret_cast<void*>(-1))  //!< Used to signal a Box2D fixture as "invisible" to sensors.

namespace rapidxml
{
    // Forward declarations
    template<class Ch> class xml_node;
    template<class Ch> class xml_attribute;
    template<class Ch> class xml_document;
}

namespace mrpt {
	namespace opengl {
		class COpenGLScene;
		class CSetOfObjects;
	}
}

namespace mv2dsim
{
	class World;
	class VehicleBase;

	/** XX */
	struct TSimulContext
	{
		b2World * b2_world;
		double    simul_time; //!< Current time in the simulated world
	};


	/** Vector to store a pose (x,y,yaw), vel (dx,dy,omega) or acc (ddx,ddy,alpha) */
	struct vec3
	{
		double vals[3];
		vec3() {}
		vec3(double x,double y,double th) { vals[0]=x;vals[1]=y;vals[2]=th; }
	};

	/** 2D points or vectors */
	struct vec2
	{
		double vals[2];
		vec2() {}
		vec2(double x,double y) { vals[0]=x;vals[1]=y;}
	};


}
