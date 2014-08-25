/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/Wheel.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/stock_objects.h>
#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mv2dsim;
using namespace std;

Wheel::Wheel() :
	x(.0),y(-.5),yaw(.0),
	diameter(.4),width(.2),
	mass(2.0),
	color(0xff323232),
	phi(.0), w(.0)
{
	MRPT_TODO("Inertia of cylinder");
	Iyy = 1.0;
}

void Wheel::getAs3DObject(mrpt::opengl::CSetOfObjects &obj)
{
	obj.clear();

	mrpt::opengl::CCylinderPtr gl_wheel = mrpt::opengl::CCylinder::Create( 0.5*diameter,0.5*diameter,this->width, 15, 1);
	gl_wheel->setColor(mrpt::utils::TColorf(color));
	gl_wheel->setPose(mrpt::poses::CPose3D(0,0.5*width,0,  0,0,mrpt::utils::DEG2RAD(90) ));

	mrpt::opengl::CSetOfObjectsPtr gl_wheel_frame = mrpt::opengl::CSetOfObjects::Create();
	gl_wheel_frame->insert(gl_wheel);
	{
		mrpt::opengl::CSetOfObjectsPtr gl_xyz = mrpt::opengl::stock_objects::CornerXYZSimple( 0.75*diameter );
		gl_wheel_frame->insert( gl_xyz );
	}

	obj.setPose( mrpt::math::TPose3D( x,y, 0.5*diameter, yaw, 0.0, 0.0) );

     obj.insert(gl_wheel_frame);
}

void Wheel::loadFromXML(const rapidxml::xml_node<char> *xml_node)
{
	ASSERT_(xml_node)
	// Parse attributes:
	// <l_wheel pos="0.0 -0.5 [OPTIONAL_ANG]" mass="2.0" width="0.10" diameter="0.30" />
	// pos:
     {
     	const rapidxml::xml_attribute<char> * attr = xml_node->first_attribute("pos");
		if (attr && attr->value())
		{
			const std::string sAttr = attr->value();
			vec3 v = parseXYPHI(sAttr, true);
			this->x =v.vals[0];
			this->y =v.vals[1];
			this->yaw =v.vals[2];
		}
     }

	std::map<std::string,TParamEntry> attribs;
	attribs["mass"] = TParamEntry("%lf", &this->mass);
	attribs["width"] = TParamEntry("%lf", &this->width);
	attribs["diameter"] = TParamEntry("%lf", &this->diameter);
	attribs["color"] = TParamEntry("%color", &this->color);
	
	parse_xmlnode_attribs(*xml_node, attribs,"[Wheel]" );

}

