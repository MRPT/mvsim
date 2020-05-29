/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/Wheel.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/stock_objects.h>
#include <rapidxml.hpp>

#include "xml_utils.h"

#include <mrpt/version.h>
#if MRPT_VERSION < 0x199
#include <mrpt/utils/TColor.h>
using mrpt::utils::DEG2RAD;
using mrpt::utils::TColor;
using mrpt::utils::TColorf;
#else
#include <mrpt/img/TColor.h>
using mrpt::DEG2RAD;
using mrpt::img::TColor;
using mrpt::img::TColorf;
#endif

using namespace mvsim;
using namespace std;

Wheel::Wheel()
	: x(.0),
	  y(-.5),
	  yaw(.0),
	  diameter(.4),
	  width(.2),
	  mass(2.0),
	  Iyy(1.0),
	  color(0xff323232),
	  phi(.0),
	  w(.0)
{
	recalcInertia();
}

void Wheel::getAs3DObject(mrpt::opengl::CSetOfObjects& obj)
{
	obj.clear();

	auto gl_wheel = mrpt::opengl::CCylinder::Create(
		0.5 * diameter, 0.5 * diameter, this->width, 15);
	gl_wheel->setColor(TColorf(color));
	gl_wheel->setPose(
		mrpt::poses::CPose3D(0, 0.5 * width, 0, 0, 0, DEG2RAD(90)));

	auto gl_wheel_frame = mrpt::opengl::CSetOfObjects::Create();
	gl_wheel_frame->insert(gl_wheel);
	{
		mrpt::opengl::CSetOfObjects::Ptr gl_xyz =
			mrpt::opengl::stock_objects::CornerXYZSimple(0.9 * diameter, 2.0);
		gl_wheel_frame->insert(gl_xyz);
	}

	obj.setPose(mrpt::math::TPose3D(x, y, 0.5 * diameter, yaw, 0.0, 0.0));

	obj.insert(gl_wheel_frame);
}

void Wheel::loadFromXML(const rapidxml::xml_node<char>* xml_node)
{
	ASSERT_(xml_node);
	// Parse attributes:
	// <l_wheel pos="0.0 -0.5 [OPTIONAL_ANG]" mass="2.0" width="0.10"
	// diameter="0.30" />
	// pos:
	{
		const rapidxml::xml_attribute<char>* attr =
			xml_node->first_attribute("pos");
		if (attr && attr->value())
		{
			const std::string sAttr = attr->value();
			vec3 v = parseXYPHI(sAttr, true);
			this->x = v.vals[0];
			this->y = v.vals[1];
			this->yaw = v.vals[2];
		}
	}

	// Detect if inertia is manually set:
	const double INERTIA_NOT_SET = -1.;
	this->Iyy = INERTIA_NOT_SET;

	std::map<std::string, TParamEntry> attribs;
	attribs["mass"] = TParamEntry("%lf", &this->mass);
	attribs["width"] = TParamEntry("%lf", &this->width);
	attribs["diameter"] = TParamEntry("%lf", &this->diameter);
	attribs["color"] = TParamEntry("%color", &this->color);
	attribs["inertia"] = TParamEntry("%lf", &this->Iyy);

	parse_xmlnode_attribs(*xml_node, attribs, "[Wheel]");

	// If not manually overrided, calc automatically:
	if (Iyy == INERTIA_NOT_SET) this->recalcInertia();
}

// Recompute Iyy from mass, diameter and height.
void Wheel::recalcInertia()
{
	// Iyy = m*r^2 / 2
	Iyy = mass * (0.25 * diameter * diameter) * 0.5;
}
