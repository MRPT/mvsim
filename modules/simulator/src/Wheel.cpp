/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/img/TColor.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/stock_objects.h>
#include <mvsim/Wheel.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

Wheel::Wheel() { recalcInertia(); }

void Wheel::getAs3DObject(mrpt::opengl::CSetOfObjects& obj)
{
	obj.clear();

	auto gl_wheel = mrpt::opengl::CCylinder::Create(
		0.5 * diameter, 0.5 * diameter, this->width, 15);
	gl_wheel->setColor_u8(color);
	gl_wheel->setPose(
		mrpt::poses::CPose3D(0, 0.5 * width, 0, 0, 0, mrpt::DEG2RAD(90)));

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
			mrpt::math::TPose2D v = parseXYPHI(sAttr, true);
			this->x = v.x;
			this->y = v.y;
			this->yaw = v.phi;
		}
	}

	// Detect if inertia is manually set:
	const double INERTIA_NOT_SET = -1.;
	this->Iyy = INERTIA_NOT_SET;

	parse_xmlnode_attribs(*xml_node, m_params, {}, "[Wheel]");

	// If not manually overrided, calc automatically:
	if (Iyy == INERTIA_NOT_SET) this->recalcInertia();
}

// Recompute Iyy from mass, diameter and height.
void Wheel::recalcInertia()
{
	// Iyy = m*r^2 / 2
	Iyy = mass * (0.25 * diameter * diameter) * 0.5;
}
