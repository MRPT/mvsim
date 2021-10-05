/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/version.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/HorizontalPlane.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace rapidxml;
using namespace mvsim;
using namespace std;

HorizontalPlane::HorizontalPlane(
	World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent)
{
	// Create opengl object: in this class, we'll store most state data directly
	// in the mrpt::opengl object.
	loadConfigFrom(root);
}

HorizontalPlane::~HorizontalPlane() {}

void HorizontalPlane::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	if (!root) return;	// Assume defaults

	MRPT_TODO("Allow loading texture, and texture coordinates");

	TParameterDefinitions params;
	params["color"] = TParamEntry("%color", &m_color);

	params["x_min"] = TParamEntry("%lf", &m_x_min);
	params["x_max"] = TParamEntry("%lf", &m_x_max);
	params["y_min"] = TParamEntry("%lf", &m_y_min);
	params["y_max"] = TParamEntry("%lf", &m_y_max);
	params["z"] = TParamEntry("%lf", &m_z);
	params["cull_face"] = TParamEntry("%s", &m_cull_faces);

	parse_xmlnode_children_as_param(*root, params);
}

void HorizontalPlane::internalGuiUpdate(
	mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
	bool childrenOnly)
{
	using namespace mrpt::math;

	// 1st call?
	if (!m_gl_plane)
	{
		m_gl_plane = mrpt::opengl::CTexturedPlane::Create();
		m_gl_plane->setPlaneCorners(m_x_min, m_x_max, m_y_min, m_y_max);
		m_gl_plane->setLocation(0, 0, m_z);

		m_gl_plane->setColor_u8(m_color);

#if MRPT_VERSION >= 0x240
		m_gl_plane->cullFaces(
			mrpt::typemeta::TEnumType<mrpt::opengl::TCullFace>::name2value(
				m_cull_faces));
#endif

		viz.insert(m_gl_plane);
		physical.insert(m_gl_plane);
	}
}
