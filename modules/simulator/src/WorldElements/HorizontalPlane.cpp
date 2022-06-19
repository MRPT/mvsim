/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/system/filesystem.h>
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

	TParameterDefinitions params;
	params["color"] = TParamEntry("%color", &m_color);

	params["x_min"] = TParamEntry("%f", &m_x_min);
	params["x_max"] = TParamEntry("%f", &m_x_max);
	params["y_min"] = TParamEntry("%f", &m_y_min);
	params["y_max"] = TParamEntry("%f", &m_y_max);
	params["z"] = TParamEntry("%f", &m_z);
	params["cull_face"] = TParamEntry("%s", &m_cull_faces);

	params["texture"] = TParamEntry("%s", &m_textureFileName);
	params["texture_size_x"] = TParamEntry("%lf", &m_textureSizeX);
	params["texture_size_y"] = TParamEntry("%lf", &m_textureSizeY);

	parse_xmlnode_children_as_param(*root, params);
}

void HorizontalPlane::internalGuiUpdate(
	mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
	[[maybe_unused]] bool childrenOnly)
{
	using namespace mrpt::math;

	// 1st call? (w/o texture)
	if (!m_gl_plane && m_textureFileName.empty())
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
	// 1st call? (with texture)
	if (!m_gl_plane_text && !m_textureFileName.empty())
	{
		const std::string localFileName =
			m_world->xmlPathToActualPath(m_textureFileName);
		ASSERT_FILE_EXISTS_(localFileName);

		mrpt::img::CImage texture;
		bool textureReadOk = texture.loadFromFile(localFileName);
		ASSERT_(textureReadOk);

		// Compute (U,V) texture coordinates:
		float u_min = 0;
		float v_min = 0;
		float u_max = (m_x_max - m_x_min) / m_textureSizeX;
		float v_max = (m_y_max - m_y_min) / m_textureSizeY;

		m_gl_plane_text = mrpt::opengl::CSetOfTexturedTriangles::Create();

		{
			mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
			t.vertices[0].xyzrgba.pt = {m_x_min, m_y_min, m_z};
			t.vertices[1].xyzrgba.pt = {m_x_max, m_y_min, m_z};
			t.vertices[2].xyzrgba.pt = {m_x_max, m_y_max, m_z};

			t.vertices[0].uv = {u_min, v_min};
			t.vertices[1].uv = {u_max, v_min};
			t.vertices[2].uv = {u_max, v_max};

			m_gl_plane_text->insertTriangle(t);
		}
		{
			mrpt::opengl::CSetOfTexturedTriangles::TTriangle t;
			t.vertices[0].xyzrgba.pt = {m_x_min, m_y_min, m_z};
			t.vertices[1].xyzrgba.pt = {m_x_max, m_y_max, m_z};
			t.vertices[2].xyzrgba.pt = {m_x_min, m_y_max, m_z};

			t.vertices[0].uv = {u_min, v_min};
			t.vertices[1].uv = {u_max, v_max};
			t.vertices[2].uv = {u_min, v_max};

			m_gl_plane_text->insertTriangle(t);
		}

		m_gl_plane_text->assignImage(texture);

#if MRPT_VERSION >= 0x240
		m_gl_plane_text->cullFaces(
			mrpt::typemeta::TEnumType<mrpt::opengl::TCullFace>::name2value(
				m_cull_faces));
#endif

		viz.insert(m_gl_plane_text);
		physical.insert(m_gl_plane_text);
	}
}
