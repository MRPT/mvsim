/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/WorldElements/GroundGrid.h>
#include <mv2dsim/World.h>
#include "xml_utils.h"

#include <mrpt/opengl/COpenGLScene.h>
#include <rapidxml.hpp>

using namespace rapidxml;
using namespace mv2dsim;
using namespace std;


GroundGrid::GroundGrid(World*parent,const rapidxml::xml_node<char> *root) :
	WorldElementBase(parent),
	m_is_floating(true)
{
	// Create opengl object: in this class, we'll store most state data directly in the mrpt::opengl object.
	loadConfigFrom(root);
}

GroundGrid::~GroundGrid()
{
}

void GroundGrid::loadConfigFrom(const rapidxml::xml_node<char> *root)
{
	if (!root) return; // Assume defaults

	std::map<std::string,TParamEntry> params;
	params["floating"] = TParamEntry("%bool", &m_is_floating);
	params["floating_focus"] = TParamEntry("%s", &m_float_center_at_vehicle_name);

	parse_xmlnode_children_as_param(*root,params);
}

void GroundGrid::gui_update( mrpt::opengl::COpenGLScene &scene)
{
	using namespace mrpt::math;

	// 1st call OR gridmap changed?
	if (!m_gl_groundgrid)
	{
		m_gl_groundgrid = mrpt::opengl::CGridPlaneXY::Create();		
		scene.insert(m_gl_groundgrid);
	}

	// Update:

}

