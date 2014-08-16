/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/WorldElements/OccupancyGridMap.h>

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>
#include <rapidxml.hpp>

using namespace rapidxml;
using namespace mv2dsim;
using namespace mrpt::slam;
using namespace std;

OccupancyGridMap::OccupancyGridMap(World*parent,const rapidxml::xml_node<char> *root) :
	WorldElementBase(parent),
	m_gui_uptodate(false)
{
	loadConfigFrom(root);
}

OccupancyGridMap::~OccupancyGridMap()
{
}

void OccupancyGridMap::loadConfigFrom(const rapidxml::xml_node<char> *root)
{
	m_gui_uptodate=false;

	// <file>FILENAME.{png,gridmap}</file>
	xml_node<> *xml_file = root->first_node("file");
	if (!xml_file || !xml_file->value())
		throw std::runtime_error("Error: <file></file> XML entry not found inside gridmap node!");

	const string sFile = xml_file->value();
	const string sFileExt = mrpt::system::extractFileExtension(sFile, true /*ignore gz*/);

	// MRPT gridmaps format:
	if ( sFileExt=="gridmap")
	{
		mrpt::utils::CFileGZInputStream f(sFile);
		f >> m_grid;
	}
	else
	// Assume it's an image:
	{
		float xcenterpixel=-1,ycenterpixel=-1;
		float resolution=0.10;
		m_grid.loadFromBitmapFile(sFile,resolution,xcenterpixel,ycenterpixel);
	}

}

void OccupancyGridMap::gui_update( mrpt::opengl::COpenGLScene &scene)
{
	using namespace mrpt::math;

	// 1st time call?? -> Create objects
	if (!m_gl_grid)
	{
		m_gl_grid = mrpt::opengl::CSetOfObjects::Create();
		scene.insert(m_gl_grid);
	}

	// 1st call OR gridmap changed?
	if (!m_gui_uptodate)
	{
		m_grid.getAs3DObject(m_gl_grid);
		m_gui_uptodate=true;
	}
}
