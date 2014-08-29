/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/WorldElements/ElevationMap.h>
#include <mv2dsim/World.h>
#include "xml_utils.h"

#include <mrpt/opengl/COpenGLScene.h>
//#include <mrpt/system/filesystem.h>
//#include <mrpt/utils/CObject.h>
#include <rapidxml.hpp>

using namespace rapidxml;
using namespace mv2dsim;
using namespace std;


ElevationMap::ElevationMap(World*parent,const rapidxml::xml_node<char> *root) :
	WorldElementBase(parent),
	m_first_scene_rendering(true),
	m_resolution(1.0)
{
	loadConfigFrom(root);
}

ElevationMap::~ElevationMap()
{
}

void ElevationMap::loadConfigFrom(const rapidxml::xml_node<char> *root)
{
	// Other general params:
	std::map<std::string,TParamEntry> params;
	std::string sElevationImgFile;
	params["elevation_image"] = TParamEntry("%s", &sElevationImgFile);
	double img_min_z=0.0, img_max_z=5.0;
	params["elevation_image_min_z"] = TParamEntry("%lf", &img_min_z);
	params["elevation_image_max_z"] = TParamEntry("%lf", &img_max_z);
	
	mrpt::utils::TColor mesh_color;
	params["mesh_color"] = TParamEntry("%color", &mesh_color);

	params["resolution"] = TParamEntry("%lf", &m_resolution);
	
	parse_xmlnode_children_as_param(*root,params);


	// Load elevation & color data:
	mrpt::utils::CImage mesh_image;
	bool has_mesh_image = false;

	mrpt::math::CMatrixFloat elevation_data;
	if (!sElevationImgFile.empty())
	{
		if (!mesh_image.loadFromFile(sElevationImgFile))
			throw std::runtime_error(mrpt::format("[ElevationMap] ERROR: Cannot read image '%s'",sElevationImgFile.c_str()));

		//has_mesh_image = true;

		// Convert to grayscale img for elevation:
		mrpt::utils::CImage imgElev(mesh_image, mrpt::utils::FAST_REF_OR_CONVERT_TO_GRAY);
		imgElev.getAsMatrix(elevation_data);  // Get image normalized in range [0,1]
		
		// Scale: [0,1] => [min_z,max_z]
		ASSERT_(img_min_z!=img_max_z)
		elevation_data.adjustRange(img_min_z,img_max_z);
	}
	else
	{
		MRPT_TODO("Imgs or txt matrix")
	}


	// Build mesh:
	m_gl_mesh = mrpt::opengl::CMesh::Create();

	m_gl_mesh->enableTransparency(false);

	if (has_mesh_image)
	{
		ASSERT_EQUAL_(mesh_image.getWidth(),(size_t)elevation_data.cols())
		ASSERT_EQUAL_(mesh_image.getHeight(),(size_t)elevation_data.rows())
		m_gl_mesh->assignImageAndZ(mesh_image, elevation_data);
	}
	else
	{
		m_gl_mesh->setZ(elevation_data);
		m_gl_mesh->setColor_u8(mesh_color);
	}
	
	// Extension: X,Y
	const double LX = elevation_data.cols() * m_resolution;
	const double LY = elevation_data.rows() * m_resolution;
	m_gl_mesh->setGridLimits(-0.5*LX, 0.5*LX, -0.5*LY, 0.5*LY);
		
}

void ElevationMap::gui_update( mrpt::opengl::COpenGLScene &scene)
{
	using namespace mrpt::math;

	ASSERTMSG_(m_gl_mesh, "ERROR: Can't render Mesh before loading it! Have you called loadConfigFrom() first?")

	// 1st time call?? -> Create objects
	if (m_first_scene_rendering)
	{
		m_first_scene_rendering=false;
		SCENE_INSERT_Z_ORDER(scene,0, m_gl_mesh);
	}

}

void ElevationMap::simul_pre_timestep(const TSimulContext &context)
{

}

void ElevationMap::simul_post_timestep(const TSimulContext &context)
{
	MRPT_TODO("Save all elements positions in prestep, then here scale their movements * cos(angle)")

}
