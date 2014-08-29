/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/WorldElements/WorldElementBase.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/synch/CCriticalSection.h>

namespace mv2dsim
{
	class ElevationMap : public WorldElementBase
	{
		DECLARES_REGISTER_WORLD_ELEMENT(ElevationMap)
	public:
		ElevationMap(World*parent,const rapidxml::xml_node<char> *root);
		virtual ~ElevationMap();

		virtual void loadConfigFrom(const rapidxml::xml_node<char> *root) ; //!< See docs in base class
		virtual void gui_update( mrpt::opengl::COpenGLScene &scene); //!< See docs in base class

		virtual void simul_pre_timestep(const TSimulContext &context); //!< See docs in base class
		virtual void simul_post_timestep(const TSimulContext &context); //!< See docs in base class

	protected:
		/** This object holds both, the mesh data, and is in charge of 3D rendering. */
		mrpt::opengl::CMeshPtr  m_gl_mesh;
		bool  m_first_scene_rendering;
		double m_resolution;

	};
}
