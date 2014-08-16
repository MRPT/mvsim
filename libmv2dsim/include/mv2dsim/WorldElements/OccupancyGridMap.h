/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/WorldElements/WorldElementBase.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/opengl/CSetOfObjects.h>

namespace mv2dsim
{
	class OccupancyGridMap : public WorldElementBase
	{
	public:
		OccupancyGridMap(World*parent,const rapidxml::xml_node<char> *root);
		virtual ~OccupancyGridMap();

		virtual void loadConfigFrom(const rapidxml::xml_node<char> *root) ; //!< See docs in base class
		virtual void gui_update( mrpt::opengl::COpenGLScene &scene); //!< See docs in base class

		virtual void simul_pre_timestep(const TSimulContext &context); //!< See docs in base class
		virtual void simul_post_timestep(const TSimulContext &context); //!< See docs in base class

	protected:
		mrpt::slam::COccupancyGridMap2D  m_grid;

		bool m_gui_uptodate; //!< Whether m_gl_grid has to be updated upon next call of gui_update()
		mrpt::opengl::CSetOfObjectsPtr m_gl_grid;

	};
}
