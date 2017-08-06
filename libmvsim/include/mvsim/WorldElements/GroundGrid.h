/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mvsim/WorldElements/WorldElementBase.h>
#include <mrpt/opengl/CGridPlaneXY.h>

namespace mvsim
{
	class GroundGrid : public WorldElementBase
	{
		DECLARES_REGISTER_WORLD_ELEMENT(GroundGrid)
	public:
		GroundGrid(World*parent,const rapidxml::xml_node<char> *root);
		virtual ~GroundGrid();

		virtual void loadConfigFrom(const rapidxml::xml_node<char> *root) ; //!< See docs in base class
		virtual void gui_update( mrpt::opengl::COpenGLScene &scene); //!< See docs in base class

	protected:
		bool m_is_floating;
		std::string m_float_center_at_vehicle_name;
		double m_x_min,m_x_max,m_y_min,m_y_max, m_interval;
		mrpt::utils::TColor m_color;
		double m_line_width;

    mrpt::opengl::CGridPlaneXY::Ptr m_gl_groundgrid;

	};
}
