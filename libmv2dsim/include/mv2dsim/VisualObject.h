/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/basic_types.h>

namespace mv2dsim
{
	class World;

	/** Virtual base class for any entity that can be shown in the 3D viewer (or sent out to RViz) */
	class VisualObject
	{
	public:
		VisualObject(World * parent) : m_world(parent) {}
		virtual ~VisualObject() {} 

		/** Must create a new object in the scene and/or update it according to the current state */
		virtual void gui_update( mrpt::opengl::COpenGLScene &scene) = 0;

	protected:
		World * m_world;
	};
}