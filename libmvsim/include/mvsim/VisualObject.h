/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/basic_types.h>

namespace mvsim
{
class World;

/** Virtual base class for any entity that can be shown in the 3D viewer (or
 * sent out to RViz) */
class VisualObject
{
   public:
	VisualObject(World* parent) : m_world(parent) {}
	virtual ~VisualObject() {}
	/** Must create a new object in the scene and/or update it according to the
	 * current state */
	virtual void gui_update(mrpt::opengl::COpenGLScene& scene) = 0;

	World* getWorldObject() { return m_world; }
	const World* getWorldObject() const { return m_world; }
   protected:
	World* m_world;
};
}

/** Example usage: SCENE_INSERT_Z_ORDER(scene, 0, my_gl_obj );  */
#define SCENE_INSERT_Z_ORDER(_SCENE, _ZORDER_INDEX, _OBJ_TO_INSERT) \
	std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(         \
		_SCENE.getByName("level_" #_ZORDER_INDEX))                  \
		->insert(_OBJ_TO_INSERT)
