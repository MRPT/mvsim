/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/opengl/opengl_frwds.h>
#include <mvsim/basic_types.h>
#include <cstdint>
#include <memory>

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
	virtual void guiUpdate(mrpt::opengl::COpenGLScene& scene);

	World* getWorldObject() { return m_world; }
	const World* getWorldObject() const { return m_world; }

   protected:
	World* m_world;

	/** If not empty, will override the derived-class visualization for this
	 * object. */
	std::shared_ptr<mrpt::opengl::CSetOfObjects> m_customVisual;
	int32_t m_customVisualId = -1;

	virtual void internalGuiUpdate(mrpt::opengl::COpenGLScene& scene) = 0;
};
}  // namespace mvsim
