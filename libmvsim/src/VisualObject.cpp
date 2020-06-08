/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mvsim/VisualObject.h>
#include <atomic>

using namespace mvsim;

static std::atomic_int32_t g_uniqueCustomVisualId = 0;

void VisualObject::guiUpdate(mrpt::opengl::COpenGLScene& scene)
{
	using namespace std::string_literals;

	if (m_customVisual)
	{
		// Assign a unique ID on first call:
		if (m_customVisualId < 0)
		{
			// Assign a unique name, so we can localize the object in the scene
			// if needed.
			m_customVisualId = g_uniqueCustomVisualId++;
			const auto name = "_autoViz"s + std::to_string(m_customVisualId);
			m_customVisual->setName(name);
			// Add to the 3D scene:
			scene.insert(m_customVisual);
		}
	}
	else
	{
		// Default:
		internalGuiUpdate(scene);
	}
}
