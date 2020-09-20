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
#include <mrpt/poses/CPose3D.h>
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

	/** Returns bounding boxes, as loaded by parseVisual() from an XML config
	 * file. */
	void getVisualModelBoundingBox(
		mrpt::math::TPoint3D& bbmin, mrpt::math::TPoint3D& bbmax) const
	{
		bbmin = viz_bbmin_;
		bbmax = viz_bbmax_;
	}
	void showBoundingBox(bool show);

   protected:
	bool parseVisual(const rapidxml::xml_node<char>* visual_node);

	World* m_world;

	/** If not empty, will override the derived-class visualization for this
	 * object. */
	std::shared_ptr<mrpt::opengl::CSetOfObjects> m_customVisual;
	int32_t m_customVisualId = -1;

	virtual void internalGuiUpdate(
		mrpt::opengl::COpenGLScene& scene, bool childrenOnly = false) = 0;
	virtual mrpt::poses::CPose3D internalGuiGetVisualPose()
	{
		throw std::runtime_error(
			"internalGuiGetVisualPose: not implemented for this class!");
	}

   private:
	mrpt::math::TPoint3D viz_bbmin_{}, viz_bbmax_{};
};
}  // namespace mvsim
