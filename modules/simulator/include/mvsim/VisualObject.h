/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/optional_ref.h>
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
	VisualObject(
		World* parent, bool insertCustomVizIntoViz = true,
		bool insertCustomVizIntoPhysical = true)
		: world_(parent),
		  insertCustomVizIntoViz_(insertCustomVizIntoViz),
		  insertCustomVizIntoPhysical_(insertCustomVizIntoPhysical)
	{
	}

	virtual ~VisualObject();

	/** This creates a new object in the scene and/or update it according to the
	 * current state of the object. If none of the scenes are passed, the poses
	 * of existing visual objects are updated, but no new ones are created.
	 */
	virtual void guiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical);

	World* getWorldObject() { return world_; }
	const World* getWorldObject() const { return world_; }

	void customVisualVisible(const bool visible);
	bool customVisualVisible() const;

	/** Returns bounding boxes, as loaded by parseVisual() from an XML config
	 * file. */
	void getVisualModelBoundingBox(
		mrpt::math::TPoint3D& bbmin, mrpt::math::TPoint3D& bbmax) const
	{
		bbmin = viz_bbmin_;
		bbmax = viz_bbmax_;
	}
	void showBoundingBox(bool show);

	static void FreeOpenGLResources();

   protected:
	bool parseVisual(const rapidxml::xml_node<char>* visual_node);

	World* world_;

	/** If not empty, will override the derived-class visualization for this
	 * object. */
	std::shared_ptr<mrpt::opengl::CSetOfObjects> glCustomVisual_;
	std::shared_ptr<mrpt::opengl::CSetOfObjects> glBoundingBox_;
	int32_t glCustomVisualId_ = -1;

	const bool insertCustomVizIntoViz_ = true;
	const bool insertCustomVizIntoPhysical_ = true;

	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
		bool childrenOnly = false) = 0;

   private:
	mrpt::math::TPoint3D viz_bbmin_{-1.0, -1.0, .0}, viz_bbmax_{1.0, 1.0, 1.0};
};
}  // namespace mvsim
