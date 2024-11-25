/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/optional_ref.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/poses/CPose3D.h>
#include <mvsim/Shape2p5.h>
#include <mvsim/basic_types.h>

#include <cstdint>
#include <memory>

namespace mvsim
{
class World;

/** @addtogroup virtual_interfaces_module  Virtual interfaces
 * \ingroup mvsim_simulator_module
 */

/** Virtual base class for any entity that can be shown in the 3D viewer (or
 * sent out to RViz)
 *  \ingroup virtual_interfaces_module
 */
class VisualObject
{
   public:
	VisualObject(
		World* parent, bool insertCustomVizIntoViz = true, bool insertCustomVizIntoPhysical = true);

	virtual ~VisualObject();

	/** This creates a new object in the scene and/or update it according to the
	 * current state of the object. If none of the scenes are passed, the poses
	 * of existing visual objects are updated, but no new ones are created.
	 */
	virtual void guiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical);

	World* parent() { return world_; }
	const World* parent() const { return world_; }

	void customVisualVisible(const bool visible);
	bool customVisualVisible() const;

	/** Returns the collision shape, if defined (should be for regular entities
	 * after correct initialization). */
	const std::optional<Shape2p5>& collisionShape() const { return collisionShape_; }

	void showCollisionShape(bool show);

	static void FreeOpenGLResources();

	/** Epsilon for geometry checks related to bounding boxes (default:1e-3) */
	static double GeometryEpsilon;

   protected:
	/// Returns true if there is at least one `<visual>...</visual>` entry.
	bool parseVisual(const rapidxml::xml_node<char>& rootNode);
	bool parseVisual(const JointXMLnode<>& rootNode);

	World* world_;

	/** If not empty, will override the derived-class visualization for this
	 * object. */
	std::shared_ptr<mrpt::opengl::CSetOfObjects> glCustomVisual_;
	std::shared_ptr<mrpt::opengl::CSetOfObjects> glCollision_;
	int32_t glCustomVisualId_ = -1;

	const bool insertCustomVizIntoViz_ = true;
	const bool insertCustomVizIntoPhysical_ = true;

	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
		bool childrenOnly = false) = 0;

	void addCustomVisualization(
		const mrpt::opengl::CRenderizable::Ptr& glModel, const mrpt::poses::CPose3D& modelPose = {},
		const float modelScale = 1.0f, const std::string& modelName = "group",
		const std::optional<std::string>& modelURI = std::nullopt,
		const bool initialShowBoundingBox = false);

	void setCollisionShape(const Shape2p5& cs) { collisionShape_ = cs; }

   private:
	std::optional<Shape2p5> collisionShape_;

	/// Called by parseVisual once per "visual" block.
	bool implParseVisual(const rapidxml::xml_node<char>& visual_node);
};
}  // namespace mvsim
