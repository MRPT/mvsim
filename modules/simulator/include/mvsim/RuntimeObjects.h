/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>

#include <cstddef>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace mvsim
{
class World;

/** Description of a visual-only object that can be spawned, moved or removed
 *  while the simulation runs: it has no physics (no collisions, intangible),
 *  but it is rendered in the GUI and, optionally, seen by camera and RGB-D
 *  sensors (e.g. marks drawn on the ground, per-trial targets, debug overlays).
 *
 *  \ingroup mvsim_simulator_module
 */
struct RuntimeObjectDescription
{
	enum class Shape : uint8_t
	{
		Box = 0,  //!< size: (lx, ly, lz), centered at the pose
		Cylinder,  //!< size: (diameter, -, height), base at the pose
		Sphere,	 //!< size: (diameter, -, -), centered at the pose
		Rectangle,	//!< Flat decal. size: (lx, ly, -), centered at the pose
		Disk,  //!< Flat decal. size: (diameter, -, -)
		Polygon,  //!< Flat decal: convex `polygon` in the local XY plane
		Triangles,	//!< Arbitrary mesh: `points` holds 3 local points per triangle
		Lines  //!< Line segments: `points` holds 2 local points per segment.
			   //!< size.x is the line width (pixels).
	};

	/** Unique name. Spawning an object with an existing name replaces it. */
	std::string name;

	Shape shape = Shape::Box;

	/** Pose in world coordinates. For flat decals, the local XY plane is the
	 * decal plane. See also `on_ground`. */
	mrpt::math::TPose3D pose = mrpt::math::TPose3D::Identity();

	mrpt::math::TPoint3D size = {0.1, 0.1, 0.1};

	mrpt::img::TColor color = {0xff, 0x00, 0x00, 0xff};

	/** Vertices for Polygon (convex) shapes, in local coordinates. */
	std::vector<mrpt::math::TPoint2D> polygon;

	/** Points for Triangles and Lines shapes, in local coordinates. */
	std::vector<mrpt::math::TPoint3D> points;

	/** Optional per-point colors for Triangles (same length as `points`). */
	std::vector<mrpt::img::TColor> point_colors;

	/** Optional image file for Rectangle decals (path relative to the world
	 * file directory, or absolute). */
	std::string texture;

	/** If false, it is only shown in the GUI (an overlay), not seen by
	 * sensors. */
	bool visible_to_sensors = true;

	/** If true, `pose.z` is interpreted as a height over the ground under
	 * (pose.x, pose.y), evaluated at spawn time. */
	bool on_ground = false;
};

/** Thread-safe container of runtime visual objects (see
 * RuntimeObjectDescription). Methods can be called from any thread; the
 * changes are applied to the 3D scenes by the GUI/rendering thread right
 * before the next rendering of sensors or the GUI.
 *
 *  \ingroup mvsim_simulator_module
 */
class RuntimeObjects
{
   public:
	explicit RuntimeObjects(World& parent) : parent_(parent) {}

	/** Creates or replaces objects (by name). Throws on invalid input. */
	void spawn(const std::vector<RuntimeObjectDescription>& objs);

	void spawn(const RuntimeObjectDescription& obj) { spawn(std::vector{obj}); }

	/** Changes the pose of an existing object. \return false if not found */
	bool setPose(const std::string& name, const mrpt::math::TPose3D& pose);

	/** \return The object pose, or empty if not found */
	std::optional<mrpt::math::TPose3D> getPose(const std::string& name) const;

	/** Removes objects by name. \return Number of removed objects */
	size_t remove(const std::vector<std::string>& names);

	/** Removes all objects whose name starts with `prefix` (all if empty).
	 * \return Number of removed objects */
	size_t removeByPrefix(const std::string& prefix);

	size_t size() const;

	/** Name and pose of all the objects */
	std::vector<std::pair<std::string, mrpt::math::TPose3D>> poses() const;

	/** Applies pending changes to the scenes. Call from the rendering thread
	 * only, with the physical scene mutex locked. */
	void guiUpdate(mrpt::viz::Scene& viz, mrpt::viz::Scene& physical);

   private:
	World& parent_;

	struct Entry
	{
		RuntimeObjectDescription desc;
		mrpt::viz::CVisualObject::Ptr gl;
		bool needsRebuild = true;
		bool needsPose = true;
	};

	mutable std::mutex mtx_;
	std::map<std::string, Entry> objects_;
	std::vector<mrpt::viz::CVisualObject::Ptr> pendingRemovals_;
	bool anyChange_ = false;

	mrpt::viz::CSetOfObjects::Ptr glContainerViz_, glContainerPhysical_;

	void removeEntry(std::map<std::string, Entry>::iterator it);
};

}  // namespace mvsim
