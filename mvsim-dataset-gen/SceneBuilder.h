/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/World.h>
#include <mvsim/raytracer/RayScene.h>

#include <set>
#include <string>

namespace mvsim_dataset_gen
{
/** Converts a loaded, headless `mvsim::World` into a ray-traceable
 * `mvsim::rt::RayScene`.
 *
 * Only analytic geometry is supported: `<block>` shapes (explicit `<shape>`
 * polygons or `<geometry type="cylinder|sphere|box|ramp|
 * semi_cylinder_bump">`), `<element class="horizontal_plane">`,
 * `<element class="vertical_plane">` (door/window openings excluded, as
 * gaps), and `<element class="elevation_map">`. Anything else (occupancy
 * grids, point clouds, actors, extra vehicles, `<shape_from_visual/>`
 * meshes) is either skipped (purely visual elements: ground_grid, sky_box)
 * or rejected.
 */
class SceneBuilder
{
   public:
	struct Options
	{
		/** If true, unsupported entities are skipped with a warning on
		 * stderr instead of throwing. */
		bool allowUnsupported = false;

		/** Names of the blocks to leave out of the scene (e.g. those that
		 * move during the run, see buildBlock()). */
		std::set<std::string> excludedBlocks;
	};

	/** Builds a scene from every block and world element in `world`,
	 * excluding `egoVehicleName` (which never has ray-traceable geometry of
	 * its own in this tool: it is the sensor platform, not an obstacle).
	 * Throws on unsupported geometry unless `opts.allowUnsupported`. */
	static mvsim::rt::RayScene build(
		const mvsim::World& world, const std::string& egoVehicleName, const Options& opts);

	/** Builds a scene with one block only, in the block's own frame (that
	 * is, as if its pose were the identity), whatever its `<init_pose>`:
	 * for blocks that move during the run. */
	static mvsim::rt::RayScene buildBlock(const mvsim::Block& block);
};

}  // namespace mvsim_dataset_gen
