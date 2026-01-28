/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mvsim/WorldElements/WorldElementBase.h>

#include <any>

namespace mvsim
{
/** A non-visual world element that defines a named property within a 3D axis-aligned bounding box.
 * Useful for simulating GPS-denied areas, friction zones, or logical regions.
 */
class PropertyRegion : public WorldElementBase
{
	DECLARES_REGISTER_WORLD_ELEMENT(PropertyRegion)
   public:
	PropertyRegion(World* parent, const rapidxml::xml_node<char>* root);
	virtual ~PropertyRegion() = default;

	// Non-copyable and non-movable
	PropertyRegion(const PropertyRegion&) = delete;
	PropertyRegion& operator=(const PropertyRegion&) = delete;
	PropertyRegion(PropertyRegion&&) = delete;
	PropertyRegion& operator=(PropertyRegion&&) = delete;

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;

	// The core logic: returns the property value if the point is inside the box
	virtual std::optional<std::any> queryProperty(
		const std::string& propertyName, const mrpt::math::TPoint3D& worldXYZ) const override;

	// No-op for physics and GUI
	virtual void simul_pre_timestep(const TSimulContext&) override {}
	virtual void simul_post_timestep(const TSimulContext&) override {}
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>&,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>&, bool) override
	{
	}

   protected:
	mrpt::math::TPoint3D boxMin_ = {-1.0, -1.0, -1000.0};
	mrpt::math::TPoint3D boxMax_ = {1.0, 1.0, 1000.0};

	std::string propertyName_;
	std::any propertyValue_;
};
}  // namespace mvsim