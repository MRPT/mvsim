/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/ClassFactory.h>
#include <mvsim/Simulable.h>
#include <mvsim/VisualObject.h>

namespace mvsim
{
/** @addtogroup world_elements_module  Simulated world elements
 * \ingroup mvsim_simulator_module
 */

/** Virtual base class for all world simulated "entities" (not vehicles, not blocks)
 *
 * \ingroup world_elements_module
 *
 */
class WorldElementBase : public VisualObject, public Simulable
{
   public:
	using Ptr = std::shared_ptr<WorldElementBase>;

	WorldElementBase(World* parent) : VisualObject(parent), Simulable(parent) {}
	virtual ~WorldElementBase() {}

	WorldElementBase(const WorldElementBase&) = delete;
	WorldElementBase(WorldElementBase&&) = delete;
	WorldElementBase& operator=(const WorldElementBase&) = delete;
	WorldElementBase& operator=(WorldElementBase&&) = delete;

	/** Class factory: Creates a world element from XML description of type
	 * "<element class='*'>...</element>".
	 * Only if xml_node==nullptr, the world element name can be passed in
	 * class_name. Otherwise, class_name is ignored.
	 */
	static Ptr factory(
		World* parent, const rapidxml::xml_node<char>* xml_node, const char* class_name = nullptr);

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) = 0;

	virtual std::optional<std::any> queryProperty(
		[[maybe_unused]] const std::string& propertyName,
		[[maybe_unused]] const mrpt::math::TPoint3D& worldXYZ) const
	{
		return {};
	}

   protected:
};

// Class factory:
using TClassFactory_worldElements =
	ClassFactory<WorldElementBase, World*, const rapidxml::xml_node<char>*>;

extern TClassFactory_worldElements classFactory_worldElements;

#define DECLARES_REGISTER_WORLD_ELEMENT(CLASS_NAME) \
	DECLARES_REGISTER_CLASS2(CLASS_NAME, WorldElementBase, World*, const rapidxml::xml_node<char>*)

#define REGISTER_WORLD_ELEMENT(TEXTUAL_NAME, CLASS_NAME) \
	REGISTER_CLASS2(                                     \
		TClassFactory_worldElements, classFactory_worldElements, TEXTUAL_NAME, CLASS_NAME)
}  // namespace mvsim
