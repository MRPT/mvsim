/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/VisualObject.h>
#include <mvsim/Simulable.h>
#include <mvsim/ClassFactory.h>

namespace mvsim
{
class WorldElementBase : public VisualObject, public Simulable
{
   public:
	WorldElementBase(World* parent) : VisualObject(parent) {}
	virtual ~WorldElementBase() {}
	/** Class factory: Creates a world element from XML description of type
	 * "<element class='*'>...</element>".
	  * Only if xml_node==NULL, the world element name can be passed in
	 * class_name. Otherwise, class_name is ignored.
	  */
	static WorldElementBase* factory(
		World* parent, const rapidxml::xml_node<char>* xml_node,
		const char* class_name = NULL);

	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) = 0;

   protected:
};

// Class factory:
typedef ClassFactory<WorldElementBase, World*, const rapidxml::xml_node<char>*>
	TClassFactory_worldElements;
extern TClassFactory_worldElements classFactory_worldElements;

#define DECLARES_REGISTER_WORLD_ELEMENT(CLASS_NAME) \
	DECLARES_REGISTER_CLASS2(                       \
		CLASS_NAME, WorldElementBase, World*, const rapidxml::xml_node<char>*)

#define REGISTER_WORLD_ELEMENT(TEXTUAL_NAME, CLASS_NAME)                       \
	REGISTER_CLASS2(                                                           \
		TClassFactory_worldElements, classFactory_worldElements, TEXTUAL_NAME, \
		CLASS_NAME)
}
