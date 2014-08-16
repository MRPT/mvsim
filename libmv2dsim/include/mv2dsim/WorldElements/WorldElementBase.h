/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/VisualObject.h>

namespace mv2dsim
{
	class WorldElementBase : public VisualObject
	{
	public:
		WorldElementBase(World*parent) : VisualObject(parent) { }
		virtual ~WorldElementBase() { }

		/** Class factory: Creates a world element from XML description of type "<world:*>...</world:*>".  */
		static WorldElementBase* factory(World* parent, const rapidxml::xml_node<char> *xml_node);

		virtual void loadConfigFrom(const rapidxml::xml_node<char> *root) = 0;

	protected:

	};

}
