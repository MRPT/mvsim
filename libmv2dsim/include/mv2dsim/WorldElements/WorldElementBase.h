/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

namespace mv2dsim
{
	class World;

	class WorldElementBase
	{
	public:
		WorldElementBase(World*parent) : m_parent(parent) 
		{
		}

	protected:
		World *m_parent;

	};

}