/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/basic_types.h>

namespace mv2dsim
{
	/** Virtual base for controllers of vehicles of any type (template) */
	template <class VEH_DYNAMICS>
	class ControllerBaseTempl
	{
	public:
		ControllerBaseTempl(VEH_DYNAMICS &veh) : m_veh(veh) {}
		virtual ~ControllerBaseTempl() {}

		/** The core of the controller: will be called at each timestep before the numeric integration of dynamical eqs */
		virtual void control_step(const typename VEH_DYNAMICS::TControllerInput &ci, typename VEH_DYNAMICS::TControllerOutput &co) =0;

		/** Override to load class-specific options from the <controller> node */
		virtual void load_config(const rapidxml::xml_node<char>&node ) {  /*default: do nothing*/ }

	protected:
		VEH_DYNAMICS &m_veh;
	};
}
