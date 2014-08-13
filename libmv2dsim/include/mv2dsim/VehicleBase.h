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
	/** Virtual base class for each vehicle "actor" in the simulation. 
	  * Derived classes implements different dynamical models (Differential, Ackermann,...)
	  */
	class VehicleBase
	{
	public:
		/** Class factory: Creates a vehicle from XML description of type "<vehicle>...</vehicle>".  */
		static VehicleBase* factory( const rapidxml::xml_node<char> *xml_node);
		/// \overload
		static VehicleBase* factory( const std::string &xml_text);

		/** Loads vehicle params from input XML node of type "<vehicle>...</vehicle>". 
		  * See derived classes & documentation for a list of accepted params.  
		  */
		void load_params_from_xml(const rapidxml::xml_node<char> *xml_node);
		/// \overload
		void load_params_from_xml(const std::string &xml_text);
		
	protected:
		/** Parse node <dynamics>: The derived-class part of load_params_from_xml(), also called in factory() */
		virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node) = 0;

		b2World * m_b2_world; 
				
		vec3 m_q;   //!< Last time-step pose (of the ref. point, in global coords)
		vec3 m_dq;  //!< Last time-step velocity (of the ref. point, in global coords)
		vec3 m_ddq; //!< Last time-step acceleration (of the ref. point, in global coords)

		VehicleBase();
	};
}