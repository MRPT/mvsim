/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/basic_types.h>  // fwrd decls.
#include <mv2dsim/ClassFactory.h>
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

namespace mv2dsim
{

	/** Virtual base class for all friction models */
	class FrictionBase
	{
	public:
		FrictionBase(VehicleBase & my_vehicle);
		virtual ~FrictionBase();

		/** Class factory: Creates a friction object from XML description of type "<friction>...</friction>".  */
		static FrictionBase* factory(VehicleBase & parent, const rapidxml::xml_node<char> *xml_node);

		virtual void update_step(const TSimulContext &context)=0;

	protected:
		World* m_world;
		VehicleBase & m_my_vehicle;
	};

	typedef stlplus::smart_ptr<FrictionBase> FrictionBasePtr;

	// Class factory:
	typedef ClassFactory<FrictionBase,VehicleBase&,const rapidxml::xml_node<char>*> TClassFactory_friction;
	extern TClassFactory_friction classFactory_friction;


	#define DECLARES_REGISTER_FRICTION(CLASS_NAME) \
		DECLARES_REGISTER_CLASS2(CLASS_NAME,FrictionBase,VehicleBase&,const rapidxml::xml_node<char> *)

	#define REGISTER_FRICTION(TEXTUAL_NAME,CLASS_NAME) \
		REGISTER_CLASS2(TClassFactory_friction,classFactory_friction,TEXTUAL_NAME,CLASS_NAME) 
	

}
