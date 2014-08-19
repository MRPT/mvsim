/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/VisualObject.h>
#include <mv2dsim/Simulable.h>
#include <mv2dsim/ClassFactory.h>
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

namespace mv2dsim
{
	class VehicleBase;

	class SensorBase : public VisualObject, public Simulable
	{
	public:
		SensorBase(VehicleBase& vehicle); //!< Ctor takes a ref to the vehicle to which the sensor is attached.
		virtual ~SensorBase();

		/** Class factory: Creates a sensor from XML description of type "<sensor:*>...</sensor:*>".  */
		static SensorBase* factory(VehicleBase & parent, const rapidxml::xml_node<char> *xml_node);

		virtual void loadConfigFrom(const rapidxml::xml_node<char> *root) = 0;

	protected:
		VehicleBase & m_vehicle; //!< The vehicle this sensor is attached to

	};

	typedef stlplus::smart_ptr<SensorBase> SensorBasePtr;

	// Class factory:
	typedef ClassFactory<SensorBase,VehicleBase&,const rapidxml::xml_node<char>*> TClassFactory_sensors;
	extern TClassFactory_sensors classFactory_sensors;


	#define DECLARES_REGISTER_SENSOR(CLASS_NAME) \
		DECLARES_REGISTER_CLASS2(CLASS_NAME,SensorBase,VehicleBase&,const rapidxml::xml_node<char> *)

	#define REGISTER_SENSOR(TEXTUAL_NAME,CLASS_NAME) \
		REGISTER_CLASS2(TClassFactory_sensors,classFactory_sensors,TEXTUAL_NAME,CLASS_NAME) 
	


}
