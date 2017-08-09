/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mvsim/VisualObject.h>
#include <mvsim/Simulable.h>
#include <mvsim/ClassFactory.h>

namespace mvsim
{
	class VehicleBase;

	class SensorBase : public VisualObject, public Simulable
  {

	public:
    using Ptr = std::shared_ptr<SensorBase>;

		SensorBase(VehicleBase& vehicle); //!< Ctor takes a ref to the vehicle to which the sensor is attached.
		virtual ~SensorBase();

		/** Class factory: Creates a sensor from XML description of type "<sensor class='CLASS_NAME'>...</sensor>".  */
		static SensorBase* factory(VehicleBase & parent, const rapidxml::xml_node<char> *xml_node);

		virtual void loadConfigFrom(const rapidxml::xml_node<char> *root) = 0;

		double m_sensor_period; //!< Generate one sensor reading every this period (in seconds) (Default = 0.1)

	protected:
		VehicleBase & m_vehicle; //!< The vehicle this sensor is attached to

		double m_sensor_last_timestamp; //!< The last sensor reading timestamp. See m_sensor_period

	};

	// Class factory:
	typedef ClassFactory<SensorBase,VehicleBase&,const rapidxml::xml_node<char>*> TClassFactory_sensors;
	extern TClassFactory_sensors classFactory_sensors;


	#define DECLARES_REGISTER_SENSOR(CLASS_NAME) \
		DECLARES_REGISTER_CLASS2(CLASS_NAME,SensorBase,VehicleBase&,const rapidxml::xml_node<char> *)

	#define REGISTER_SENSOR(TEXTUAL_NAME,CLASS_NAME) \
		REGISTER_CLASS2(TClassFactory_sensors,classFactory_sensors,TEXTUAL_NAME,CLASS_NAME) 
	


}
