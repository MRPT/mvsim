/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/obs_frwds.h>
#include <mvsim/ClassFactory.h>
#include <mvsim/Simulable.h>
#include <mvsim/VisualObject.h>

namespace mvsim
{
class Simulable;

class SensorBase : public VisualObject, public Simulable
{
   public:
	using Ptr = std::shared_ptr<SensorBase>;

	SensorBase(Simulable& vehicle);	 //!< Ctor takes a ref to the vehicle to
									 //! which the sensor is attached.
	virtual ~SensorBase();

	/** Class factory: Creates a sensor from XML description of type "<sensor
	 * class='CLASS_NAME'>...</sensor>".  */
	static SensorBase::Ptr factory(
		Simulable& parent, const rapidxml::xml_node<char>* xml_node);

	/** Loads the parameters common to all sensors. Should be overriden, and
	 * they call to this base method. */
	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root);

	/** Generate one sensor reading every this period [s] (Default = 0.1) */
	double m_sensor_period = 0.1;

	void registerOnServer(mvsim::Client& c) override;

	virtual void simulateOn3DScene(	 //
		[[maybe_unused]] mrpt::opengl::COpenGLScene& gl_scene)
	{
	}

   protected:
	Simulable& m_vehicle;  //!< The vehicle this sensor is attached to

	/** The last sensor reading timestamp. See  m_sensor_period */
	double m_sensor_last_timestamp = 0;

	std::string publishTopic_;

	/// Filled in by SensorBase::loadConfigFrom()
	std::map<std::string, std::string> m_varValues;

	bool parseSensorPublish(
		const rapidxml::xml_node<char>* node,
		const std::map<std::string, std::string>& varValues);

	void reportNewObservation(
		const std::shared_ptr<mrpt::obs::CObservation>& obs,
		const TSimulContext& context);

	/// Assign a sensible default name/sensor label if none is provided:
	void make_sure_we_have_a_name(const std::string& prefix);
};

using TListSensors = std::vector<SensorBase::Ptr>;

// Class factory:
using TClassFactory_sensors =
	ClassFactory<SensorBase, Simulable&, const rapidxml::xml_node<char>*>;

extern TClassFactory_sensors classFactory_sensors;

#define DECLARES_REGISTER_SENSOR(CLASS_NAME) \
	DECLARES_REGISTER_CLASS2(                \
		CLASS_NAME, SensorBase, Simulable&, const rapidxml::xml_node<char>*)

#define REGISTER_SENSOR(TEXTUAL_NAME, CLASS_NAME) \
	REGISTER_CLASS2(                              \
		TClassFactory_sensors, classFactory_sensors, TEXTUAL_NAME, CLASS_NAME)
}  // namespace mvsim
