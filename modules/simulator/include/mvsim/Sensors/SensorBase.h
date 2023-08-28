/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/poses/CPose3D.h>
#include <mvsim/ClassFactory.h>
#include <mvsim/Simulable.h>
#include <mvsim/VisualObject.h>

#include <memory>

namespace mvsim
{
class Simulable;

class SensorBase : public VisualObject, public Simulable
{
   public:
	using Ptr = std::shared_ptr<SensorBase>;

	/** Ctor takes a ref to the vehicle to which the sensor is attached. */
	SensorBase(Simulable& vehicle);
	virtual ~SensorBase();

	/** Class factory: Creates a sensor from XML description of type "<sensor
	 * class='CLASS_NAME'>...</sensor>".  */
	static SensorBase::Ptr factory(
		Simulable& parent, const rapidxml::xml_node<char>* xml_node);

	/** Loads the parameters common to all sensors. Should be overriden, and
	 * they call to this base method. */
	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root);

	void registerOnServer(mvsim::Client& c) override;

	virtual void simulateOn3DScene(	 //
		[[maybe_unused]] mrpt::opengl::COpenGLScene& gl_scene)
	{
	}

	// Get all sensors visuals API:
	static std::shared_ptr<mrpt::opengl::CSetOfObjects>
		GetAllSensorsOriginViz();

	static std::shared_ptr<mrpt::opengl::CSetOfObjects> GetAllSensorsFOVViz();
	static void RegisterSensorFOVViz(
		const std::shared_ptr<mrpt::opengl::CSetOfObjects>& o);
	static void RegisterSensorOriginViz(
		const std::shared_ptr<mrpt::opengl::CSetOfObjects>& o);

	double sensor_period() const { return sensor_period_; }

	/** The vehicle this sensor is attached to */
	Simulable& vehicle() { return vehicle_; }
	const Simulable& vehicle() const { return vehicle_; }

   protected:
	/** Should be called within each derived class simul_post_timestep() method
	 *  to update sensor_last_timestamp_ and check if the sensor should be
	 * simulated now, given the current simulation time, and the sensor rate in
	 * sensor_period_.
	 *
	 * \return true if it is now time to simulate a new sensor reading,
	 *         false otherwise.
	 */
	bool should_simulate_sensor(const TSimulContext& context);

	Simulable& vehicle_;  //!< The vehicle this sensor is attached to

	World* world() { return vehicle_.getSimulableWorldObject(); }
	const World* world() const { return vehicle_.getSimulableWorldObject(); }

	/** Generate one sensor reading every this period [s] (Default = 0.1) */
	double sensor_period_ = 0.1;

	std::string save_to_rawlog_;
	std::shared_ptr<mrpt::io::CFileGZOutputStream> rawlog_io_;

	/** The last sensor reading timestamp. See  sensor_period_ */
	double sensor_last_timestamp_ = 0;

	/** Publish to MVSIM ZMQ topic stream, if not empty (default) */
	std::string publishTopic_;

	/// Filled in by SensorBase::loadConfigFrom()
	std::map<std::string, std::string> varValues_;

	bool parseSensorPublish(
		const rapidxml::xml_node<char>* node,
		const std::map<std::string, std::string>& varValues);

	void reportNewObservation(
		const std::shared_ptr<mrpt::obs::CObservation>& obs,
		const TSimulContext& context);

	void reportNewObservation_lidar_2d(
		const std::shared_ptr<mrpt::obs::CObservation2DRangeScan>& obs,
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
