/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/obs_frwds.h>
#include <mrpt/opengl/opengl_frwds.h>
#include <mrpt/poses/CPose3D.h>
#include <mvsim/ClassFactory.h>
#include <mvsim/Simulable.h>
#include <mvsim/VisualObject.h>

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <optional>

namespace mvsim
{
class Simulable;

/** @addtogroup sensors_module  Sensor simulation module
 * \ingroup mvsim_simulator_module
 */

/**
 * @brief Statistics tracking for sensor observation rates.
 *
 * This structure maintains a ring buffer of observation timestamps
 * to compute actual sensor rates and timing information.
 *
 * \ingroup sensors_module
 */
struct SensorStats
{
	/// Number of samples to keep for rate calculation
	static constexpr size_t STATS_HISTORY_SIZE = 100;

	/// Ring buffer of observation timestamps
	std::array<double, STATS_HISTORY_SIZE> observationTimestamps_{};

	/// Current index in ring buffer
	size_t ringIndex_ = 0;

	/// Number of samples currently stored (up to STATS_HISTORY_SIZE)
	size_t sampleCount_ = 0;

	/// Total observations since simulation start
	size_t totalObservations_ = 0;

	/// Last observation timestamp
	double lastObservationTime_ = 0.0;

	/// Mutex for thread-safe access
	mutable std::mutex mtx_;

	/// Record a new observation timestamp
	void recordObservation(double timestamp)
	{
		std::lock_guard<std::mutex> lck(mtx_);

		observationTimestamps_[ringIndex_] = timestamp;
		ringIndex_ = (ringIndex_ + 1) % STATS_HISTORY_SIZE;

		if (sampleCount_ < STATS_HISTORY_SIZE)
		{
			++sampleCount_;
		}

		++totalObservations_;
		lastObservationTime_ = timestamp;
	}

	/// Calculate actual observation rate in Hz over the stored window
	double getActualRateHz() const
	{
		std::lock_guard<std::mutex> lck(mtx_);

		if (sampleCount_ < 2)
		{
			return 0.0;
		}

		// Find oldest and newest timestamps in the ring buffer
		double oldest = observationTimestamps_[0];
		double newest = observationTimestamps_[0];

		for (size_t i = 0; i < sampleCount_; ++i)
		{
			const double t = observationTimestamps_[i];
			if (t < oldest)
			{
				oldest = t;
			}
			if (t > newest)
			{
				newest = t;
			}
		}

		const double timeSpan = newest - oldest;
		if (timeSpan < 1e-9)
		{
			return 0.0;
		}

		// Rate = (number of intervals) / time span
		const double numIntervals = static_cast<double>(sampleCount_ - 1);
		return numIntervals / timeSpan;
	}

	/// Get time since last observation (returns -1 if no observations yet)
	double getTimeSinceLastObservation(double currentTime) const
	{
		std::lock_guard<std::mutex> lck(mtx_);

		if (totalObservations_ == 0)
		{
			return -1.0;
		}

		return currentTime - lastObservationTime_;
	}

	/// Get total observation count
	size_t getTotalObservations() const
	{
		std::lock_guard<std::mutex> lck(mtx_);
		return totalObservations_;
	}

	/// Get last observation time
	double getLastObservationTime() const
	{
		std::lock_guard<std::mutex> lck(mtx_);
		return lastObservationTime_;
	}

	/// Reset all statistics
	void reset()
	{
		std::lock_guard<std::mutex> lck(mtx_);

		ringIndex_ = 0;
		sampleCount_ = 0;
		totalObservations_ = 0;
		lastObservationTime_ = 0.0;
	}
};

/**
 * @brief Custom statistics container for sensor-specific values.
 *
 * Allows sensors to report additional statistics like points per scan,
 * image resolution, etc.
 *
 * \ingroup sensors_module
 */
struct SensorCustomStats
{
	mutable std::mutex mtx_;
	std::map<std::string, double> values_;

	void set(const std::string& key, double value)
	{
		std::lock_guard<std::mutex> lck(mtx_);
		values_[key] = value;
	}

	std::optional<double> get(const std::string& key) const
	{
		std::lock_guard<std::mutex> lck(mtx_);
		auto it = values_.find(key);
		if (it != values_.end())
		{
			return it->second;
		}
		return std::nullopt;
	}

	std::map<std::string, double> getAll() const
	{
		std::lock_guard<std::mutex> lck(mtx_);
		return values_;
	}

	void clear()
	{
		std::lock_guard<std::mutex> lck(mtx_);
		values_.clear();
	}
};

/**
 * @brief Virtual base class for all sensors.
 *
 * \ingroup sensors_module
 */
class SensorBase : public VisualObject, public Simulable
{
   public:
	using Ptr = std::shared_ptr<SensorBase>;

	/** Ctor takes a ref to the vehicle to which the sensor is attached. */
	SensorBase(Simulable& vehicle);

	/** Class factory: Creates a sensor from XML description of type "<sensor
	 * class='CLASS_NAME'>...</sensor>".  */
	static SensorBase::Ptr factory(Simulable& parent, const rapidxml::xml_node<char>* xml_node);

	/** Loads the parameters common to all sensors. Should be overriden, and
	 * they call to this base method. */
	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root);

	void registerOnServer(mvsim::Client& c) override;

	virtual void simulateOn3DScene([[maybe_unused]] mrpt::opengl::COpenGLScene& gl_scene) {}

	// Get all sensors visuals API:
	static std::shared_ptr<mrpt::opengl::CSetOfObjects> GetAllSensorsOriginViz();

	static std::shared_ptr<mrpt::opengl::CSetOfObjects> GetAllSensorsFOVViz();

	static void RegisterSensorFOVViz(const std::shared_ptr<mrpt::opengl::CSetOfObjects>& o);

	static void RegisterSensorOriginViz(const std::shared_ptr<mrpt::opengl::CSetOfObjects>& o);

	double sensor_period() const { return sensor_period_; }

	/** The vehicle this sensor is attached to */
	Simulable& vehicle() { return vehicle_; }

	const Simulable& vehicle() const { return vehicle_; }

	/** Get the sensor statistics object (const) */
	const SensorStats& getStats() const { return stats_; }

	/** Get custom statistics (e.g., points per scan, resolution) */
	const SensorCustomStats& getCustomStats() const { return customStats_; }

	/** Get target rate in Hz (1.0 / sensor_period) */
	double getTargetRateHz() const
	{
		if (sensor_period_ > 1e-9)
		{
			return 1.0 / sensor_period_;
		}
		return 0.0;
	}

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

	/** The last sensor reading timestamp. See  sensor_period_ */
	double sensor_last_timestamp_ = 0;

	/** Publish to MVSIM ZMQ topic stream, if not empty (default) */
	std::string publishTopic_;

	/// Filled in by SensorBase::loadConfigFrom()
	std::map<std::string, std::string> varValues_;

	bool parseSensorPublish(
		const rapidxml::xml_node<char>* node, const std::map<std::string, std::string>& varValues);

	void reportNewObservation(
		const std::shared_ptr<mrpt::obs::CObservation>& obs, const TSimulContext& context);

	void reportNewObservation_lidar_2d(
		const std::shared_ptr<mrpt::obs::CObservation2DRangeScan>& obs,
		const TSimulContext& context);

	/// Assign a sensible default name/sensor label if none is provided:
	void make_sure_we_have_a_name(const std::string& prefix);

	/** Statistics for this sensor (observation rate tracking) */
	SensorStats stats_;

	/** Custom statistics (sensor-specific values like points/scan) */
	SensorCustomStats customStats_;

	/** Helper to set a custom stat value */
	void setCustomStat(const std::string& key, double value) { customStats_.set(key, value); }
};

using TListSensors = std::vector<SensorBase::Ptr>;

// Class factory:
using TClassFactory_sensors = ClassFactory<SensorBase, Simulable&, const rapidxml::xml_node<char>*>;

extern TClassFactory_sensors classFactory_sensors;

#define DECLARES_REGISTER_SENSOR(CLASS_NAME) \
	DECLARES_REGISTER_CLASS2(CLASS_NAME, SensorBase, Simulable&, const rapidxml::xml_node<char>*)

#define REGISTER_SENSOR(TEXTUAL_NAME, CLASS_NAME) \
	REGISTER_CLASS2(TClassFactory_sensors, classFactory_sensors, TEXTUAL_NAME, CLASS_NAME)

}  // namespace mvsim
