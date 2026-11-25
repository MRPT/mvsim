/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random.h>
#include <mvsim/Sensors/SensorBase.h>

#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mvsim
{
/** A Global Navigation Satellite System (GNSS) sensor (GPS).
 *
 * XML parameters: `horizontal_std_noise`, `vertical_std_noise` [m], and
 * `fix_type` (`no_fix`, `single`, `dgps` (default), `rtk_float`,
 * `rtk_fixed`).
 *
 * Optional degradation events, in simulation time, for robustness tests:
 * \code
 * <event start="10" end="20" fix_type="rtk_float" horizontal_std_noise="0.5" />
 * <event start="30" end="35" outage="true" />       <!-- no data at all -->
 * <event start="40" end="45" fix_type="no_fix" />   <!-- data without fix -->
 * <event start="50" end="60" offset="2.0 -1.0 0" /> <!-- ENU position jump [m] -->
 * \endcode
 *
 * An event with an offset and no other change keeps the fix type and the
 * reported covariance: with `rtk_fixed`, a "false fix" meters off.
 *
 * \sa See the property `gps_no_coverage` in `property_region` world elements.
 *
 * \ingroup sensors_module
 */
class GNSS : public SensorBase
{
	DECLARES_REGISTER_SENSOR(GNSS)
   public:
	GNSS(Simulable& parent, const rapidxml::xml_node<char>* root);
	virtual ~GNSS();

	// Deleted copy and move operations
	GNSS(const GNSS&) = delete;
	GNSS& operator=(const GNSS&) = delete;
	GNSS(GNSS&&) = delete;
	GNSS& operator=(GNSS&&) = delete;

	// See docs in base class
	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;

	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;

	void registerOnServer(mvsim::Client& c) override;

	/** A time interval with modified GNSS quality */
	struct Event
	{
		double start = 0;  //!< [s] simulation time
		double end = 0;	 //!< [s] simulation time
		bool outage = false;
		std::optional<mrpt::obs::GnssFixType> fix_type;
		std::optional<double> horizontal_std_noise;
		std::optional<double> vertical_std_noise;
		mrpt::math::TPoint3D offset = {0, 0, 0};  //!< ENU [m]
	};

	/** \name Read-only access to the XML-configured parameters, for exact
	 * (non-simulation) consumers such as the offline dataset generator.
	 * @{ */
	double horizontalStdNoise() const { return horizontal_std_noise_; }
	double verticalStdNoise() const { return vertical_std_noise_; }
	mrpt::obs::GnssFixType fixType() const { return fix_type_; }
	const std::vector<Event>& events() const { return events_; }
	/** The antenna pose on the vehicle (see IMU::sensorPoseOnVehicle()) */
	mrpt::poses::CPose3D sensorPoseOnVehicle() const { return obs_model_.sensorPose; }
	/** @} */

	/** The XML name of a fix type ("rtk_fixed", etc.) */
	static std::string fixTypeName(mrpt::obs::GnssFixType t);

	/** The quality of the readings at a given time */
	struct Quality
	{
		bool outage = false;  //!< No readings at all
		mrpt::obs::GnssFixType fix_type = mrpt::obs::GnssFixType::DGPS;
		double horizontal_std_noise = 0;  //!< [m]
		double vertical_std_noise = 0;	//!< [m]
		mrpt::math::TPoint3D offset = {0, 0, 0};  //!< ENU [m]

		bool operator==(const Quality& o) const
		{
			return outage == o.outage && fix_type == o.fix_type &&
				   horizontal_std_noise == o.horizontal_std_noise &&
				   vertical_std_noise == o.vertical_std_noise && offset == o.offset;
		}
		bool operator!=(const Quality& o) const { return !(*this == o); }
	};

	/** The quality at simulation time `t` [s]: the sensor defaults, changed
	 * by the events covering `t` in their order (the offsets add up). */
	Quality qualityAt(double t) const;

	/** Simulates one reading at simulation time `t` [s] (the clock of the
	 * degradation events), stamped `stamp`, with the vehicle at `vehiclePose`
	 * in the world. `noiseScale` scales the position noise and the event
	 * offsets, but not the reported covariance. Returns nullptr if there is
	 * no reading: during an outage, in a `gps_no_coverage` region, or if the
	 * world has no `<georeference>`. */
	mrpt::obs::CObservationGPS::Ptr simulateReading(
		const mrpt::poses::CPose3D& vehiclePose, double t, const mrpt::Clock::time_point& stamp,
		mrpt::random::CRandomGenerator& rng, double noiseScale = 1.0) const;

   protected:
	void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::viz::Scene>& viz,
		[[maybe_unused]] const mrpt::optional_ref<mrpt::viz::Scene>& physical,
		[[maybe_unused]] bool childrenOnly) override;

	void notifySimulableSetPose(const mrpt::math::TPose3D& newPose) override;

	mrpt::math::TPose3D getRelativePose() const override { return obs_model_.sensorPose.asTPose(); }
	void setRelativePose(const mrpt::math::TPose3D& p) override
	{
		obs_model_.sensorPose = mrpt::poses::CPose3D(p);
	}

	void internal_simulate_gnss(const TSimulContext& context);

	double horizontal_std_noise_ = 2.0;	 //!< [m]
	double vertical_std_noise_ = 4.0;  //!< [m]
	mrpt::obs::GnssFixType fix_type_ = mrpt::obs::GnssFixType::DGPS;

	std::vector<Event> events_;

	// Store here all default parameters. This obj will be copied as a
	// "pattern" to fill it with actual data.
	mrpt::obs::CObservationGPS obs_model_;

	std::mutex last_obs_cs_;

	/** Last simulated obs */
	mrpt::obs::CObservationGPS::Ptr last_obs_;

	mrpt::viz::CSetOfObjects::Ptr gl_sensor_origin_, gl_sensor_origin_corner_;

	mrpt::random::CRandomGenerator rng_;
	bool noiseSeeded_ = false;
};
}  // namespace mvsim
