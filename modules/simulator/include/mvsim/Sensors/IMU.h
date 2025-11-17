/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/random.h>
#include <mvsim/Sensors/SensorBase.h>

#include <mutex>

namespace mvsim
{
/** An Inertial Measurement Unit (IMU) sensor.
 * \ingroup sensors_module
 */
class IMU : public SensorBase
{
	DECLARES_REGISTER_SENSOR(IMU)
   public:
	IMU(Simulable& parent, const rapidxml::xml_node<char>* root);
	virtual ~IMU();

	// See docs in base class
	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;

	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;

	void registerOnServer(mvsim::Client& c) override;

   protected:
	void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		[[maybe_unused]] const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
		[[maybe_unused]] bool childrenOnly) override;

	void notifySimulableSetPose(const mrpt::math::TPose3D& newPose) override;

	mrpt::math::TPose3D getRelativePose() const override { return obs_model_.sensorPose.asTPose(); }
	void setRelativePose(const mrpt::math::TPose3D& p) override
	{
		obs_model_.sensorPose = mrpt::poses::CPose3D(p);
	}

	void internal_simulate_imu(const TSimulContext& context);

	double orientationStdNoise_ = 0.005;  //!< [rad]
	double angularVelocityStdNoise_ = 2e-4;	 //!< [rad/s]
	double linearAccelerationStdNoise_ = 0.017;	 //!< [m/s²]
	bool measure_orientation_ = false;	//!< If true, the IMU will also measure orientation

	// Store here all default parameters. This obj will be copied as a
	// "pattern" to fill it with actual data.
	mrpt::obs::CObservationIMU obs_model_;

	std::mutex last_obs_cs_;

	/** Last simulated obs */
	mrpt::obs::CObservationIMU::Ptr last_obs_;

	mrpt::opengl::CSetOfObjects::Ptr gl_sensor_origin_, gl_sensor_origin_corner_;

	mrpt::random::CRandomGenerator rng_;
};
}  // namespace mvsim
