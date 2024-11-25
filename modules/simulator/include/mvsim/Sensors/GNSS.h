/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
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

namespace mvsim
{
/** A Global Navigation Satellite System (GNSS) sensor (GPS).
 *
 * \ingroup sensors_module
 */
class GNSS : public SensorBase
{
	DECLARES_REGISTER_SENSOR(GNSS)
   public:
	GNSS(Simulable& parent, const rapidxml::xml_node<char>* root);
	virtual ~GNSS();

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

	void internal_simulate_gnss(const TSimulContext& context);

	double horizontal_std_noise_ = 2.0;	 //!< [m]
	double vertical_std_noise_ = 4.0;  //!< [m]

	// Store here all default parameters. This obj will be copied as a
	// "pattern" to fill it with actual data.
	mrpt::obs::CObservationGPS obs_model_;

	std::mutex last_obs_cs_;

	/** Last simulated obs */
	mrpt::obs::CObservationGPS::Ptr last_obs_;

	mrpt::opengl::CSetOfObjects::Ptr gl_sensor_origin_, gl_sensor_origin_corner_;

	mrpt::random::CRandomGenerator rng_;
};
}  // namespace mvsim
