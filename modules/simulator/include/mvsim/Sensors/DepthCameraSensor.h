/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mvsim/Sensors/SensorBase.h>

#include <mutex>

namespace mvsim
{
/** RGB+D camera sensor on board a vehicle.
 */
class DepthCameraSensor : public SensorBase
{
	DECLARES_REGISTER_SENSOR(DepthCameraSensor)
   public:
	DepthCameraSensor(Simulable& parent, const rapidxml::xml_node<char>* root);
	virtual ~DepthCameraSensor();

	// See docs in base class
	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;

	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;

	void poses_mutex_lock() override {}
	void poses_mutex_unlock() override {}

	void simulateOn3DScene(mrpt::opengl::COpenGLScene& gl_scene) override;

	void freeOpenGLResources() override;

   protected:
	virtual void internalGuiUpdate(
		mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
		bool childrenOnly) override;

	// Store here all sensor intrinsic parameters. This obj will be copied as a
	// "pattern" to fill it with actual scan data.
	mrpt::obs::CObservation3DRangeScan m_sensor_params;

	std::mutex m_last_obs_cs;
	/** Last simulated scan */
	mrpt::obs::CObservation3DRangeScan::Ptr m_last_obs;
	mrpt::obs::CObservation3DRangeScan::Ptr m_last_obs2gui;

	std::shared_ptr<mrpt::opengl::CFBORender> m_fbo_renderer;

	/** Whether m_gl_scan has to be updated upon next call of
	 * internalGuiUpdate() from m_last_scan2gui */
	bool m_gui_uptodate = false;
	std::mutex m_gui_mtx;
	mrpt::opengl::CPointCloudColoured::Ptr m_gl_obs;
	std::optional<TSimulContext> m_has_to_render;
};
}  // namespace mvsim
