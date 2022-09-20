/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationImage.h>
#include <mrpt/opengl/CFBORender.h>
#include <mvsim/Sensors/SensorBase.h>

#include <mutex>

namespace mvsim
{
/** A "RGB" camera sensor on board a vehicle.
 */
class CameraSensor : public SensorBase
{
	DECLARES_REGISTER_SENSOR(CameraSensor)

   public:
	CameraSensor(Simulable& parent, const rapidxml::xml_node<char>* root);
	virtual ~CameraSensor();

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
	mrpt::obs::CObservationImage m_sensor_params;

	std::mutex m_last_obs_cs;
	/** Last simulated scan */
	mrpt::obs::CObservationImage::Ptr m_last_obs;
	mrpt::obs::CObservationImage::Ptr m_last_obs2gui;

	std::shared_ptr<mrpt::opengl::CFBORender> m_fbo_renderer_rgb;

	/** Whether m_gl_scan has to be updated upon next call of
	 * internalGuiUpdate() from m_last_scan2gui */
	bool m_gui_uptodate = false;
	std::recursive_mutex m_gui_mtx;
	std::optional<TSimulContext> m_has_to_render;

	float m_rgb_clip_min = 1e-2, m_rgb_clip_max = 1e+4;
	float m_ambient_light = 0.6f;

	mrpt::opengl::CSetOfObjects::Ptr m_gl_sensor_origin,
		m_gl_sensor_origin_corner;
	mrpt::opengl::CSetOfObjects::Ptr m_gl_sensor_fov, m_gl_sensor_frustum;
};
}  // namespace mvsim
