/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mvsim/Sensors/SensorBase.h>

#include <mutex>

namespace mvsim
{
/** "RGB+D" or just "D" (depth without RGB) camera sensor on board a vehicle.
 *
 *  Use the XML parameters:
 * \code
 * <sense_depth>true</sense_depth>
 * <sense_rgb>true</sense_rgb>
 * \endcode
 *
 * to optionally disable the simulation of either the RGB or Depth part of
 * the outcoming mrpt::obs::CObservation3DRangeScan observations.
 *
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

	// Note: we need 2 to support different resolutions for RGB vs Depth.
	std::shared_ptr<mrpt::opengl::CFBORender> m_fbo_renderer_rgb,
		m_fbo_renderer_depth;

	/** Whether m_gl_scan has to be updated upon next call of
	 * internalGuiUpdate() from m_last_scan2gui */
	bool m_gui_uptodate = false;
	std::recursive_mutex m_gui_mtx;
	mrpt::opengl::CPointCloudColoured::Ptr m_gl_obs;
	std::optional<TSimulContext> m_has_to_render;

	float m_rgb_clip_min = 1e-2, m_rgb_clip_max = 1e+4;
	float m_depth_clip_min = 0.1, m_depth_clip_max = 15.0;
	float m_depth_resolution = 1e-3;

	float m_ambient_light = 0.6f;

	bool m_sense_depth = true;	//!< Simulate the DEPTH sensor part
	bool m_sense_rgb = true;  //!< Simulate the RGB sensor part

	float m_depth_noise_sigma = 1e-3;
	bool m_show_3d_pointcloud = false;

	mrpt::opengl::CSetOfObjects::Ptr m_gl_sensor_origin,
		m_gl_sensor_origin_corner;
	mrpt::opengl::CSetOfObjects::Ptr m_gl_sensor_fov, m_gl_sensor_frustum;

	mrpt::math::CMatrixFloat m_depthImage;	// to avoid memory allocs
};
}  // namespace mvsim
