/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/poses/CPose2D.h>
#include <mvsim/Sensors/SensorBase.h>

#include <mutex>

namespace mvsim
{
/**
 * @brief A 3D LiDAR sensor, with 360 degrees horizontal fielf-of-view, and a
 * configurable vertical FOV.
 * The number of rays in the vertical FOV and the number of samples in each
 * horizontal row are configurable.
 */
class Lidar3D : public SensorBase
{
	DECLARES_REGISTER_SENSOR(Lidar3D)
   public:
	Lidar3D(Simulable& parent, const rapidxml::xml_node<char>* root);
	virtual ~Lidar3D();

	// See docs in base class
	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;

	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;

	void simulateOn3DScene(mrpt::opengl::COpenGLScene& gl_scene) override;
	void freeOpenGLResources() override;

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
		bool childrenOnly) override;

	mrpt::poses::CPose3D m_sensorPoseOnVeh;

	double m_rangeStdNoise = 0.01;
	bool m_ignore_parent_body = false;

	float m_viz_pointSize = 3.0f;
	float m_maxRange = 80.0f;
	double m_vertical_fov = mrpt::DEG2RAD(30.0);
	int m_vertNumRays = 16, m_horzNumRays = 180;

	/** Last simulated scan */
	mrpt::obs::CObservationPointCloud::Ptr m_last_scan2gui, m_last_scan;
	std::mutex m_last_scan_cs;

	/** Whether m_gl_scan has to be updated upon next call of
	 * internalGuiUpdate() from m_last_scan2gui */
	bool m_gui_uptodate = false;

	mrpt::opengl::CPointCloudColoured::Ptr m_glPoints;
	mrpt::opengl::CSetOfObjects::Ptr m_gl_sensor_origin,
		m_gl_sensor_origin_corner;
	mrpt::opengl::CSetOfObjects::Ptr m_gl_sensor_fov;

	std::optional<TSimulContext> m_has_to_render;
	std::mutex m_has_to_render_mtx;

	std::shared_ptr<mrpt::opengl::CFBORender> m_fbo_renderer_depth;
};
}  // namespace mvsim
