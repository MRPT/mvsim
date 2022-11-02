/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/opengl/CPlanarLaserScan.h>
#include <mrpt/poses/CPose2D.h>
#include <mvsim/Sensors/SensorBase.h>

#include <mutex>

namespace mvsim
{
class LaserScanner : public SensorBase
{
	DECLARES_REGISTER_SENSOR(LaserScanner)
   public:
	LaserScanner(Simulable& parent, const rapidxml::xml_node<char>* root);
	virtual ~LaserScanner();

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

	// when not using the 3D raytrace mode.
	void internal_simulate_lidar_2d_mode(const TSimulContext& context);

	int m_z_order;	//!< to help rendering multiple scans
	mrpt::poses::CPose2D m_sensor_pose_on_veh;
	double m_rangeStdNoise = 0.01;
	double m_angleStdNoise = mrpt::DEG2RAD(0.01);
	/** Whether all box2d "fixtures" are visible (solid) or not (Default=true)
	 */
	bool m_see_fixtures = true;

	/** If enabled, use realistic 3D depth measurement using the scene 3D model,
	 * instead of 2D "fixtures" used for collisions. */
	bool m_raytrace_3d = false;

	bool m_ignore_parent_body = false;

	bool m_viz_visiblePlane = false;
	bool m_viz_visiblePoints = false;
	float m_viz_pointSize = 3.0f;

	// Store here all scan parameters. This obj will be copied as a
	// "pattern" to fill it with actual scan data.
	mrpt::obs::CObservation2DRangeScan m_scan_model;

	std::mutex m_last_scan_cs;
	/** Last simulated scan */
	mrpt::obs::CObservation2DRangeScan::Ptr m_last_scan;
	mrpt::obs::CObservation2DRangeScan::Ptr m_last_scan2gui;

	/** Whether m_gl_scan has to be updated upon next call of
	 * internalGuiUpdate() from m_last_scan2gui */
	bool m_gui_uptodate = false;

	std::recursive_mutex m_gui_mtx;
	mrpt::opengl::CPlanarLaserScan::Ptr m_gl_scan;
	mrpt::opengl::CSetOfObjects::Ptr m_gl_sensor_origin,
		m_gl_sensor_origin_corner;
	mrpt::opengl::CSetOfObjects::Ptr m_gl_sensor_fov;

	std::optional<TSimulContext> m_has_to_render;
	std::mutex m_has_to_render_mtx;

	std::shared_ptr<mrpt::opengl::CFBORender> m_fbo_renderer_depth;
};
}  // namespace mvsim
