/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
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
/**
 * @brief A 2D lidar scanner, with FOV up to 360 degrees.
 *
 * There are two working modes:
 * - `raytrace_3d=false`: Very fast simulation, applicable to 2D worlds.
 * - `raytrace_3d=true`: Accurate version using detailed 3D meshes for all
 *   available objects in the scene.
 *
 */
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

	void simulateOn3DScene(mrpt::opengl::COpenGLScene& gl_scene) override;
	void freeOpenGLResources() override;

	void registerOnServer(mvsim::Client& c) override;

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
		bool childrenOnly) override;

	// when not using the 3D raytrace mode.
	void internal_simulate_lidar_2d_mode(const TSimulContext& context);

	int z_order_;  //!< to help rendering multiple scans
	mrpt::poses::CPose2D sensor_pose_on_veh_;
	double rangeStdNoise_ = 0.01;
	double angleStdNoise_ = mrpt::DEG2RAD(0.01);
	/** Whether all box2d "fixtures" are visible (solid) or not (Default=true)
	 */
	bool see_fixtures_ = true;

	/** If enabled, use realistic 3D depth measurement using the scene 3D model,
	 * instead of 2D "fixtures" used for collisions. */
	bool raytrace_3d_ = false;

	bool ignore_parent_body_ = false;

	bool viz_visiblePlane_ = false;
	bool viz_visiblePoints_ = false;
	float viz_pointSize_ = 3.0f;

	// Store here all scan parameters. This obj will be copied as a
	// "pattern" to fill it with actual scan data.
	mrpt::obs::CObservation2DRangeScan scan_model_;

	std::mutex last_scan_cs_;
	/** Last simulated scan */
	mrpt::obs::CObservation2DRangeScan::Ptr last_scan_;
	mrpt::obs::CObservation2DRangeScan::Ptr last_scan2gui_;

	/** Whether gl_scan_ has to be updated upon next call of
	 * internalGuiUpdate() from last_scan2gui_ */
	bool gui_uptodate_ = false;

	mrpt::opengl::CPlanarLaserScan::Ptr gl_scan_;
	mrpt::opengl::CSetOfObjects::Ptr gl_sensor_origin_,
		gl_sensor_origin_corner_;
	mrpt::opengl::CSetOfObjects::Ptr gl_sensor_fov_;

	std::optional<TSimulContext> has_to_render_;
	std::mutex has_to_render_mtx_;

	std::shared_ptr<mrpt::opengl::CFBORender> fbo_renderer_depth_;

	std::vector<size_t> angleIdx2pixelIdx_;
	std::vector<float> angleIdx2secant_;
};
}  // namespace mvsim
