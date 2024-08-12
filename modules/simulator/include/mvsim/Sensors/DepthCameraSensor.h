/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
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

	void simulateOn3DScene(mrpt::opengl::COpenGLScene& gl_scene) override;

	void freeOpenGLResources() override;

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	void notifySimulableSetPose(const mrpt::math::TPose3D& newPose) override;

	mrpt::math::TPose3D getRelativePose() const override
	{
		return sensor_params_.sensorPose.asTPose();
	}
	void setRelativePose(const mrpt::math::TPose3D& p) override
	{
		sensor_params_.setSensorPose(mrpt::poses::CPose3D(p));
	}

	// Store here all sensor intrinsic parameters. This obj will be copied as a
	// "pattern" to fill it with actual scan data.
	mrpt::obs::CObservation3DRangeScan sensor_params_;

	std::mutex last_obs_cs_;
	/** Last simulated scan */
	mrpt::obs::CObservation3DRangeScan::Ptr last_obs_;
	mrpt::obs::CObservation3DRangeScan::Ptr last_obs2gui_;

	// Note: we need 2 to support different resolutions for RGB vs Depth.
	std::shared_ptr<mrpt::opengl::CFBORender> fbo_renderer_rgb_, fbo_renderer_depth_;

	/** Whether gl_scan_ has to be updated upon next call of
	 * internalGuiUpdate() from last_scan2gui_ */
	bool gui_uptodate_ = false;
	mrpt::opengl::CPointCloudColoured::Ptr gl_obs_;

	std::optional<TSimulContext> has_to_render_;
	std::mutex has_to_render_mtx_;

	float rgbClipMin_ = 1e-2, rgbClipMax_ = 1e+4;
	float depth_clip_min_ = 0.1, depth_clip_max_ = 15.0;
	float depth_resolution_ = 1e-3;

	bool sense_depth_ = true;  //!< Simulate the DEPTH sensor part
	bool sense_rgb_ = true;	 //!< Simulate the RGB sensor part

	float depth_noise_sigma_ = 1e-3;
	bool show_3d_pointcloud_ = false;

	mrpt::opengl::CSetOfObjects::Ptr gl_sensor_origin_, gl_sensor_origin_corner_;
	mrpt::opengl::CSetOfObjects::Ptr gl_sensor_fov_, gl_sensor_frustum_;

	mrpt::math::CMatrixFloat depthImage_;  // to avoid memory allocs
};
}  // namespace mvsim
