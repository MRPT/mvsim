/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
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
/** A 3D LiDAR sensor, with 360 degrees horizontal fielf-of-view, and a
 * configurable vertical FOV.
 * The number of rays in the vertical FOV and the number of samples in each
 * horizontal row are configurable.
 *
 * \ingroup sensors_module
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
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly) override;

	void notifySimulableSetPose(const mrpt::math::TPose3D& newPose) override;

	mrpt::math::TPose3D getRelativePose() const override { return sensorPoseOnVeh_.asTPose(); }
	void setRelativePose(const mrpt::math::TPose3D& p) override
	{
		sensorPoseOnVeh_ = mrpt::poses::CPose3D(p);
	}

	mrpt::poses::CPose3D sensorPoseOnVeh_;

	double rangeStdNoise_ = 0.01;
	bool ignore_parent_body_ = false;

	float viz_pointSize_ = 3.0f;
	float minRange_ = 0.01f;
	float maxRange_ = 80.0f;

	/** vertical FOV will be symmetric above and below the horizontal line,
	 *  unless vertical_ray_angles_str_ is set */
	double vertical_fov_ = 30.0;  //!< In *degrees* !!

	/** If not empty, will define the list of ray vertical angles, in degrees,
	 * separated by whitespaces. Positive angles are above, negative below
	 * the horizontal plane.
	 */
	std::string vertical_ray_angles_str_;

	int vertNumRays_ = 16, horzNumRays_ = 180;
	double horzResolutionFactor_ = 1.0;
	double vertResolutionFactor_ = 1.0;
	float maxDepthInterpolationStepVert_ = 0.30f;
	float maxDepthInterpolationStepHorz_ = 0.10f;

	/** Last simulated scan */
	mrpt::obs::CObservationPointCloud::Ptr last_scan2gui_, last_scan_;
	std::mutex last_scan_cs_;

	/** Whether gl_scan_ has to be updated upon next call of
	 * internalGuiUpdate() from last_scan2gui_ */
	bool gui_uptodate_ = false;

	mrpt::opengl::CPointCloudColoured::Ptr glPoints_;
	mrpt::opengl::CSetOfObjects::Ptr gl_sensor_origin_, gl_sensor_origin_corner_;
	mrpt::opengl::CSetOfObjects::Ptr gl_sensor_fov_;

	std::optional<TSimulContext> has_to_render_;
	std::mutex has_to_render_mtx_;

	std::shared_ptr<mrpt::opengl::CFBORender> fbo_renderer_depth_;

	struct PerRayLUT
	{
		float u = 0, v = 0;	 //!< Pixel coords
		float depth2range = 0;
	};
	struct PerHorzAngleLUT
	{
		std::vector<PerRayLUT> column;
	};

	std::vector<PerHorzAngleLUT> lut_;

	/// Upon initialization, vertical_rays_str_ is parsed in this vector of
	/// angles in radians.
	std::vector<double> vertical_ray_angles_;
};
}  // namespace mvsim
