/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/viz/CPointCloudColoured.h>
#include <mvsim/Sensors/SensorBase.h>

#include <mutex>

namespace mvsim
{
/** A 3D LiDAR sensor, with 360 degrees horizontal fielf-of-view, and a
 * configurable vertical FOV.
 * The number of rays in the vertical FOV and the number of samples in each
 * horizontal row are configurable.
 *
 * Optionally, each point carries the color of the scene where the ray hits
 * (`generate_rgb`), as in RGB LiDARs (e.g. Ouster Rev8), in the uint8 point
 * fields `color_r`, `color_g`, `color_b`.
 *
 * \ingroup sensors_module
 */
class Lidar3D : public SensorBase
{
	DECLARES_REGISTER_SENSOR(Lidar3D)
   public:
	Lidar3D(Simulable& parent, const rapidxml::xml_node<char>* root);

	// See docs in base class
	virtual void loadConfigFrom(const rapidxml::xml_node<char>* root) override;

	virtual void simul_pre_timestep(const TSimulContext& context) override;
	virtual void simul_post_timestep(const TSimulContext& context) override;

	void simulateOn3DScene(mrpt::viz::Scene& gl_scene) override;
	void freeOpenGLResources() override;
	bool rendersWithOpenGL() const override { return true; }

	/** Read-only access to the raw XML-configured parameters, for exact
	 * (non-visual, non-rendering) consumers such as the offline ray
	 * tracer's Lidar3DModel. Mirrors exactly what simulateOn3DScene() uses
	 * to build its own (lazy, GUI-only) `vertical_ray_angles_`, so a
	 * caller can reproduce the same ray set without ever rendering. */
	int vertNumRays() const { return vertNumRays_; }
	int horzNumRays() const { return horzNumRays_; }
	double verticalFovDegrees() const { return vertical_fov_; }
	const std::string& verticalRayAnglesStr() const { return vertical_ray_angles_str_; }
	float minRange() const { return minRange_; }
	float maxRange() const { return maxRange_; }
	double rangeStdNoise() const { return rangeStdNoise_; }
	bool generateIntensityFromRGB() const { return generateIntensityFromRGB_; }
	bool generateRGB() const { return generateRGB_; }

	/** Realistic sensor error model. Applied by the offline ray tracer
	 * (mvsim-dataset-gen) only; all zero: disabled (the default). */
	struct ErrorModel
	{
		/** Range noise std growing linearly with range [m per m]. */
		double rangeStdNoisePerMeter = 0;
		/** Full beam divergence [deg]: the footprint on an oblique surface
		 * spreads the return over a range interval. */
		double beamDivergenceDeg = 0;
		/** No return beyond this incidence angle [deg] (0: disabled). */
		double maxIncidenceDeg = 0;
		/** Probability of a mixed return (a range between both surfaces)
		 * at a depth edge between neighbor rays. */
		double mixedPixelProbability = 0;
		/** Per-beam calibration errors (std of the elevation and azimuth
		 * offsets [deg], and of the range bias [m]), drawn once per run from
		 * calibrationSeed. Rays sample the scene along the perturbed
		 * direction, but points are reported along the nominal one. */
		double beamElevationStdDeg = 0;
		double beamAzimuthStdDeg = 0;
		double beamRangeBiasStd = 0;
		int calibrationSeed = 0;
	};
	const ErrorModel& errorModel() const { return errorModel_; }

	/** The sensor's pose on the vehicle. Equivalent to getRelativePose(),
	 * exposed under a public, non-overridden name: Lidar3D re-declares
	 * getRelativePose() as protected (overriding Simulable's public one),
	 * which makes it inaccessible through a `Lidar3D*`/`Lidar3D&` (C++
	 * access control follows the static type, not virtual dispatch). */
	mrpt::poses::CPose3D sensorPoseOnVehicle() const { return sensorPoseOnVeh_; }

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::viz::Scene>& viz,
		const mrpt::optional_ref<mrpt::viz::Scene>& physical, bool childrenOnly) override;

	void notifySimulableSetPose(const mrpt::math::TPose3D& newPose) override;

	mrpt::math::TPose3D getRelativePose() const override { return sensorPoseOnVeh_.asTPose(); }
	void setRelativePose(const mrpt::math::TPose3D& p) override
	{
		sensorPoseOnVeh_ = mrpt::poses::CPose3D(p);
	}

	mrpt::poses::CPose3D sensorPoseOnVeh_;

	double rangeStdNoise_ = 0.01;
	ErrorModel errorModel_;

	/// Precomputed unit-variance noise of this sensor (reproducible with <random_seed>)
	std::vector<float> noiseSeq_;
	size_t noiseIdx_ = 0;

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

	mrpt::viz::CPointCloudColoured::Ptr glPoints_;
	mrpt::viz::CSetOfObjects::Ptr gl_sensor_origin_, gl_sensor_origin_corner_;
	mrpt::viz::CSetOfObjects::Ptr gl_sensor_fov_;

	std::optional<TSimulContext> has_to_render_;
	std::mutex has_to_render_mtx_;

	std::shared_ptr<mrpt::opengl::CFBORender> fbo_renderer_depth_;

	/** If true, intensity values will be generated from the grayscale of
	 *  the rendered RGB image for each lidar return. */
	bool generateIntensityFromRGB_ = true;

	/** If true, each point gets the color (`color_{r,g,b}` uint8 fields)
	 *  of the rendered RGB image pixel of its lidar return. */
	bool generateRGB_ = false;

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
