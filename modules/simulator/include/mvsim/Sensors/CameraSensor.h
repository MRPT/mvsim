/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CObservationImage.h>
#include <mrpt/opengl/CFBORender.h>
#include <mvsim/Sensors/SensorBase.h>
#include <mvsim/TParameterDefinitions.h>

#include <mutex>
#include <string>

/** Lens distortion and pixel noise are applied by the MRPT GPU renderer */
#define MIN_MRPT_VERSION_CAMERA_DISTORTION 0x030600

namespace mvsim
{
/** Lens distortion and pixel noise options of an RGB camera image, read from
 * the XML tags `distortion_model`, `k1`, `k2`, `p1`, `p2`, `k3`, `k4` and
 * `image_noise_std`, optionally with a prefix (e.g. `rgb_k1`). Also the
 * rolling shutter (`rolling_shutter_line_time`, `rolling_shutter_bands`) and
 * the image circle of fisheye lenses (`image_circle_fov_deg`), only
 * simulated by mvsim-dataset-gen.
 * \ingroup sensors_module
 */
struct CameraDistortionOptions
{
	/** "none", "plumb_bob" (k1 k2 p1 p2 k3) or "kannala_brandt" (fisheye,
	 * k1 k2 k3 k4; also accepted as "equidistant") */
	std::string distortionModel = "none";
	double k1 = 0;
	double k2 = 0;
	double p1 = 0;
	double p2 = 0;
	double k3 = 0;
	double k4 = 0;	//!< Only for kannala_brandt
	double imageNoiseStd = 0;  //!< Gaussian pixel noise [intensity levels, 0-255]

	/** Fisheye (kannala_brandt) lenses: full field of view of the image
	 * circle [deg]. Pixels whose rays are more than half of it away from the
	 * optical axis are black. 0: no image circle. Used by mvsim-dataset-gen
	 * only. */
	double imageCircleFovDeg = 0;

	/** Rolling shutter: time between the exposures of consecutive image rows
	 * [s] (0: global shutter). Used by mvsim-dataset-gen only. */
	double rollingShutterLineTime = 0;
	/** Rolling shutter: number of image bands rendered, each from the camera
	 * pose at its middle row's time. */
	int rollingShutterBands = 8;

	/** Adds the XML parameters, with tag names starting with `prefix` */
	void declareParams(TParameterDefinitions& params, const std::string& prefix = "");

	/** Validates the parsed options and sets the distortion model of `cam` */
	void applyTo(mrpt::img::TCamera& cam) const;

	/** Enables distortion and noise in a renderer of `cam` images */
	void applyTo(
		mrpt::opengl::CFBORender& renderer, const mrpt::img::TCamera& cam,
		uint32_t noiseSeed = 0) const;
};

/** An "RGB" camera sensor on board a vehicle.
 * \ingroup sensors_module
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

	void simulateOn3DScene(mrpt::viz::Scene& gl_scene) override;

	void freeOpenGLResources() override;
	bool rendersWithOpenGL() const override { return true; }

	/** \name Read-only access to the XML-configured parameters, for exact
	 * (non-simulation) consumers such as the offline dataset generator.
	 * @{ */
	/** Camera intrinsics (`cameraParams`, with the lens distortion) and pose
	 * on the vehicle (`cameraPose`, of the optical frame: +Z forward). */
	const mrpt::obs::CObservationImage& sensorParams() const { return sensor_params_; }
	float clipMin() const { return rgbClipMin_; }
	float clipMax() const { return rgbClipMax_; }
	const CameraDistortionOptions& imageOptions() const { return distortion_; }
	/** @} */

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::viz::Scene>& viz,
		const mrpt::optional_ref<mrpt::viz::Scene>& physical, bool childrenOnly) override;

	void notifySimulableSetPose(const mrpt::math::TPose3D& newPose) override;

	mrpt::math::TPose3D getRelativePose() const override { return sensor_params_.sensorPose(); }
	void setRelativePose(const mrpt::math::TPose3D& p) override
	{
		sensor_params_.setSensorPose(mrpt::poses::CPose3D(p));
	}

	// Store here all sensor intrinsic parameters. This obj will be copied as a
	// "pattern" to fill it with actual scan data.
	mrpt::obs::CObservationImage sensor_params_;

	std::mutex last_obs_cs_;
	/** Last simulated scan */
	mrpt::obs::CObservationImage::Ptr last_obs_;
	mrpt::obs::CObservationImage::Ptr last_obs2gui_;

	std::shared_ptr<mrpt::opengl::CFBORender> fbo_renderer_rgb_;

	/** Whether gl_* have to be updated upon next call of
	 * internalGuiUpdate() from last_scan2gui_ */
	bool gui_uptodate_ = false;

	std::optional<TSimulContext> has_to_render_;
	std::mutex has_to_render_mtx_;

	float rgbClipMin_ = 1e-2, rgbClipMax_ = 1e+4;

	CameraDistortionOptions distortion_;

	mrpt::viz::CSetOfObjects::Ptr gl_sensor_origin_, gl_sensor_origin_corner_;
	mrpt::viz::CSetOfObjects::Ptr gl_sensor_fov_, gl_sensor_frustum_;
};
}  // namespace mvsim
