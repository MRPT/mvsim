/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
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

	void simulateOn3DScene(mrpt::opengl::COpenGLScene& gl_scene) override;

	void freeOpenGLResources() override;

   protected:
	virtual void internalGuiUpdate(
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
		const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
		bool childrenOnly) override;

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

	mrpt::opengl::CSetOfObjects::Ptr gl_sensor_origin_,
		gl_sensor_origin_corner_;
	mrpt::opengl::CSetOfObjects::Ptr gl_sensor_fov_, gl_sensor_frustum_;
};
}  // namespace mvsim
