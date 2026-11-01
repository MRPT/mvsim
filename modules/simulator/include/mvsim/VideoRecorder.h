/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint3D.h>
#include <mrpt/viz/CCamera.h>
#include <mrpt/viz/Scene.h>

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace rapidxml
{
template <class Ch>
class xml_node;
}

namespace mrpt::opengl
{
class CFBORender;
}

namespace mvsim
{
/** Records videos of the 3D view, in simulation time.
 *
 * Frames are taken every 1/fps seconds of simulated time, and the simulation
 * waits for each one to be rendered, as for camera sensors, so videos play
 * at real speed and never drop frames, however fast or slow the simulation
 * runs. Frames are rendered off-screen at a fixed resolution, and piped to an
 * `ffmpeg` process, or saved as image files if the output file name has a
 * printf-like pattern (e.g. "frames/img_%06d.png").
 *
 * Thread safety: start(), stop() and status() can be called from any thread.
 * onSimulationStep() runs in the simulation thread, and renderPendingFrame()
 * and freeOpenGLResources() in the rendering thread.
 *
 * \ingroup mvsim_simulator_module
 */
class VideoRecorder
{
   public:
	VideoRecorder();
	~VideoRecorder();

	VideoRecorder(const VideoRecorder&) = delete;
	VideoRecorder& operator=(const VideoRecorder&) = delete;

	struct Options
	{
		/// Output video file. Empty: "mvsim_video_<date>_<time>.mp4".
		/// With a printf pattern (e.g. "img_%06d.png"): one image per frame.
		std::string output_file;
		double fps = 30.0;	//!< In simulated time
		unsigned int width = 1280;
		unsigned int height = 720;
		double start_time = 0;	//!< Simulation time to start, if autostarted [s]
		double duration = 0;  //!< Simulated time to record [s] (0=until stopped)
		bool close_simulator_when_done = false;	 //!< When duration is reached
		std::string ffmpeg_options = "-c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p";

		/// "gui": the GUI view (in headless mode, the <gui> camera options).
		/// "orbit": the camera parameters below.
		std::string camera = "gui";
		std::string follow_vehicle;	 //!< Vehicle to look at (empty=none)
		/// Azimuth relative to the heading of the followed vehicle
		bool follow_heading = false;
		double cam_distance = 10.0;
		double cam_azimuth = 45.0;	//!< [deg]
		double cam_elevation = 20.0;  //!< [deg]
		/// Target point: in the vehicle frame if following one, else global
		mrpt::math::TPoint3D cam_point_to{0, 0, 0};
		double fov_deg = 60.0;
		bool ortho = false;

		/** Parses the `<video_recording>` XML block (its children). Returns
		 * the value of its "enabled" attribute (autostart). */
		bool loadFromXML(
			const rapidxml::xml_node<char>& node, const std::map<std::string, std::string>& vars);
	};

	/// While not recording, that of the last recording, if any.
	struct Status
	{
		bool recording = false;
		std::string output_file;
		size_t frames = 0;	//!< Written to the video so far
		double duration = 0;  //!< Simulated time recorded [s]
		std::string error;	//!< Last encoder error, if any
	};

	/** Starts recording (stopping the current recording, if any). The first
	 * frame is taken right after the next simulation step.
	 * \exception std::exception On invalid options, or if the encoder (ffmpeg)
	 * can not be launched. */
	void start(const Options& opts);

	/** Stops recording, and waits for the video file to be complete. Returns
	 * the final status. It does nothing if not recording. */
	Status stop();

	Status status() const;

	bool isRecording() const;

	/// Options of the current recording, if any.
	std::optional<Options> currentOptions() const;

	/** Options from the world file (used by the GUI and as defaults for the
	 * ZMQ service), and whether to start recording at their start_time. */
	void setDefaultOptions(const Options& opts, bool autostart);
	Options defaultOptions() const;

	/** To be called from the simulation thread after each time step: marks a
	 * frame as pending when its time comes. Returns true if one is pending,
	 * so the caller must wait for the rendering thread to call
	 * renderPendingFrame().
	 * It also handles the autostart, and the end of a recording with a
	 * duration (returned in \a closeSimulator if it must be closed then).
	 */
	bool onSimulationStep(double simTime, bool& closeSimulator);

	/// Simulation time of the next frame, if recording.
	std::optional<double> nextFrameTime() const;

	bool hasPendingFrame() const;

	/** Renders the pending frame, if any, and hands it to the encoder. It may
	 * wait for the encoder, if it is slower than the simulation.
	 * \param hasToCreateEGLContext Called when an off-screen renderer is
	 * created: whether it must create its own OpenGL context (headless mode).
	 */
	void renderPendingFrame(
		const mrpt::viz::Scene& scene, const mrpt::viz::CCamera& camera,
		const std::function<bool()>& hasToCreateEGLContext);

	/// Must be called from the rendering thread before it exits.
	void freeOpenGLResources();

   private:
	class Encoder;	// encoder thread and process (see .cpp)

	mutable std::mutex mtx_;
	std::optional<Options> opts_;  //!< Set while recording
	Options defaultOpts_;
	bool autostartPending_ = false;

	std::shared_ptr<Encoder> encoder_;
	std::optional<double> startTime_;  //!< Simulation time of the first frame
	double nextFrameTime_ = 0;
	size_t pendingFrameRepeats_ = 0;  //!< >0: a frame must be rendered
	size_t framesRequested_ = 0;
	Status last_;  //!< Of the last recording

	/** Only used from the rendering thread. One per frame size: they are
	 * kept until freeOpenGLResources(), since in headless mode the first one
	 * may own the OpenGL context of all sensors. */
	std::vector<std::shared_ptr<mrpt::opengl::CFBORender>> fbos_;

	Status internalStop();	// mtx_ must be locked
	Status internalStatus() const;	// mtx_ must be locked
};

}  // namespace mvsim
