/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Video recording of the 3D view (see VideoRecorder.h): the World side.

#include <mrpt/core/bits_math.h>
#include <mrpt/imgui/CImGuiSceneView.h>
#include <mvsim/World.h>

using namespace mvsim;

void World::parse_tag_video_recording(const XmlParserContext& ctx)
{
	VideoRecorder::Options opts;
	const bool autostart = opts.loadFromXML(*ctx.node, user_defined_variables());
	videoRecorder_.setDefaultOptions(opts, autostart);
}

void World::internalPostSimulStepForVideo()
{
	const bool wasRecording = videoRecorder_.isRecording();

	bool closeSimulator = false;
	if (videoRecorder_.onSimulationStep(get_simul_time(), closeSimulator))
	{
		// The simulation waits for the frame, as for OpenGL sensors:
		mark_as_pending_running_sensors_on_3D_scene();
	}

	// Started (from the world file) or completed here:
	if (const auto st = videoRecorder_.status(); st.recording != wasRecording)
	{
		if (st.recording)
		{
			MRPT_LOG_INFO_STREAM("Recording video to: " << st.output_file);
		}
		else if (!st.error.empty())
		{
			MRPT_LOG_ERROR_STREAM("Video recording failed: " << st.error);
		}
		else
		{
			MRPT_LOG_INFO_STREAM(
				"Video saved: " << st.output_file << " (" << st.frames << " frames, " << st.duration
								<< " s)");
		}
	}
	if (closeSimulator)
	{
		MRPT_LOG_INFO("Video recording done: closing the simulator.");
		simulator_must_close(true);
	}
}

void World::internalRenderVideoFrame()
{
	if (!videoRecorder_.hasPendingFrame())
	{
		return;
	}
	const auto opts = videoRecorder_.currentOptions();
	if (!opts)
	{
		return;
	}

	// Only set by the GUI thread, so also needed in headless mode:
	worldVisual_->getViewport()->setViewportClipDistances(
		guiOptions_.clip_plane_min, guiOptions_.clip_plane_max);

	videoRecorder_.renderPendingFrame(
		*worldVisual_, internalVideoCamera(*opts),
		[this]() { return sensor_has_to_create_egl_context(); });
}

mrpt::viz::CCamera World::internalVideoCamera(const VideoRecorder::Options& opts) const
{
	mrpt::viz::CCamera cam;

	const bool fromGui = opts.camera == "gui";
	if (fromGui && gui_.sceneView)
	{
		// What the user sees in the GUI:
		gui_.sceneView->cameraController.applyTo(cam);
		return cam;
	}

	// An orbit camera: from the recording options, or (in headless mode)
	// from the <gui> ones:
	const std::string& follow = fromGui ? guiOptions_.follow_vehicle : opts.follow_vehicle;
	mrpt::math::TPoint3D target = fromGui ? guiOptions_.camera_point_to : opts.cam_point_to;
	double azimuth = fromGui ? guiOptions_.camera_azimuth_deg : opts.cam_azimuth;

	if (!follow.empty())
	{
		if (const auto it = vehicles_.find(follow); it != vehicles_.end())
		{
			const auto p = it->second->getCPose3D();
			if (fromGui)
			{
				// As the GUI does:
				target = {p.x(), p.y(), p.z()};
			}
			else
			{
				// Offset in the vehicle frame, ignoring its pitch and roll:
				const mrpt::poses::CPose3D heading(p.x(), p.y(), p.z(), p.yaw(), 0, 0);
				target = heading.composePoint(opts.cam_point_to);
				if (opts.follow_heading)
				{
					azimuth += mrpt::RAD2DEG(p.yaw());
				}
			}
		}
	}

	cam.setPointingAt(target + worldRenderOffset());
	cam.setZoomDistance(
		static_cast<float>(fromGui ? guiOptions_.camera_distance : opts.cam_distance));
	cam.setAzimuthDegrees(static_cast<float>(azimuth));
	cam.setElevationDegrees(
		static_cast<float>(fromGui ? guiOptions_.camera_elevation_deg : opts.cam_elevation));
	cam.setProjectiveModel(!(fromGui ? guiOptions_.ortho : opts.ortho));
	cam.setProjectiveFOVdeg(static_cast<float>(fromGui ? guiOptions_.fov_deg : opts.fov_deg));
	return cam;
}
