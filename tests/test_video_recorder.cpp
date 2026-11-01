/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Video recording in simulation time: number of frames, frame repetition for
// long time steps, image contents, and the ffmpeg output (if available).
// Requires off-screen OpenGL rendering (skipped if not available).

#include <mrpt/core/format.h>
#include <mrpt/img/CImage.h>
#include <mrpt/io/vector_loadsave.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/World.h>

#include <atomic>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "test_utils.h"

int g_failures = 0;

namespace
{
// A red box in front of an orbit camera:
std::string worldXml(const std::string& recordingOptions, double dt)
{
	return mrpt::format(
			   R"(
<mvsim_world version="1.0">
  <simul_timestep>%f</simul_timestep>
  <element class="horizontal_plane">
    <cutout>-20 -20 20 20</cutout>
    <color>#808080</color>
  </element>
  <block name="mark">
    <static>true</static> <zmin>0</zmin> <zmax>1.0</zmax>
    <color>#ff0000</color>
    <shape><pt>-0.5 -0.5</pt><pt>0.5 -0.5</pt><pt>0.5 0.5</pt><pt>-0.5 0.5</pt></shape>
    <init_pose>0 0 0</init_pose>
  </block>
  <video_recording enabled="true">
    <width>64</width> <height>48</height>
    <camera>orbit</camera>
    <cam_point_to>0 0 0.5</cam_point_to>
    <cam_distance>3</cam_distance> <cam_azimuth>30</cam_azimuth> <cam_elevation>20</cam_elevation>
)",
			   dt) +
		   recordingOptions + R"(
  </video_recording>
</mvsim_world>
)";
}

/** Runs the simulation for `simTime` seconds, with a rendering thread as in
 *  headless mode. */
void runWorld(mvsim::World& world, double simTime, double dt)
{
	std::atomic_bool stop = false;
	std::exception_ptr renderError;
	std::thread th(
		[&]()
		{
			try
			{
				while (!stop && !world.simulator_must_close())
				{
					world.internalGraphicsLoopTasksForSimulation();
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
				world.internalFreeOpenGLResourcesForSimulation();
			}
			catch (...)
			{
				renderError = std::current_exception();
				stop = true;
			}
		});
	const int nSteps = static_cast<int>(std::lround(simTime / dt));
	for (int i = 0; i < nSteps && !stop && !world.simulator_must_close(); i++)
	{
		world.run_simulation(dt);
	}
	stop = true;
	th.join();
	if (renderError)
	{
		std::rethrow_exception(renderError);
	}
}

size_t countFiles(const std::string& pattern, size_t maxFiles)
{
	size_t n = 0;
	while (n < maxFiles &&
		   mrpt::system::fileExists(mrpt::format(pattern.c_str(), static_cast<int>(n))))
	{
		n++;
	}
	return n;
}

size_t countRedPixels(const mrpt::img::CImage& im)
{
	size_t n = 0;
	for (int v = 0; v < static_cast<int>(im.getHeight()); v++)
	{
		for (int u = 0; u < static_cast<int>(im.getWidth()); u++)
		{
			const int r = im.at<uint8_t>(u, v, 0);
			const int g = im.at<uint8_t>(u, v, 1);
			const int b = im.at<uint8_t>(u, v, 2);
			if (r > g + 60 && r > b + 60)
			{
				n++;
			}
		}
	}
	return n;
}

void testImageSequence(const std::string& dir, double dt, bool closeWhenDone)
{
	const std::string pattern =
		dir + mrpt::format("/dt%03d_img_%%04d.png", static_cast<int>(dt * 1000));

	mvsim::World world;
	world.headless(true);
	world.load_from_XML(
		worldXml(
			"<output_file>" + pattern +
				"</output_file> <fps>20</fps> <duration>0.5</duration>"
				"<close_simulator_when_done>" +
				std::string(closeWhenDone ? "true" : "false") + "</close_simulator_when_done>",
			dt),
		".");

	runWorld(world, 1.0, dt);

	// 0.5 s at 20 fps: frames 0..9, whatever the time step:
	const size_t n = countFiles(pattern, 100);
	std::cout << "dt=" << dt << ": " << n << " frames\n";
	EXPECT_TRUE(n == 10U);
	EXPECT_TRUE(!world.videoRecorder().isRecording());
	EXPECT_TRUE(world.simulator_must_close() == closeWhenDone);
	if (closeWhenDone)
	{
		// It closed right after the last frame:
		EXPECT_LT(world.get_simul_time(), 0.5 + 2 * dt + 1e-6);
	}

	mrpt::img::CImage im;
	EXPECT_TRUE(im.loadFromFile(mrpt::format(pattern.c_str(), 9)));
	EXPECT_TRUE(im.getWidth() == 64U && im.getHeight() == 48U);
	const size_t red = countRedPixels(im);
	std::cout << "Red pixels: " << red << " / " << 64 * 48 << "\n";
	EXPECT_GT(red, 100);
	EXPECT_LT(red, 64 * 48 / 2);
}

void testFfmpeg(const std::string& dir)
{
	const std::string file = dir + "/video.mp4";

	mvsim::World world;
	world.headless(true);
	world.load_from_XML(
		worldXml(
			"<output_file>" + file + "</output_file> <fps>25</fps> <duration>1.0</duration>", 0.01),
		".");
	runWorld(world, 1.2, 0.01);

	EXPECT_TRUE(!world.videoRecorder().isRecording());
	EXPECT_TRUE(world.videoRecorder().status().error.empty());
	EXPECT_TRUE(mrpt::system::fileExists(file));
	EXPECT_GT(mrpt::system::getFileSize(file), 1000);

	// Its number of frames, according to ffprobe:
	const std::string cmd =
		"ffprobe -v error -count_frames -select_streams v:0 -show_entries stream=nb_read_frames "
		"-of csv=p=0 \"" +
		file + "\" > \"" + dir + "/nframes.txt\"";
	if (std::system(cmd.c_str()) == 0)
	{
		if (const auto lines = mrpt::io::loadTextFile(dir + "/nframes.txt");
			lines && !lines->empty())
		{
			std::cout << "ffprobe frames: " << lines->front() << "\n";
			EXPECT_TRUE(std::atoi(lines->front().c_str()) == 25);
		}
	}
}

void testInvalidOptions()
{
	mvsim::VideoRecorder rec;
	mvsim::VideoRecorder::Options o;
	o.output_file = "img_%04d.png";
	o.fps = 0;
	bool failed = false;
	try
	{
		rec.start(o);
	}
	catch (const std::exception&)
	{
		failed = true;
	}
	EXPECT_TRUE(failed);
	EXPECT_TRUE(!rec.isRecording());

	// Output file names are printf formats for image sequences: only one
	// integer conversion (and "%%") is accepted.
	for (const char* name : {"img_%s.png", "img_%d_%d.png", "img_%f.png", "img_%"})
	{
		o = mvsim::VideoRecorder::Options();
		o.output_file = name;
		bool rejected = false;
		try
		{
			rec.start(o);
		}
		catch (const std::exception&)
		{
			rejected = true;
		}
		EXPECT_TRUE(rejected);
		EXPECT_TRUE(!rec.isRecording());
	}
}
}  // namespace

int main()
{
	try
	{
		testInvalidOptions();

		// Skip only if off-screen rendering is not available at all:
		try
		{
			mrpt::opengl::CFBORender probe(16, 16);
		}
		catch (const std::exception& e)
		{
			std::cout << "[SKIPPED] No OpenGL rendering available: " << e.what() << "\n";
			return g_failures ? 1 : 0;
		}

		const std::string dir = mrpt::system::getTempFileName() + "_mvsim_video";
		mrpt::system::createDirectory(dir);

		testImageSequence(dir, 0.01, false);
		// Time step longer than the frame period: frames are repeated.
		testImageSequence(dir, 0.1, true);

		if (std::system("ffmpeg -version > /dev/null 2>&1") == 0)
		{
			testFfmpeg(dir);
		}
		else
		{
			std::cout << "[SKIPPED] ffmpeg not found\n";
		}

		mrpt::system::deleteFilesInDirectory(dir, true);
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return g_failures ? 1 : 0;
}
