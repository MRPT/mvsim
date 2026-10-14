/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Camera with plumb_bob lens distortion: objects must appear where the
// distorted pinhole model projects them. Requires off-screen OpenGL
// rendering (skipped if not available).

#include <mrpt/img/camera_geometry.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/version.h>
#include <mvsim/Sensors/CameraSensor.h>
#include <mvsim/World.h>

#include <atomic>
#include <cmath>
#include <exception>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "test_utils.h"

int g_failures = 0;

namespace
{
// Camera 2 m above the ground at x=3, looking down; a red square on the
// ground near a corner of the image.
std::string worldXml(const std::string& distortion)
{
	return R"(
<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <element class="horizontal_plane">
    <cutout>-20 -20 20 20</cutout>
    <color>#808080</color>
  </element>
  <block name="mark">
    <static>true</static> <zmin>0</zmin> <zmax>0.05</zmax>
    <color>#ff0000</color>
    <shape><pt>-0.08 -0.08</pt><pt>0.08 -0.08</pt><pt>0.08 0.08</pt><pt>-0.08 0.08</pt></shape>
    <init_pose>2.4 -1.0 0</init_pose>
  </block>
  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
    <sensor class="camera" name="cam">
      <pose_3d>3.0 0.0 2.0 -90.0 0.0 180.0</pose_3d>
      <sensor_period>0.05</sensor_period>
      <ncols>160</ncols> <nrows>120</nrows>
      <cx>80</cx> <cy>60</cy> <fx>80</fx> <fy>80</fy>
      <clip_min>0.01</clip_min> <clip_max>100</clip_max>
      <preview_win_visible>false</preview_win_visible>
      )" + distortion +
		   R"(
    </sensor>
  </vehicle>
</mvsim_world>
)";
}

[[maybe_unused]] mrpt::obs::CObservationImage::Ptr getImage(const std::string& xml)
{
	mvsim::World world;
	world.headless(true);
	world.load_from_XML(xml, ".");

	std::mutex obsMtx;
	mrpt::obs::CObservationImage::Ptr img;
	world.registerCallbackOnObservation(
		[&](const mvsim::Simulable&, const mrpt::obs::CObservation::Ptr& o)
		{
			if (auto i = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(o); i)
			{
				std::lock_guard<std::mutex> lck(obsMtx);
				img = i;
			}
		});

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
					std::this_thread::sleep_for(std::chrono::milliseconds(2));
				}
				world.internalFreeOpenGLResourcesForSimulation();
			}
			catch (...)
			{
				renderError = std::current_exception();
				stop = true;
			}
		});
	for (int i = 0; i < 30 && !stop && !world.simulator_must_close(); i++)
	{
		world.run_simulation(0.01);
		std::lock_guard<std::mutex> lck(obsMtx);
		if (img)
		{
			break;
		}
	}
	stop = true;
	th.join();
	if (renderError)
	{
		std::rethrow_exception(renderError);
	}
	std::lock_guard<std::mutex> lck(obsMtx);
	return img;
}

/** Centroid of the red pixels */
[[maybe_unused]] std::optional<mrpt::img::TPixelCoordf> redCentroid(const mrpt::img::CImage& im)
{
	double su = 0;
	double sv = 0;
	size_t n = 0;
	for (int v = 0; v < static_cast<int>(im.getHeight()); v++)
	{
		for (int u = 0; u < static_cast<int>(im.getWidth()); u++)
		{
			const int r = im.at<uint8_t>(u, v, 0);
			const int g = im.at<uint8_t>(u, v, 1);
			const int b = im.at<uint8_t>(u, v, 2);
			if (r > g + 80 && r > b + 80)
			{
				su += u;
				sv += v;
				n++;
			}
		}
	}
	if (n == 0)
	{
		return {};
	}
	return mrpt::img::TPixelCoordf(static_cast<float>(su / n), static_cast<float>(sv / n));
}
/** Whether loading the world with these camera options throws an error
 * containing `msg` */
bool loadFails(const std::string& cameraOptions, const std::string& msg)
{
	try
	{
		mvsim::World w;
		w.headless(true);
		w.load_from_XML(worldXml(cameraOptions));
	}
	catch (const std::exception& e)
	{
		return std::string(e.what()).find(msg) != std::string::npos;
	}
	return false;
}
}  // namespace

int main()
{
	// Invalid values are rejected:
	EXPECT_TRUE(loadFails("<distortion_model>fisheye</distortion_model>", "must be 'none'"));
	EXPECT_TRUE(loadFails("<k1>nan</k1>", "must be finite"));
	EXPECT_TRUE(loadFails("<image_noise_std>inf</image_noise_std>", "non-negative"));
	EXPECT_TRUE(loadFails("<image_noise_std>-1</image_noise_std>", "non-negative"));

#if MRPT_VERSION < MIN_MRPT_VERSION_CAMERA_DISTORTION
	// Not supported: loading must fail with a clear error.
	EXPECT_TRUE(
		loadFails("<distortion_model>plumb_bob</distortion_model> <k1>-0.3</k1>", "require MRPT"));
	EXPECT_TRUE(loadFails("<image_noise_std>2</image_noise_std>", "require MRPT"));
	std::cout << "[SKIPPED] Camera distortion requires a newer MRPT.\n";
	return g_failures ? 1 : 0;
#else
	try
	{
		const std::string distortion = R"(
      <distortion_model>plumb_bob</distortion_model>
      <k1>-0.3</k1> <k2>0.05</k2> <p1>0.002</p1> <p2>-0.001</p2>)";

		// Skip only if off-screen rendering is not available at all:
		try
		{
			mrpt::opengl::CFBORender probe(16, 16);
		}
		catch (const std::exception& e)
		{
			std::cout << "[SKIPPED] No OpenGL rendering available: " << e.what() << "\n";
			return 0;
		}

		const auto obs = getImage(worldXml(distortion));
		EXPECT_TRUE(obs != nullptr);
		if (!obs)
		{
			return 1;
		}
		EXPECT_TRUE(obs->image.getWidth() == 160U && obs->image.getHeight() == 120U);
		EXPECT_TRUE(obs->cameraParams.distortion == mrpt::img::DistortionModel::plumb_bob);

		// Top face center of the mark, in the camera frame (+X right = -y
		// world, +Y down = -x world, +Z = down):
		const mrpt::math::TPoint3D ptCam(1.0, 0.6, 2.0 - 0.05);
		mrpt::img::TPixelCoordf expected;
		mrpt::img::camera_geometry::projectPoint_with_distortion(
			ptCam, obs->cameraParams, expected);
		auto ideal = obs->cameraParams;
		ideal.distortion = mrpt::img::DistortionModel::none;
		mrpt::img::TPixelCoordf expectedNoDist;
		mrpt::img::camera_geometry::projectPoint_with_distortion(ptCam, ideal, expectedNoDist);

		const auto c = redCentroid(obs->image);
		EXPECT_TRUE(c.has_value());
		if (c)
		{
			std::cout << "Red mark at (" << c->x << "," << c->y << "), expected (" << expected.x
					  << "," << expected.y << "), without distortion (" << expectedNoDist.x << ","
					  << expectedNoDist.y << ")\n";
			EXPECT_NEAR(c->x, expected.x, 1.0);
			EXPECT_NEAR(c->y, expected.y, 1.0);
			// The test is only meaningful if distortion moves it clearly:
			EXPECT_GT(
				std::hypot(expected.x - expectedNoDist.x, expected.y - expectedNoDist.y), 3.0);
		}

		// Image noise:
		const auto noisy = getImage(worldXml("<image_noise_std>10</image_noise_std>"));
		const auto clean = getImage(worldXml(""));
		EXPECT_TRUE(noisy && clean);
		if (noisy && clean)
		{
			double sumSq = 0;
			const int u0 = 10;
			for (int v = 10; v < 30; v++)
			{
				for (int u = u0; u < u0 + 20; u++)
				{
					const double d = static_cast<double>(noisy->image.at<uint8_t>(u, v, 1)) -
									 clean->image.at<uint8_t>(u, v, 1);
					sumSq += d * d;
				}
			}
			const double stdDev = std::sqrt(sumSq / 400.0);
			std::cout << "Noise std: " << stdDev << " (expected ~10)\n";
			EXPECT_NEAR(stdDev, 10.0, 2.0);
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << std::endl;
		return 1;
	}
	if (g_failures)
	{
		std::cerr << g_failures << " failure(s)\n";
		return 1;
	}
	std::cout << "All tests passed.\n";
	return 0;
#endif
}
