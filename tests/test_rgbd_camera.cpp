/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// RGBD camera depth images: rendered together with the RGB image (if both
// cameras see the same view) or separately, they must be the same. Requires
// off-screen OpenGL rendering (skipped if not available).

#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/opengl/CFBORender.h>
#include <mrpt/system/CTimeLogger.h>
#include <mvsim/World.h>

#include <atomic>
#include <cmath>
#include <exception>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "test_utils.h"

int g_failures = 0;

namespace
{
constexpr int NCOLS = 160;
constexpr int NROWS = 120;

// Camera 1 m above the ground looking along +X. A wall 3 m ahead covers the
// left half of the image, another one beyond the maximum range (15 m) covers
// the right half.
std::string worldXml(const std::string& cameraOptions)
{
	return R"(
<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <block name="near">
    <static>true</static> <zmin>0</zmin> <zmax>5</zmax>
    <shape><pt>3 0</pt><pt>3.5 0</pt><pt>3.5 10</pt><pt>3 10</pt></shape>
    <init_pose>0 0 0</init_pose>
  </block>
  <block name="far">
    <static>true</static> <zmin>0</zmin> <zmax>20</zmax>
    <shape><pt>20 -30</pt><pt>21 -30</pt><pt>21 30</pt><pt>20 30</pt></shape>
    <init_pose>0 0 0</init_pose>
  </block>
  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
    <sensor class="rgbd_camera" name="cam">
      <pose_3d>0 0 1 0 0 0</pose_3d>
      <relativePoseIntensityWRTDepth>0 0 0 -90 0 -90</relativePoseIntensityWRTDepth>
      <sensor_period>0.05</sensor_period>
      <preview_win_visible>false</preview_win_visible>
      <depth_ncols>160</depth_ncols> <depth_nrows>120</depth_nrows>
      <depth_cx>80</depth_cx> <depth_cy>60</depth_cy> <depth_fx>80</depth_fx> <depth_fy>80</depth_fy>
      <depth_resolution>1e-3</depth_resolution>
      <depth_clip_min>0.01</depth_clip_min> <depth_clip_max>15</depth_clip_max>
      <rgb_ncols>160</rgb_ncols> <rgb_nrows>120</rgb_nrows>
      <rgb_cx>80</rgb_cx> <rgb_cy>60</rgb_cy> <rgb_fx>80</rgb_fx> <rgb_fy>80</rgb_fy>
      <rgb_clip_max>1000</rgb_clip_max>
      )" + cameraOptions +
		   R"(
    </sensor>
  </vehicle>
</mvsim_world>
)";
}

struct Result
{
	mrpt::obs::CObservation3DRangeScan::Ptr obs;
	size_t numDepthRenders = 0;
};

Result getObservation(const std::string& xml)
{
	mvsim::World world;
	world.headless(true);
	world.load_from_XML(xml, ".");

	std::mutex obsMtx;
	Result res;
	world.registerCallbackOnObservation(
		[&](const mvsim::Simulable&, const mrpt::obs::CObservation::Ptr& o)
		{
			if (auto i = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(o); i)
			{
				std::lock_guard<std::mutex> lck(obsMtx);
				res.obs = i;
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
		if (res.obs)
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

	std::map<std::string, mrpt::system::CTimeLogger::TCallStats> stats;
	world.getTimeLogger().getStats(stats);
	if (const auto it = stats.find("sensor.RGBD.renderD"); it != stats.end())
	{
		res.numDepthRenders = it->second.n_calls;
	}
	std::lock_guard<std::mutex> lck(obsMtx);
	return res;
}

float rangeAt(const mrpt::obs::CObservation3DRangeScan& obs, int u, int v)
{
	return obs.rangeImage(v, u) * obs.rangeUnits;
}

void checkDepthImage(const mrpt::obs::CObservation3DRangeScan& obs)
{
	EXPECT_TRUE(obs.hasRangeImage && obs.hasIntensityImage);
	EXPECT_TRUE(obs.rangeImage.cols() == NCOLS && obs.rangeImage.rows() == NROWS);
	// Near wall (left), far wall beyond the maximum range (right):
	EXPECT_NEAR(rangeAt(obs, 20, 60), 3.0, 0.01);
	EXPECT_NEAR(rangeAt(obs, 60, 20), 3.0, 0.01);
	EXPECT_NEAR(rangeAt(obs, 140, 60), 0.0, 1e-6);
}
}  // namespace

int main()
{
	try
	{
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

		const std::string noNoise = "<depth_noise_sigma>0</depth_noise_sigma>";

		// Same view: a single render. A different near clip distance requires
		// a separate depth render:
		const auto single = getObservation(worldXml(noNoise));
		const auto separate =
			getObservation(worldXml(noNoise + "<rgb_clip_min>0.02</rgb_clip_min>"));
		EXPECT_TRUE(single.obs && separate.obs);
		if (!single.obs || !separate.obs)
		{
			return 1;
		}
		EXPECT_TRUE(single.numDepthRenders == 0);
		EXPECT_GT(separate.numDepthRenders, 0U);

		checkDepthImage(*single.obs);
		checkDepthImage(*separate.obs);

		int numDifferent = 0;
		for (int v = 0; v < NROWS; v++)
		{
			for (int u = 0; u < NCOLS; u++)
			{
				const int a = single.obs->rangeImage(v, u);
				const int b = separate.obs->rangeImage(v, u);
				numDifferent += std::abs(a - b) > 1 ? 1 : 0;
			}
		}
		std::cout << "Pixels with different depth: " << numDifferent << "\n";
		EXPECT_TRUE(numDifferent < NCOLS * NROWS / 1000);

		// Valid depths below one range unit are not invalid ranges:
		{
			std::string xml = worldXml(noNoise);
			const std::string res = "<depth_resolution>1e-3</depth_resolution>";
			xml.replace(xml.find(res), res.size(), "<depth_resolution>5</depth_resolution>");
			const auto coarse = getObservation(xml);
			EXPECT_TRUE(coarse.obs != nullptr);
			if (coarse.obs)
			{
				EXPECT_TRUE(coarse.obs->rangeImage(60, 20) == 1);
				EXPECT_TRUE(coarse.obs->rangeImage(60, 140) == 0);
			}
		}

		// Noise: zero mean, the given sigma, and valid ranges only:
		const auto noisy = getObservation(worldXml("<depth_noise_sigma>0.05</depth_noise_sigma>"));
		EXPECT_TRUE(noisy.obs != nullptr);
		if (noisy.obs)
		{
			double sum = 0;
			double sumSq = 0;
			int n = 0;
			for (int v = 0; v < NROWS; v++)
			{
				for (int u = 0; u < NCOLS / 2 - 2; u++)
				{
					if (single.obs->rangeImage(v, u) == 0)
					{
						continue;  // no ground: nothing below the near wall
					}
					const double d = rangeAt(*noisy.obs, u, v) - rangeAt(*single.obs, u, v);
					sum += d;
					sumSq += d * d;
					n++;
				}
				for (int u = NCOLS / 2 + 2; u < NCOLS; u++)
				{
					EXPECT_NEAR(rangeAt(*noisy.obs, u, v), 0.0, 1e-6);
				}
			}
			const double mean = sum / n;
			const double stdDev = std::sqrt(sumSq / n - mean * mean);
			std::cout << "Depth noise mean: " << mean << " std: " << stdDev << "\n";
			EXPECT_NEAR(mean, 0.0, 0.01);
			EXPECT_NEAR(stdDev, 0.05, 0.01);
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
		return 1;
	}

	if (g_failures)
	{
		std::cerr << g_failures << " failures\n";
		return 1;
	}
	std::cout << "All tests passed\n";
	return 0;
}
