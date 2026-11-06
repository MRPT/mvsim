/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// 3D LiDAR per-point RGB (generate_rgb): points get the color of what they
// hit, the geometry does not change, and lidars without it have no color
// fields. Requires off-screen OpenGL rendering (skipped if not available).

#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/opengl/CFBORender.h>
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
// Lidar 1 m above the ground. A red wall 4 m ahead (+X), a blue one 4 m
// behind (-X).
std::string worldXml(const std::string& lidarOptions)
{
	return R"(
<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <block name="red">
    <static>true</static> <zmin>0</zmin> <zmax>5</zmax> <color>#ff0000</color>
    <shape><pt>4 -10</pt><pt>4.5 -10</pt><pt>4.5 10</pt><pt>4 10</pt></shape>
    <init_pose>0 0 0</init_pose>
  </block>
  <block name="blue">
    <static>true</static> <zmin>0</zmin> <zmax>5</zmax> <color>#0000ff</color>
    <shape><pt>-4.5 -10</pt><pt>-4 -10</pt><pt>-4 10</pt><pt>-4.5 10</pt></shape>
    <init_pose>0 0 0</init_pose>
  </block>
  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
    <sensor class="lidar3d" name="lidar">
      <pose_3d>0 0 1 0 0 0</pose_3d>
      <sensor_period>0.05</sensor_period>
      <vert_nrays>16</vert_nrays> <vert_fov_degrees>30</vert_fov_degrees>
      <horz_nrays>360</horz_nrays>
      <range_std_noise>0</range_std_noise>
      <min_range>0.5</min_range> <max_range>50</max_range>
      )" + lidarOptions +
		   R"(
    </sensor>
  </vehicle>
</mvsim_world>
)";
}

mrpt::obs::CObservationPointCloud::Ptr getObservation(const std::string& xml)
{
	mvsim::World world;
	world.headless(true);
	world.load_from_XML(xml, ".");

	std::mutex obsMtx;
	mrpt::obs::CObservationPointCloud::Ptr obs;
	world.registerCallbackOnObservation(
		[&](const mvsim::Simulable&, const mrpt::obs::CObservation::Ptr& o)
		{
			if (auto p = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(o); p)
			{
				std::lock_guard<std::mutex> lck(obsMtx);
				obs = p;
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
		if (obs)
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
	return obs;
}

const mrpt::maps::CGenericPointsMap& points(const mrpt::obs::CObservationPointCloud& obs)
{
	const auto* pts = dynamic_cast<const mrpt::maps::CGenericPointsMap*>(obs.pointcloud.get());
	ASSERT_(pts);
	return *pts;
}

void checkColors(const mrpt::maps::CGenericPointsMap& pts)
{
	using mrpt::maps::CPointsMap;
	const auto* R = pts.getPointsBufferRef_uint8_field(CPointsMap::POINT_FIELD_COLOR_Ru8);
	const auto* G = pts.getPointsBufferRef_uint8_field(CPointsMap::POINT_FIELD_COLOR_Gu8);
	const auto* B = pts.getPointsBufferRef_uint8_field(CPointsMap::POINT_FIELD_COLOR_Bu8);
	EXPECT_TRUE(R && G && B);
	if (!R || !G || !B)
	{
		return;
	}
	EXPECT_TRUE(R->size() == pts.size() && G->size() == pts.size() && B->size() == pts.size());

	size_t nRed = 0;
	size_t nBlue = 0;
	size_t nWrong = 0;
	for (size_t i = 0; i < pts.size(); i++)
	{
		float x = 0;
		float y = 0;
		float z = 0;
		pts.getPoint(i, x, y, z);
		const int r = (*R)[i];
		const int g = (*G)[i];
		const int b = (*B)[i];
		if (x > 3.9f)
		{
			nRed++;
			if (!(r > 2 * g + 20 && r > 2 * b + 20))
			{
				nWrong++;
			}
		}
		else if (x < -3.9f)
		{
			nBlue++;
			if (!(b > 2 * r + 20 && b > 2 * g + 20))
			{
				nWrong++;
			}
		}
	}
	EXPECT_GT(nRed, 100U);
	EXPECT_GT(nBlue, 100U);
	// Allow a few pixels at the wall edges:
	EXPECT_LT(nWrong, (nRed + nBlue) / 50 + 1);
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

		using mrpt::maps::CPointsMap;

		const auto plain =
			getObservation(worldXml("<generate_intensity>false</generate_intensity>"));
		const auto rgb = getObservation(worldXml(
			"<generate_intensity>false</generate_intensity><generate_rgb>true</generate_rgb>"));
		const auto rgbI = getObservation(worldXml(
			"<generate_intensity>true</generate_intensity><generate_rgb>true</generate_rgb>"));
		EXPECT_TRUE(plain && rgb && rgbI);
		if (!plain || !rgb || !rgbI)
		{
			return 1;
		}

		// No color fields unless requested:
		EXPECT_FALSE(points(*plain).hasColor_u8());
		EXPECT_TRUE(points(*rgb).hasColor_u8());

		checkColors(points(*rgb));
		checkColors(points(*rgbI));

		// Same geometry with or without color:
		const auto& p0 = points(*plain);
		const auto& p1 = points(*rgb);
		EXPECT_TRUE(p0.size() == p1.size());
		if (p0.size() == p1.size())
		{
			double maxErr = 0;
			for (size_t i = 0; i < p0.size(); i++)
			{
				float x0 = 0;
				float y0 = 0;
				float z0 = 0;
				float x1 = 0;
				float y1 = 0;
				float z1 = 0;
				p0.getPoint(i, x0, y0, z0);
				p1.getPoint(i, x1, y1, z1);
				maxErr = std::max<double>(
					maxErr, std::abs(x0 - x1) + std::abs(y0 - y1) + std::abs(z0 - z1));
			}
			EXPECT_LT(maxErr, 1e-4);
		}

		// Intensity: red is brighter than blue in luminance (RGB order):
		const auto& pI = points(*rgbI);
		const auto* I = pI.getPointsBufferRef_float_field(CPointsMap::POINT_FIELD_INTENSITY);
		EXPECT_TRUE(I && I->size() == pI.size());
		if (I && I->size() == pI.size())
		{
			double sumRed = 0;
			double sumBlue = 0;
			size_t nRed = 0;
			size_t nBlue = 0;
			for (size_t i = 0; i < pI.size(); i++)
			{
				float x = 0;
				float y = 0;
				float z = 0;
				pI.getPoint(i, x, y, z);
				if (x > 3.9f)
				{
					sumRed += (*I)[i];
					nRed++;
				}
				else if (x < -3.9f)
				{
					sumBlue += (*I)[i];
					nBlue++;
				}
			}
			EXPECT_TRUE(nRed > 0 && nBlue > 0);
			if (nRed > 0 && nBlue > 0)
			{
				EXPECT_GT(sumRed / nRed, sumBlue / nBlue);
			}
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
		return 1;
	}

	if (g_failures != 0)
	{
		std::cerr << g_failures << " failure(s)\n";
		return 1;
	}
	std::cout << "All tests passed.\n";
	return 0;
}
