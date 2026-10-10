/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Ground texture placed by its geodetic corners (e.g. an orthophoto): each
// image quadrant must be seen at its geodetic location, in a world rotated
// wrt ENU. Requires off-screen OpenGL rendering (skipped if not available).

#include <mrpt/img/CImage.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/topography/conversions.h>
#include <mvsim/World.h>

#include <atomic>
#include <cmath>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "test_utils.h"

int g_failures = 0;

namespace
{
constexpr double kLat0 = 36.0;
constexpr double kLon0 = -2.0;
constexpr double kRotDeg = 30.0;
constexpr double kHalfDeg = 1e-4;  // ~ +-10 m around the origin
constexpr double kCamHeight = 40.0;
constexpr int kW = 200;
constexpr int kH = 200;
constexpr double kF = 100.0;  // => field of view 90 deg

// Quadrant colors of the texture (north-up image):
const mrpt::img::TColor kNW(255, 0, 0);
const mrpt::img::TColor kNE(0, 255, 0);
const mrpt::img::TColor kSW(0, 0, 255);
const mrpt::img::TColor kSE(255, 255, 0);

std::string makeTexture()
{
	mrpt::img::CImage img(64, 64, mrpt::img::CH_RGB);
	for (int v = 0; v < 64; v++)
	{
		for (int u = 0; u < 64; u++)
		{
			const bool north = v < 32;	// image rows go from north to south
			const bool east = u >= 32;
			img.setPixel({u, v}, north ? (east ? kNE : kNW) : (east ? kSE : kSW));
		}
	}
	const auto file = mrpt::system::getTempFileName() + ".png";
	ASSERT_(img.saveToFile(file));
	return file;
}

std::string worldXml(const std::string& texFile)
{
	return mrpt::format(
		R"(
<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <georeference>
    <latitude>%f</latitude> <longitude>%f</longitude> <height>0</height>
    <world_to_enu_rotation_deg>%f</world_to_enu_rotation_deg>
  </georeference>
  <element class="horizontal_plane">
    <texture>%s</texture>
    <geo_corner_sw>%.9f %.9f</geo_corner_sw>
    <geo_corner_ne>%.9f %.9f</geo_corner_ne>
  </element>
  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
    <sensor class="camera" name="cam">
      <pose_3d>0 0 %f -90.0 0.0 180.0</pose_3d>
      <sensor_period>0.05</sensor_period>
      <ncols>%d</ncols> <nrows>%d</nrows>
      <cx>%f</cx> <cy>%f</cy> <fx>%f</fx> <fy>%f</fy>
      <clip_min>0.1</clip_min> <clip_max>200</clip_max>
      <preview_win_visible>false</preview_win_visible>
    </sensor>
  </vehicle>
</mvsim_world>
)",
		kLat0, kLon0, kRotDeg, texFile.c_str(), kLat0 - kHalfDeg, kLon0 - kHalfDeg,
		kLat0 + kHalfDeg, kLon0 + kHalfDeg, kCamHeight, kW, kH, kW / 2.0, kH / 2.0, kF, kF);
}

mrpt::obs::CObservationImage::Ptr getImage(const std::string& xml)
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
			catch (const std::exception& e)
			{
				std::cerr << e.what() << std::endl;
			}
		});
	for (int i = 0; i < 30 && !world.simulator_must_close(); i++)
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
	std::lock_guard<std::mutex> lck(obsMtx);
	return img;
}

/** Checks the dominant color of the pixel where a geodetic point is seen */
void checkColorAt(
	const mrpt::img::CImage& im, double lat, double lon, const mrpt::img::TColor& expected)
{
	mrpt::math::TPoint3D enu;
	mrpt::topography::geodeticToENU_WGS84(
		mrpt::topography::TGeodeticCoords(lat, lon, 0), enu,
		mrpt::topography::TGeodeticCoords(kLat0, kLon0, 0));
	// world = Rz(-rot) * ENU
	const double th = -mrpt::DEG2RAD(kRotDeg);
	const double x = std::cos(th) * enu.x - std::sin(th) * enu.y;
	const double y = std::sin(th) * enu.x + std::cos(th) * enu.y;
	// Downward camera at the origin: image right = -y, image down = -x
	const int u = static_cast<int>(std::lround(kW / 2.0 + kF * (-y) / kCamHeight));
	const int v = static_cast<int>(std::lround(kH / 2.0 + kF * (-x) / kCamHeight));
	EXPECT_TRUE(u >= 0 && u < kW && v >= 0 && v < kH);
	const int r = im.at<uint8_t>(u, v, 0);
	const int g = im.at<uint8_t>(u, v, 1);
	const int b = im.at<uint8_t>(u, v, 2);
	std::cout << "pixel (" << u << "," << v << ") RGB=" << r << "," << g << "," << b << "\n";
	const auto high = [](int c) { return c > 100; };
	EXPECT_TRUE(high(r) == (expected.R > 128));
	EXPECT_TRUE(high(g) == (expected.G > 128));
	EXPECT_TRUE(high(b) == (expected.B > 128));
}
}  // namespace

int main()
{
	try
	{
		const auto obs = getImage(worldXml(makeTexture()));
		if (!obs)
		{
			std::cout << "[SKIPPED] No OpenGL rendering available.\n";
			return 0;
		}
		const double q = kHalfDeg / 2;
		checkColorAt(obs->image, kLat0 + q, kLon0 - q, kNW);
		checkColorAt(obs->image, kLat0 + q, kLon0 + q, kNE);
		checkColorAt(obs->image, kLat0 - q, kLon0 - q, kSW);
		checkColorAt(obs->image, kLat0 - q, kLon0 + q, kSE);
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
}
