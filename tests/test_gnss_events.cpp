/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// GNSS sensor: fix type, degradation events (outages, quality changes,
// position jumps) and no-coverage regions.

#include <mrpt/core/format.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/topography/conversions.h>
#include <mvsim/Sensors/GNSS.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <iostream>
#include <map>
#include <utility>
#include <vector>

#include "test_utils.h"

int g_failures = 0;

namespace
{
const char* kWorldXml = R"(
<mvsim_world version="1.0">
  <simul_timestep>0.01</simul_timestep>
  <georeference>
    <latitude>36.0</latitude> <longitude>-2.0</longitude> <height>100.0</height>
  </georeference>
  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
    <sensor class="gnss" name="gps1">
      <pose_3d>0 0 0 0 0 0</pose_3d>
      <sensor_period>0.1</sensor_period>
      <horizontal_std_noise>0.01</horizontal_std_noise>
      <vertical_std_noise>0.01</vertical_std_noise>
      <fix_type>rtk_fixed</fix_type>
      <event start="2.0" end="3.0" outage="true" />
      <event start="4.0" end="5.0" fix_type="rtk_float" horizontal_std_noise="0.3" />
      <event start="6.0" end="7.0" offset="5.0 0 0" />
      <event start="8.0" end="9.0" fix_type="no_fix" />
    </sensor>
  </vehicle>
</mvsim_world>
)";

struct Sample
{
	double t;
	mrpt::obs::GnssFixType fixType;
	int ggaQuality;
	double east;  //!< [m] wrt the georeference
	double covXX;
};
}  // namespace

int main()
{
	// Invalid event values must be rejected:
	const std::vector<std::pair<std::string, std::string>> invalid = {
		{"horizontal_std_noise=\"0.3\"", "horizontal_std_noise=\"-0.3\""},
		{"horizontal_std_noise=\"0.3\"", "horizontal_std_noise=\"inf\""},
		{"<event start=\"2.0\" end", "<event end"},
	};
	for (const auto& [from, to] : invalid)
	{
		std::string xml = kWorldXml;
		xml.replace(xml.find(from), from.size(), to);
		bool thrown = false;
		try
		{
			mvsim::World world;
			world.headless(true);
			world.load_from_XML(xml, ".");
		}
		catch (const std::exception&)
		{
			thrown = true;
		}
		EXPECT_TRUE(thrown);
	}

	try
	{
		mvsim::World world;
		world.headless(true);
		world.load_from_XML(kWorldXml, ".");

		std::vector<Sample> samples;
		world.registerCallbackOnObservation(
			[&](const mvsim::Simulable&, const mrpt::obs::CObservation::Ptr& o)
			{
				auto gps = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(o);
				if (!gps)
				{
					return;
				}
				const auto& gga = gps->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>();
				const mrpt::topography::TGeodeticCoords ref(36.0, -2.0, 100.0);
				const mrpt::topography::TGeodeticCoords pt(
					gga.fields.latitude_degrees, gga.fields.longitude_degrees,
					gga.fields.altitude_meters);
				mrpt::math::TPoint3D enu;
				mrpt::topography::geodeticToENU_WGS84(pt, enu, ref);
				samples.push_back(
					{world.get_simul_time(), gps->fix_type, gga.fields.fix_quality, enu.x,
					 (*gps->covariance_enu)(0, 0)});
			});

		for (int i = 0; i < 1000; i++)
		{
			world.run_simulation(0.01);
		}

		std::map<int, int> countPerSecond;
		for (const auto& s : samples)
		{
			const int sec = static_cast<int>(s.t - 1e-6);
			countPerSecond[sec]++;
			if (sec == 2)
			{
				continue;  // outage
			}
			if (sec == 4)
			{
				EXPECT_TRUE(s.fixType == mrpt::obs::GnssFixType::RTK_FLOAT);
				EXPECT_TRUE(s.ggaQuality == 5);
				EXPECT_NEAR(s.covXX, 0.09, 1e-9);
			}
			else if (sec == 8)
			{
				EXPECT_TRUE(s.fixType == mrpt::obs::GnssFixType::NO_FIX);
				EXPECT_TRUE(s.ggaQuality == 0);
			}
			else
			{
				EXPECT_TRUE(s.fixType == mrpt::obs::GnssFixType::RTK_FIXED);
				EXPECT_TRUE(s.ggaQuality == 4);
				EXPECT_NEAR(s.covXX, 1e-4, 1e-9);
			}
			// Position jump during [6,7), the robot is still at the origin:
			EXPECT_NEAR(s.east, sec == 6 ? 5.0 : 0.0, sec == 4 ? 1.5 : 0.1);
		}
		EXPECT_TRUE(countPerSecond[2] == 0);
		EXPECT_GT(countPerSecond[1], 8);
		EXPECT_GT(countPerSecond[3], 8);
		std::cout << "GNSS samples: " << samples.size() << "\n";
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << std::endl;
		return 1;
	}

	// No-coverage regions are in world coordinates, also when the world is
	// rotated wrt ENU: with a rotation of 90 deg and a region around (10,0), a
	// robot inside it gets no readings, while robots at (0,-10) and (0,10),
	// where the region would be if it were taken in ENU coordinates with either
	// rotation sign, do.
	const auto region = [](double xMin, double xMax, double yMin, double yMax)
	{
		return mrpt::format(
			"<element class=\"property_region\"><property_name>gps_no_coverage</property_name>"
			"<value_bool>true</value_bool><x_min>%f</x_min><x_max>%f</x_max><y_min>%f</y_min>"
			"<y_max>%f</y_max></element>",
			xMin, xMax, yMin, yMax);
	};
	const std::string geo = "<height>100.0</height>";
	try
	{
		const std::vector<std::pair<std::string, bool>> cases = {
			{"10 0 0", false}, {"0 -10 0", true}, {"0 10 0", true}};
		for (const auto& [pose, expectReadings] : cases)
		{
			std::string xml = kWorldXml;
			xml.replace(
				xml.find(geo), geo.size(),
				geo + "<world_to_enu_rotation_deg>90</world_to_enu_rotation_deg>");
			xml.replace(xml.find("<vehicle"), 0, region(9.0, 11.0, -1.0, 1.0));
			const std::string init = "<init_pose>0 0 0</init_pose>";
			xml.replace(xml.find(init), init.size(), "<init_pose>" + pose + "</init_pose>");

			mvsim::World world;
			world.headless(true);
			world.load_from_XML(xml, ".");
			int n = 0;
			world.registerCallbackOnObservation(
				[&](const mvsim::Simulable&, const mrpt::obs::CObservation::Ptr& o)
				{ n += std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(o) ? 1 : 0; });
			for (int i = 0; i < 100; i++)
			{
				world.run_simulation(0.01);
			}
			if (expectReadings)
			{
				EXPECT_GT(n, 5);
			}
			else
			{
				EXPECT_TRUE(n == 0);
			}
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << std::endl;
		return 1;
	}

	// The same in a UTM world, where world coordinates are UTM coordinates:
	try
	{
		double utmX = 0;
		double utmY = 0;
		int zone = 0;
		char band = 0;
		mrpt::topography::GeodeticToUTM(36.0, -2.0, utmX, utmY, zone, band);

		std::string xml = kWorldXml;
		xml.replace(xml.find(geo), geo.size(), geo + "<world_is_utm>true</world_is_utm>");
		xml.replace(xml.find("<vehicle"), 0, region(utmX + 9, utmX + 11, utmY - 1, utmY + 1));

		mvsim::World world;
		world.headless(true);
		world.load_from_XML(xml, ".");
		const auto gnss = std::dynamic_pointer_cast<mvsim::GNSS>(
			world.getListOfVehicles().begin()->second->getSensors().at(0));
		ASSERT_(gnss);
		mrpt::random::CRandomGenerator rng;
		const auto readingAt = [&](double x)
		{
			return gnss->simulateReading(
				mrpt::poses::CPose3D(x, utmY, 100.0, 0, 0, 0), 0.5, mrpt::Clock::now(), rng);
		};
		EXPECT_TRUE(readingAt(utmX + 10) == nullptr);
		EXPECT_TRUE(readingAt(utmX) != nullptr);
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
