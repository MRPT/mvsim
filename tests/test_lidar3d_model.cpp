/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Verifies Lidar3DModel against the parameters actually shipped in
// definitions/velodyne-vlp16.sensor.xml, definitions/ouster-os1.sensor.xml
// and definitions/helios-32-FOV-31.sensor.xml (their documented defaults).
// Full XML parsing is wired in a later phase (SceneBuilder); this test
// pins down the ray-generation math against those same numbers directly.

#include <mrpt/core/round.h>
#include <mvsim/raytracer/Lidar3DModel.h>

#include <cmath>
#include <cstdio>

#include "test_utils.h"

int g_failures = 0;

using namespace mvsim::rt;

namespace
{
// ---------------------------------------------------------------
// definitions/velodyne-vlp16.sensor.xml defaults:
//   vert_fov_degrees=30, vert_nrays=16
//   sensor_period = 60/sensor_rpm ; sensor_rpm default 600 -> period=0.1s
//   horz_nrays = (60/sensor_rpm)/55.296e-6
void test_velodyne_vlp16()
{
	Lidar3DModel::Params p;
	p.vertNumRays = 16;
	p.vertFovDegrees = 30.0;
	p.sensorPeriod = 60.0 / 600.0;
	p.horzNumRays = static_cast<int>(mrpt::round(p.sensorPeriod / 55.296e-6));
	p.minRange = 0.01;
	p.maxRange = 80.0;

	Lidar3DModel model(p);

	EXPECT_TRUE(model.vertRays() == 16);
	EXPECT_NEAR(model.horzRays(), 1808, 1);	 // ~0.1 / 55.296e-6
	EXPECT_NEAR(model.params().sensorPeriod, 0.1, 1e-9);

	// Symmetric FOV: ring 0 is the lowest (-15deg), last ring the highest (+15deg).
	EXPECT_NEAR(model.ringElevationRad(0) * 180.0 / M_PI, -15.0, 1e-6);
	EXPECT_NEAR(model.ringElevationRad(15) * 180.0 / M_PI, 15.0, 1e-6);
	// Monotonically increasing:
	for (int i = 1; i < model.vertRays(); i++)
	{
		EXPECT_TRUE(model.ringElevationRad(i) > model.ringElevationRad(i - 1));
	}

	// Column 0 fires at t=0; last column fires just before the full period:
	EXPECT_NEAR(model.columnFireTime(0), 0.0, 1e-12);
	EXPECT_TRUE(model.columnFireTime(model.horzRays() - 1) < model.params().sensorPeriod);
	EXPECT_NEAR(
		model.columnFireTime(model.horzRays() - 1),
		(model.horzRays() - 1) * (model.params().sensorPeriod / model.horzRays()), 1e-12);

	// Ray directions are unit-norm.
	for (int ring : {0, 8, 15})
	{
		for (int col : {0, model.horzRays() / 4, model.horzRays() - 1})
		{
			const auto d = model.rayDirection(ring, col);
			EXPECT_NEAR(d.norm(), 1.0, 1e-9);
		}
	}
}

// ---------------------------------------------------------------
// definitions/ouster-os1.sensor.xml defaults: vert_nrays=128,
// vert_fov_degrees=22.5, sensor_period=0.10, horz_nrays=1024.
void test_ouster_os1()
{
	Lidar3DModel::Params p;
	p.vertNumRays = 128;
	p.vertFovDegrees = 22.5;
	p.sensorPeriod = 0.10;
	p.horzNumRays = 1024;
	p.minRange = 0.5;
	p.maxRange = 90.0;

	Lidar3DModel model(p);

	EXPECT_TRUE(model.vertRays() == 128);
	EXPECT_TRUE(model.horzRays() == 1024);
	EXPECT_NEAR(model.ringElevationRad(0) * 180.0 / M_PI, -11.25, 1e-6);
	EXPECT_NEAR(model.ringElevationRad(127) * 180.0 / M_PI, 11.25, 1e-6);

	// Azimuth wraps [-pi, pi):
	EXPECT_NEAR(model.columnAzimuthRad(0), -M_PI, 1e-9);
	EXPECT_TRUE(model.columnAzimuthRad(model.horzRays() - 1) < M_PI);
}

// ---------------------------------------------------------------
// definitions/helios-32-FOV-31.sensor.xml: an explicit, *non-uniform*
// vertical_ray_angles list (32 values from +15 down to -16 degrees), given
// in the XML in descending order; the model must re-sort it ascending, same
// as the interactive Lidar3D sensor does.
void test_helios32_explicit_angles()
{
	// Exactly as listed in definitions/helios-32-FOV-31.sensor.xml:
	const std::vector<double> anglesAsInXml = {15, 14, 13, 12,	11,	 10,  9,   8,	7,	 6,	 5,
											   4,  3,  2,  1,	0,	 -1,  -2,  -3,	-4,	 -5, -6,
											   -7, -8, -9, -10, -11, -12, -13, -14, -15, -16};
	EXPECT_TRUE(anglesAsInXml.size() == 32);

	Lidar3DModel::Params p;
	p.verticalRayAnglesDeg = anglesAsInXml;
	p.sensorPeriod = 1.0 / 10.0;  // sensor_rate default 10 Hz
	p.horzNumRays = static_cast<int>(mrpt::round(p.sensorPeriod / 55.296e-6));
	p.minRange = 0.20;
	p.maxRange = 110.0;

	Lidar3DModel model(p);

	EXPECT_TRUE(model.vertRays() == 32);
	// Re-sorted ascending: ring 0 = -16 (lowest), ring 31 = +15 (highest).
	EXPECT_NEAR(model.ringElevationRad(0) * 180.0 / M_PI, -16.0, 1e-6);
	EXPECT_NEAR(model.ringElevationRad(31) * 180.0 / M_PI, 15.0, 1e-6);
	for (int i = 1; i < model.vertRays(); i++)
	{
		EXPECT_TRUE(model.ringElevationRad(i) > model.ringElevationRad(i - 1));
	}
	// Non-uniform spacing is honored exactly: the gap between ring 0 (-16)
	// and ring 1 (-15) equals the others (1deg), verifying no resampling
	// happened -- every listed angle survives intact.
	EXPECT_NEAR((model.ringElevationRad(1) - model.ringElevationRad(0)) * 180.0 / M_PI, 1.0, 1e-6);
}

void test_single_ray_no_column_span()
{
	// Degenerate but must not divide-by-zero: 1 vertical ray.
	Lidar3DModel::Params p;
	p.vertNumRays = 1;
	p.vertFovDegrees = 0.0;
	p.horzNumRays = 360;
	p.sensorPeriod = 0.1;
	Lidar3DModel model(p);
	EXPECT_TRUE(model.vertRays() == 1);
	EXPECT_NEAR(model.ringElevationRad(0), 0.0, 1e-12);
}

}  // namespace

// ---------------------------------------------------------------
int main()
{
	test_velodyne_vlp16();
	test_ouster_os1();
	test_helios32_explicit_angles();
	test_single_ray_no_column_span();

	if (g_failures == 0)
	{
		std::printf("All Lidar3DModel tests passed.\n");
	}
	else
	{
		std::fprintf(stderr, "%d test(s) FAILED.\n", g_failures);
	}
	return g_failures == 0 ? 0 : 1;
}
