/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// End-to-end test for mvsim-dataset-gen: actually runs the built binary
// against mvsim_tutorial/demo_dataset_gen.{world.xml,trajectory.tum} and
// inspects the resulting .rawlog, exercising the whole pipeline (World
// loading, SceneBuilder, TrajectorySource, ray casting, rawlog I/O).

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>

#include "test_utils.h"

int g_failures = 0;

namespace
{
const std::string kWorld = std::string(MVSIM_TUTORIAL_DIR) + "/demo_dataset_gen.world.xml";
const std::string kTraj = std::string(MVSIM_TUTORIAL_DIR) + "/demo_dataset_gen.trajectory.tum";

struct SweepStats
{
	int nObs = 0;
	size_t totalPoints = 0;
	double minRange = std::numeric_limits<double>::max();
	double maxRange = 0;
	float tMin = std::numeric_limits<float>::max();
	float tMax = -1;

	int nImu = 0;
	double maxAbsAngVel = 0;  //!< Max over samples of |w| [rad/s]
	double lastAccZ = 0;  //!< Z-accel of the last IMU sample seen

	int nOdom = 0;
	mrpt::poses::CPose2D lastOdom;

	bool chronological = true;
};

SweepStats loadAndInspect(const std::string& rawlogPath)
{
	SweepStats st;
	mrpt::io::CFileGZInputStream f(rawlogPath);
	auto arch = mrpt::serialization::archiveFrom(f);

	double lastT = -std::numeric_limits<double>::max();

	for (;;)
	{
		mrpt::serialization::CSerializable::Ptr obj;
		try
		{
			arch >> obj;
		}
		catch (const std::exception&)
		{
			break;	// EOF
		}
		if (!obj)
		{
			break;
		}

		if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(obj))
		{
			st.nObs++;
			const double t = mrpt::Clock::toDouble(obs->timestamp);
			if (t < lastT - 1e-9)
			{
				st.chronological = false;
			}
			lastT = t;

			auto pts = std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(obs->pointcloud);
			if (!pts)
			{
				continue;
			}
			const size_t n = pts->size();
			st.totalPoints += n;
			for (size_t i = 0; i < n; i++)
			{
				float x, y, z;
				pts->getPoint(i, x, y, z);
				const double r = std::sqrt(double(x) * x + double(y) * y + double(z) * z);
				st.minRange = std::min(st.minRange, r);
				st.maxRange = std::max(st.maxRange, r);
				const float tv = pts->getPointField_float(i, "t");
				st.tMin = std::min(st.tMin, tv);
				st.tMax = std::max(st.tMax, tv);
			}
		}
		else if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(obj))
		{
			st.nImu++;
			const double t = mrpt::Clock::toDouble(obs->timestamp);
			if (t < lastT - 1e-9)
			{
				st.chronological = false;
			}
			lastT = t;

			const double wx = obs->get(mrpt::obs::IMU_WX);
			const double wy = obs->get(mrpt::obs::IMU_WY);
			const double wz = obs->get(mrpt::obs::IMU_WZ);
			const double wNorm = std::sqrt(wx * wx + wy * wy + wz * wz);
			st.maxAbsAngVel = std::max(st.maxAbsAngVel, wNorm);
			st.lastAccZ = obs->get(mrpt::obs::IMU_Z_ACC);
		}
		else if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(obj))
		{
			st.nOdom++;
			const double t = mrpt::Clock::toDouble(obs->timestamp);
			if (t < lastT - 1e-9)
			{
				st.chronological = false;
			}
			lastT = t;
			st.lastOdom = obs->odometry;
		}
	}
	return st;
}

int runDatasetGen(const std::string& outFile, const std::string& extraArgs)
{
	std::ostringstream cmd;
	cmd << "\"" << MVSIM_DATASET_GEN_EXE_PATH << "\" \"" << kWorld << "\" --trajectory \"" << kTraj
		<< "\" -o \"" << outFile << "\" " << extraArgs << " > /dev/null 2>&1";
	return std::system(cmd.str().c_str());
}

bool filesEqual(const std::string& a, const std::string& b)
{
	std::ifstream fa(a, std::ios::binary);
	std::ifstream fb(b, std::ios::binary);
	if (!fa || !fb)
	{
		return false;
	}
	return std::equal(
		std::istreambuf_iterator<char>(fa), std::istreambuf_iterator<char>(),
		std::istreambuf_iterator<char>(fb));
}

std::string tempPath(const std::string& suffix)
{
	return mrpt::system::getTempFileName() + "_" + suffix;
}

// ---------------------------------------------------------------
void test_global_shutter_basic()
{
	const std::string out = tempPath("global.rawlog");
	const int rc = runDatasetGen(out, "--duration 0.5 --seed 1 --noiseless");
	EXPECT_TRUE(rc == 0);

	const auto st = loadAndInspect(out);
	EXPECT_TRUE(st.nObs > 0);
	EXPECT_TRUE(st.totalPoints > 1000);
	// Global shutter: every point's "t" field must be exactly 0.
	EXPECT_NEAR(st.tMin, 0.0f, 1e-9);
	EXPECT_NEAR(st.tMax, 0.0f, 1e-9);
	// Sanity range bounds (VLP16 default min/max range: 0.01 / 80.0 m):
	EXPECT_GT(st.minRange, 0.005);
	EXPECT_LT(st.maxRange, 80.0);

	// The demo world's straight, constant-velocity, level trajectory: the
	// IMU must read ~zero angular velocity and ~gravity on Z (regression
	// test for a real bug: a naive central difference at the trajectory's
	// own time boundary previously produced a huge spurious acceleration
	// spike on the very first sample).
	EXPECT_TRUE(st.nImu > 0);
	EXPECT_LT(st.maxAbsAngVel, 1e-6);
	EXPECT_NEAR(st.lastAccZ, 9.81, 0.05);

	// Odometry: noiseless dead reckoning must match the ground truth
	// x-displacement (straight line along +X) closely.
	EXPECT_TRUE(st.nOdom > 0);
	EXPECT_NEAR(st.lastOdom.x(), 0.5 /*duration*/ * 1.0 /*m/s*/, 0.05);

	// All 3 sensor streams must be interleaved in one global time order.
	EXPECT_TRUE(st.chronological);

	std::remove(out.c_str());
	std::remove((out + ".gt.tum").c_str());
	std::remove((out + ".lidar1.gt.tum").c_str());
}

// ---------------------------------------------------------------
void test_rolling_shutter_skew()
{
	const std::string out = tempPath("rolling.rawlog");
	const int rc = runDatasetGen(out, "--duration 0.5 --seed 1 --noiseless --shutter rolling");
	EXPECT_TRUE(rc == 0);

	const auto st = loadAndInspect(out);
	EXPECT_TRUE(st.nObs > 0);
	// Rolling shutter: per-column firing times span most of the sweep period
	// (VLP16 default: 0.1s), strictly increasing from 0.
	EXPECT_NEAR(st.tMin, 0.0f, 1e-6);
	EXPECT_GT(st.tMax, 0.09);
	EXPECT_LT(st.tMax, 0.1);

	std::remove(out.c_str());
	std::remove((out + ".gt.tum").c_str());
	std::remove((out + ".lidar1.gt.tum").c_str());
}

// ---------------------------------------------------------------
void test_determinism_same_seed()
{
	const std::string outA = tempPath("det_a.rawlog");
	const std::string outB = tempPath("det_b.rawlog");

	EXPECT_TRUE(runDatasetGen(outA, "--duration 0.5 --seed 7") == 0);
	EXPECT_TRUE(runDatasetGen(outB, "--duration 0.5 --seed 7") == 0);
	EXPECT_TRUE(filesEqual(outA, outB));

	std::remove(outA.c_str());
	std::remove(outB.c_str());
	std::remove((outA + ".gt.tum").c_str());
	std::remove((outB + ".gt.tum").c_str());
	std::remove((outA + ".lidar1.gt.tum").c_str());
	std::remove((outB + ".lidar1.gt.tum").c_str());
}

// ---------------------------------------------------------------
void test_noiseless_differs_from_noisy()
{
	const std::string outIdeal = tempPath("ideal.rawlog");
	const std::string outNoisy = tempPath("noisy.rawlog");

	EXPECT_TRUE(runDatasetGen(outIdeal, "--duration 0.5 --seed 3 --noiseless") == 0);
	EXPECT_TRUE(runDatasetGen(outNoisy, "--duration 0.5 --seed 3") == 0);
	EXPECT_FALSE(filesEqual(outIdeal, outNoisy));

	std::remove(outIdeal.c_str());
	std::remove(outNoisy.c_str());
	std::remove((outIdeal + ".gt.tum").c_str());
	std::remove((outNoisy + ".gt.tum").c_str());
	std::remove((outIdeal + ".lidar1.gt.tum").c_str());
	std::remove((outNoisy + ".lidar1.gt.tum").c_str());
}

}  // namespace

// ---------------------------------------------------------------
int main()
{
	test_global_shutter_basic();
	test_rolling_shutter_skew();
	test_determinism_same_seed();
	test_noiseless_differs_from_noisy();

	if (g_failures == 0)
	{
		std::printf("All mvsim-dataset-gen e2e tests passed.\n");
	}
	else
	{
		std::fprintf(stderr, "%d test(s) FAILED.\n", g_failures);
	}
	return g_failures == 0 ? 0 : 1;
}
