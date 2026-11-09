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
#include <mrpt/math/wrap2pi.h>
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
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

#include "test_utils.h"

int g_failures = 0;

namespace
{
const std::string kWorld = std::string(MVSIM_TUTORIAL_DIR) + "/demo_dataset_gen.world.xml";
const std::string kTraj = std::string(MVSIM_TUTORIAL_DIR) + "/demo_dataset_gen.trajectory.tum";
const std::string kWorld2D = std::string(MVSIM_TUTORIAL_DIR) + "/demo_dataset_gen_2d.world.xml";
const std::string kTraj2D = std::string(MVSIM_TUTORIAL_DIR) + "/demo_dataset_gen_2d.trajectory.txt";

constexpr double kSweepPeriod = 0.1;  // VLP16 default (600 rpm)

// Known demo world geometry and motion (demo_dataset_gen.{world.xml,trajectory.tum}):
// the vehicle moves along +X at 1 m/s from x=0 at kTrajT0, with no rotation,
// and the lidar is mounted at z=0.75 m. Pillars are vertical cylinders.
constexpr double kTrajT0 = 1700000000.0;
constexpr double kLidarZ = 0.75;
constexpr double kPillarRadius = 0.4;
const std::vector<std::pair<double, double>> kPillarCenters = {{5, -3}, {6, 1}, {5, 3}};

struct SweepStats
{
	int nObs = 0;
	size_t totalPoints = 0;
	double minRange = std::numeric_limits<double>::max();
	double maxRange = 0;
	float tMin = std::numeric_limits<float>::max();
	float tMax = -1;
	/** Max over points of |azimuth - expected column azimuth from `t`| [rad],
	 * assuming a full sweep spanning `kSweepPeriod`. */
	double maxAzimuthVsTimeErr = 0;
	/** Points on pillars, moved to the world frame with the vehicle pose at
	 * each point's own firing time (sweep start + t), and the max deviation
	 * of their distance to the pillar axis from the pillar radius [m]. */
	size_t nPillarPoints = 0;
	double maxPillarRadiusErr = 0;

	int nImu = 0;
	double maxAbsAngVel = 0;  //!< Max over samples of |w| [rad/s]
	/** Max over IMU samples of |acc - (0,0,g)| [m/s^2]. */
	double maxAccErrVsStatic = 0;
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

		if (auto obsPts = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(obj))
		{
			st.nObs++;
			const double t = mrpt::Clock::toDouble(obsPts->timestamp);
			if (t < lastT - 1e-9)
			{
				st.chronological = false;
			}
			lastT = t;

			auto pts = std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(obsPts->pointcloud);
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

				// Each column starts at azimuth -pi and sweeps 2*pi in one period:
				const double azExpected = -M_PI + 2 * M_PI * tv / kSweepPeriod;
				const double azErr = std::abs(mrpt::math::wrapToPi(std::atan2(y, x) - azExpected));
				st.maxAzimuthVsTimeErr = std::max(st.maxAzimuthVsTimeErr, azErr);

				const double vehX = (t + tv) - kTrajT0;
				const double wx = x + vehX;
				const double wy = y;
				const double wz = z + kLidarZ;
				if (wz < 0.05)
				{
					continue;  // ground
				}
				for (const auto& [cx, cy] : kPillarCenters)
				{
					const double d = std::hypot(wx - cx, wy - cy);
					if (d < kPillarRadius + 0.2)
					{
						st.nPillarPoints++;
						st.maxPillarRadiusErr =
							std::max(st.maxPillarRadiusErr, std::abs(d - kPillarRadius));
					}
				}
			}
		}
		else if (auto obsImu = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(obj))
		{
			st.nImu++;
			const double t = mrpt::Clock::toDouble(obsImu->timestamp);
			if (t < lastT - 1e-9)
			{
				st.chronological = false;
			}
			lastT = t;

			const double wx = obsImu->get(mrpt::obs::IMU_WX);
			const double wy = obsImu->get(mrpt::obs::IMU_WY);
			const double wz = obsImu->get(mrpt::obs::IMU_WZ);
			const double wNorm = std::sqrt(wx * wx + wy * wy + wz * wz);
			st.maxAbsAngVel = std::max(st.maxAbsAngVel, wNorm);
			st.lastAccZ = obsImu->get(mrpt::obs::IMU_Z_ACC);
			const double ax = obsImu->get(mrpt::obs::IMU_X_ACC);
			const double ay = obsImu->get(mrpt::obs::IMU_Y_ACC);
			const double az = obsImu->get(mrpt::obs::IMU_Z_ACC) - 9.81;
			st.maxAccErrVsStatic =
				std::max(st.maxAccErrVsStatic, std::sqrt(ax * ax + ay * ay + az * az));
		}
		else if (auto obsOdom = std::dynamic_pointer_cast<mrpt::obs::CObservationOdometry>(obj))
		{
			st.nOdom++;
			const double t = mrpt::Clock::toDouble(obsOdom->timestamp);
			if (t < lastT - 1e-9)
			{
				st.chronological = false;
			}
			lastT = t;
			st.lastOdom = obsOdom->odometry;
		}
	}
	return st;
}

int runDatasetGen(
	const std::string& world, const std::string& traj, const std::string& outFile,
	const std::string& extraArgs)
{
	std::ostringstream cmd;
	cmd << "\"" << MVSIM_DATASET_GEN_EXE_PATH << "\" \"" << world << "\" --trajectory \"" << traj
		<< "\" -o \"" << outFile << "\" " << extraArgs << " > /dev/null 2>&1";
	return std::system(cmd.str().c_str());
}

int runDatasetGen(const std::string& outFile, const std::string& extraArgs)
{
	return runDatasetGen(kWorld, kTraj, outFile, extraArgs);
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
		std::istreambuf_iterator<char>(fb), std::istreambuf_iterator<char>());
}

std::string tempPath(const std::string& suffix)
{
	return mrpt::system::getTempFileName() + "_" + suffix;
}

/** Mirrors mrpt::system::fileNameChangeExtension(), which mvsim-dataset-gen
 * uses to derive its .gt.tum companion paths: it *replaces* the rawlog's
 * extension, it does not append. */
std::string gtPath(const std::string& rawlogPath, const std::string& newExt)
{
	return mrpt::system::fileNameChangeExtension(rawlogPath, newExt);
}

// ---------------------------------------------------------------
void test_global_shutter_basic()
{
	const std::string out = tempPath("global.rawlog");
	const int rc = runDatasetGen(out, "--duration 0.5 --seed 1 --noiseless --shutter global");
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
	// Pillar hits lie on their surfaces when using the sweep-start pose:
	EXPECT_TRUE(st.nPillarPoints > 100);
	EXPECT_LT(st.maxPillarRadiusErr, 1e-3);

	// The demo world's straight, constant-velocity, level trajectory: the
	// IMU must read ~zero angular velocity and ~gravity on Z (regression
	// test for a real bug: a naive central difference at the trajectory's
	// own time boundary previously produced a huge spurious acceleration
	// spike on the very first sample).
	EXPECT_TRUE(st.nImu > 0);
	EXPECT_LT(st.maxAbsAngVel, 1e-6);
	EXPECT_NEAR(st.lastAccZ, 9.81, 0.05);
	// Every sample, not only the last one: the stencil times must be exact
	// even with epoch timestamps, or the jitter reads as acceleration.
	EXPECT_LT(st.maxAccErrVsStatic, 1e-3);

	// Odometry: noiseless dead reckoning must match the ground truth
	// x-displacement (straight line along +X) closely.
	EXPECT_TRUE(st.nOdom > 0);
	EXPECT_NEAR(st.lastOdom.x(), 0.5 /*duration*/ * 1.0 /*m/s*/, 0.05);

	// All 3 sensor streams must be interleaved in one global time order.
	EXPECT_TRUE(st.chronological);

	std::remove(out.c_str());
	std::remove(gtPath(out, "gt.tum").c_str());
	std::remove(gtPath(out, "lidar1.gt.tum").c_str());
}

// ---------------------------------------------------------------
void test_rolling_shutter_skew()
{
	const std::string out = tempPath("rolling.rawlog");
	// Rolling shutter is the default mode:
	const int rc = runDatasetGen(out, "--duration 0.5 --seed 1 --noiseless");
	EXPECT_TRUE(rc == 0);

	const auto st = loadAndInspect(out);
	EXPECT_TRUE(st.nObs > 0);
	// Rolling shutter: per-column firing times span most of the sweep period
	// (VLP16 default: 0.1s), strictly increasing from 0.
	EXPECT_NEAR(st.tMin, 0.0f, 1e-6);
	EXPECT_GT(st.tMax, 0.09);
	EXPECT_LT(st.tMax, kSweepPeriod);
	// Points are given in the sensor frame at their own column firing time
	// (not de-skewed), so each point azimuth matches its `t` exactly, even
	// while the vehicle moves.
	EXPECT_LT(st.maxAzimuthVsTimeErr, 1e-4);
	// Each column must be cast from the vehicle pose at its own firing time:
	// moved to the world frame with that pose, pillar hits lie exactly on
	// their surfaces (the sweep-start pose would be off by several cm).
	EXPECT_TRUE(st.nPillarPoints > 100);
	EXPECT_LT(st.maxPillarRadiusErr, 1e-3);

	std::remove(out.c_str());
	std::remove(gtPath(out, "gt.tum").c_str());
	std::remove(gtPath(out, "lidar1.gt.tum").c_str());
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
	std::remove(gtPath(outA, "gt.tum").c_str());
	std::remove(gtPath(outB, "gt.tum").c_str());
	std::remove(gtPath(outA, "lidar1.gt.tum").c_str());
	std::remove(gtPath(outB, "lidar1.gt.tum").c_str());
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
	std::remove(gtPath(outIdeal, "gt.tum").c_str());
	std::remove(gtPath(outNoisy, "gt.tum").c_str());
	std::remove(gtPath(outIdeal, "lidar1.gt.tum").c_str());
	std::remove(gtPath(outNoisy, "lidar1.gt.tum").c_str());
}

// ---------------------------------------------------------------
// The sensor definition reads the error model from XML variables, which
// fall back to environment variables (inherited by the generator process).
void setEnv(const char* name, const char* value)
{
#ifdef _WIN32
	_putenv_s(name, value);
#else
	setenv(name, value, 1);
#endif
}

void removeOutputs(const std::string& rawlog)
{
	std::remove(rawlog.c_str());
	std::remove(gtPath(rawlog, "gt.tum").c_str());
	std::remove(gtPath(rawlog, "lidar1.gt.tum").c_str());
}

void test_realistic_error_model()
{
	const char* vars[] = {"sensor_range_std_noise_per_meter", "sensor_beam_divergence_deg",
						  "sensor_mixed_pixel_probability",	  "sensor_beam_elevation_std_deg",
						  "sensor_beam_azimuth_std_deg",	  "sensor_beam_range_bias_std",
						  "sensor_max_incidence_deg",		  "sensor_calibration_seed"};
	const std::string outBase = tempPath("errbase.rawlog");
	const std::string outErr1 = tempPath("err1.rawlog");
	const std::string outErr2 = tempPath("err2.rawlog");
	const std::string outIdeal = tempPath("errideal.rawlog");
	const std::string outIdealErr = tempPath("erridealerr.rawlog");

	// The baseline runs with the model disabled, whatever the environment:
	for (const char* v : vars)
	{
		setEnv(v, "0");
	}
	EXPECT_TRUE(runDatasetGen(outBase, "--duration 0.5 --seed 5") == 0);
	EXPECT_TRUE(runDatasetGen(outIdeal, "--duration 0.5 --seed 5 --noiseless") == 0);

	setEnv("sensor_range_std_noise_per_meter", "0.0005");
	setEnv("sensor_beam_divergence_deg", "0.2");
	setEnv("sensor_mixed_pixel_probability", "0.5");
	setEnv("sensor_beam_elevation_std_deg", "0.05");
	setEnv("sensor_beam_azimuth_std_deg", "0.05");
	setEnv("sensor_beam_range_bias_std", "0.01");
	setEnv("sensor_max_incidence_deg", "60");
	setEnv("sensor_calibration_seed", "7");

	EXPECT_TRUE(runDatasetGen(outErr1, "--duration 0.5 --seed 5") == 0);
	EXPECT_TRUE(runDatasetGen(outErr2, "--duration 0.5 --seed 5") == 0);
	EXPECT_TRUE(runDatasetGen(outIdealErr, "--duration 0.5 --seed 5 --noiseless") == 0);

	for (const char* v : vars)
	{
		setEnv(v, "0");
	}

	// Enabled: different output, still reproducible from the seeds:
	EXPECT_FALSE(filesEqual(outBase, outErr1));
	EXPECT_TRUE(filesEqual(outErr1, outErr2));
	// --noiseless disables the error model too:
	EXPECT_TRUE(filesEqual(outIdeal, outIdealErr));
	// No returns beyond the maximum incidence angle:
	const SweepStats base = loadAndInspect(outBase);
	const SweepStats err = loadAndInspect(outErr1);
	EXPECT_TRUE(base.nObs == err.nObs);
	EXPECT_LT(err.totalPoints, base.totalPoints);

	for (const auto& f : {outBase, outErr1, outErr2, outIdeal, outIdealErr})
	{
		removeOutputs(f);
	}
}

// ---------------------------------------------------------------
// demo_dataset_gen_2d.world.xml is a linear ramp, z = 0.1*x exactly;
// demo_dataset_gen_2d.trajectory.txt drives straight along +X, so the
// terrain-following ground truth is trivial to check by hand.
void test_2d_trajectory_terrain_following()
{
	const std::string out = tempPath("2d.rawlog");
	const int rc = runDatasetGen(kWorld2D, kTraj2D, out, "--noiseless");
	EXPECT_TRUE(rc == 0);

	std::ifstream gt(gtPath(out, "gt.tum"));
	EXPECT_TRUE(bool(gt));
	std::string line;
	int nLines = 0;
	double maxZErr = 0;
	while (std::getline(gt, line))
	{
		if (line.empty())
		{
			continue;
		}
		std::istringstream ls(line);
		double t, x, y, z, qx, qy, qz, qw;
		if (!(ls >> t >> x >> y >> z >> qx >> qy >> qz >> qw))
		{
			continue;
		}
		nLines++;
		maxZErr = std::max(maxZErr, std::abs(z - 0.1 * x));
		// Flat in Y, pure roll-free pitch: no lateral tilt expected on a
		// ramp with no Y-slope.
		EXPECT_NEAR(qx, 0.0, 1e-6);
	}
	EXPECT_TRUE(nLines > 5);
	EXPECT_LT(maxZErr, 1e-3);

	std::remove(out.c_str());
	std::remove(gtPath(out, "gt.tum").c_str());
	std::remove(gtPath(out, "lidar1.gt.tum").c_str());
}

// ---------------------------------------------------------------
// The vehicle rolls in place about its X axis at a constant rate `w`. The
// demo world's IMU is mounted `r` = 0.3 m above the vehicle origin, so it
// moves on a circle and must read the centripetal acceleration w^2*r towards
// the axis, on top of gravity: f_x = 0 and |(f_y, f_z + w^2*r)| = g.
void test_imu_lever_arm()
{
	constexpr double kW = 1.0;	// rad/s
	constexpr double kImuZ = 0.3;  // demo_dataset_gen.world.xml
	constexpr double kG = 9.81;

	const std::string traj = tempPath("roll.tum");
	{
		std::ofstream f(traj);
		f << std::fixed << std::setprecision(9);
		for (int i = 0; i <= 40; i++)
		{
			const double t = 0.05 * i;
			const double phi = kW * t;
			f << kTrajT0 + t << " 0 0 0 " << std::sin(0.5 * phi) << " 0 0 " << std::cos(0.5 * phi)
			  << "\n";
		}
	}
	const std::string out = tempPath("roll.rawlog");
	const int rc =
		runDatasetGen(kWorld, traj, out, "--duration 1.5 --seed 1 --noiseless --shutter global");
	EXPECT_TRUE(rc == 0);

	mrpt::io::CFileGZInputStream f(out);
	auto arch = mrpt::serialization::archiveFrom(f);
	int nImu = 0;
	double maxErr = 0;
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
		auto obsImu = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(obj);
		if (!obsImu)
		{
			continue;
		}
		nImu++;
		const double fx = obsImu->get(mrpt::obs::IMU_X_ACC);
		const double fy = obsImu->get(mrpt::obs::IMU_Y_ACC);
		const double fz = obsImu->get(mrpt::obs::IMU_Z_ACC) + kW * kW * kImuZ;
		maxErr = std::max(maxErr, std::abs(fx));
		maxErr = std::max(maxErr, std::abs(std::sqrt(fy * fy + fz * fz) - kG));
	}
	EXPECT_TRUE(nImu > 10);
	EXPECT_LT(maxErr, 1e-3);

	std::remove(out.c_str());
	std::remove(traj.c_str());
	std::remove(gtPath(out, "gt.tum").c_str());
	std::remove(gtPath(out, "lidar1.gt.tum").c_str());
}

}  // namespace

// ---------------------------------------------------------------
int main()
{
	test_global_shutter_basic();
	test_rolling_shutter_skew();
	test_determinism_same_seed();
	test_noiseless_differs_from_noisy();
	test_realistic_error_model();
	test_2d_trajectory_terrain_following();
	test_imu_lever_arm();

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
