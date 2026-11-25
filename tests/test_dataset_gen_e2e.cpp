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

#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/topography/conversions.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
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

void unsetEnv(const char* name)
{
#ifdef _WIN32
	_putenv_s(name, "");
#else
	unsetenv(name);
#endif
}

void removeOutputs(const std::string& rawlog)
{
	std::remove(rawlog.c_str());
	std::remove(gtPath(rawlog, "gt.tum").c_str());
	std::remove(gtPath(rawlog, "lidar1.gt.tum").c_str());
	std::remove(gtPath(rawlog, "gt.yaml").c_str());
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

struct Sweep
{
	double t = 0;  //!< Sweep start time [s]
	mrpt::maps::CGenericPointsMap::Ptr pts;
};

/** Every LiDAR sweep of a rawlog, in file order. */
std::vector<Sweep> loadSweeps(const std::string& rawlogPath)
{
	std::vector<Sweep> out;
	mrpt::io::CFileGZInputStream f(rawlogPath);
	auto arch = mrpt::serialization::archiveFrom(f);
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
		if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservationPointCloud>(obj); obs)
		{
			out.push_back(
				{mrpt::Clock::toDouble(obs->timestamp),
				 std::dynamic_pointer_cast<mrpt::maps::CGenericPointsMap>(obs->pointcloud)});
		}
	}
	return out;
}

/** Whether both rawlogs have the same sweeps, point by point (sweep times,
 * positions, and the `t` and `ring` fields). */
bool samePoints(const std::string& a, const std::string& b)
{
	const auto sa = loadSweeps(a);
	const auto sb = loadSweeps(b);
	if (sa.empty() || sa.size() != sb.size())
	{
		return false;
	}
	for (size_t k = 0; k < sa.size(); k++)
	{
		const auto& pa = sa[k].pts;
		const auto& pb = sb[k].pts;
		if (sa[k].t != sb[k].t || !pa || !pb || pa->size() != pb->size())
		{
			return false;
		}
		for (size_t i = 0; i < pa->size(); i++)
		{
			float xa, ya, za, xb, yb, zb;
			pa->getPoint(i, xa, ya, za);
			pb->getPoint(i, xb, yb, zb);
			if (xa != xb || ya != yb || za != zb ||
				pa->getPointField_float(i, "t") != pb->getPointField_float(i, "t") ||
				pa->getPointField_uint16(i, "ring") != pb->getPointField_uint16(i, "ring"))
			{
				return false;
			}
		}
	}
	return true;
}

/** Sets every error-model variable of the stock sensor definitions to 0. */
void resetErrorModelEnv()
{
	for (const char* v :
		 {"sensor_range_std_noise_per_meter", "sensor_beam_divergence_deg",
		  "sensor_max_incidence_deg", "sensor_mixed_pixel_probability", "sensor_echo_separation",
		  "sensor_detection_range", "sensor_beam_elevation_std_deg", "sensor_beam_azimuth_std_deg",
		  "sensor_beam_range_bias_std", "sensor_calibration_seed"})
	{
		setEnv(v, "0");
	}
}

void test_beam_model()
{
	resetErrorModelEnv();
	const std::string outBase = tempPath("beambase.rawlog");
	const std::string outBeam1 = tempPath("beam1.rawlog");
	const std::string outBeam2 = tempPath("beam2.rawlog");
	const std::string outNear = tempPath("beamnear.rawlog");
	const std::string outNearLegacy = tempPath("beamnearlegacy.rawlog");
	const std::string outCutoff = tempPath("beamcutoff.rawlog");
	const std::string outBad = tempPath("beambad.rawlog");
	EXPECT_TRUE(runDatasetGen(outBase, "--duration 0.5 --seed 5") == 0);

	setEnv("sensor_beam_divergence_deg", "0.3");
	setEnv("sensor_echo_separation", "0.5");
	EXPECT_TRUE(runDatasetGen(outBeam1, "--duration 0.5 --seed 5") == 0);
	EXPECT_TRUE(runDatasetGen(outBeam2, "--duration 0.5 --seed 5") == 0);
	// A short detection range drops the far returns, with or without the beam
	// model:
	setEnv("sensor_detection_range", "4.0");
	EXPECT_TRUE(runDatasetGen(outNear, "--duration 0.5 --seed 5") == 0);
	setEnv("sensor_echo_separation", "0");
	EXPECT_TRUE(runDatasetGen(outNearLegacy, "--duration 0.5 --seed 5") == 0);
	setEnv("sensor_detection_range", "0");
	// The incidence cutoff applies to every sub-ray hit:
	setEnv("sensor_echo_separation", "0.5");
	setEnv("sensor_max_incidence_deg", "60");
	EXPECT_TRUE(runDatasetGen(outCutoff, "--duration 0.5 --seed 5") == 0);
	setEnv("sensor_max_incidence_deg", "0");
	// The beam model replaces the mixed returns between neighbor rays, and it
	// needs a beam divergence:
	setEnv("sensor_mixed_pixel_probability", "0.5");
	EXPECT_TRUE(runDatasetGen(outBad, "--duration 0.5 --seed 5") != 0);
	setEnv("sensor_mixed_pixel_probability", "0");
	setEnv("sensor_beam_divergence_deg", "0");
	EXPECT_TRUE(runDatasetGen(outBad, "--duration 0.5 --seed 5") != 0);
	resetErrorModelEnv();

	EXPECT_FALSE(filesEqual(outBase, outBeam1));
	EXPECT_TRUE(filesEqual(outBeam1, outBeam2));
	// Every ray whose axis hits returns, as without the model (up to the
	// returns that the range noise moves out of the valid interval):
	const SweepStats base = loadAndInspect(outBase);
	const SweepStats beam = loadAndInspect(outBeam1);
	const SweepStats nearOnly = loadAndInspect(outNear);
	const SweepStats nearLegacy = loadAndInspect(outNearLegacy);
	const SweepStats cutoff = loadAndInspect(outCutoff);
	EXPECT_TRUE(base.nObs == beam.nObs);
	EXPECT_LT(
		std::abs(static_cast<double>(beam.totalPoints) - static_cast<double>(base.totalPoints)),
		0.01 * static_cast<double>(base.totalPoints));
	EXPECT_LT(nearOnly.totalPoints, beam.totalPoints);
	EXPECT_LT(nearOnly.maxRange, 4.5);
	EXPECT_LT(nearLegacy.totalPoints, base.totalPoints);
	EXPECT_LT(nearLegacy.maxRange, 4.5);
	EXPECT_LT(cutoff.totalPoints, beam.totalPoints);

	for (const auto& f : {outBase, outBeam1, outBeam2, outNear, outNearLegacy, outCutoff, outBad})
	{
		removeOutputs(f);
	}
}

/** Ground points of a beam-model render against the exact ranges: one plane
 * stays one echo however oblique, so its returns keep their footprint spread
 * and their mean range. The demo world labels the ground 1. */
void test_beam_model_oblique_ground()
{
	resetErrorModelEnv();
	setEnv("sensor_std_noise", "0");
	const std::string outExact = tempPath("beamexact.rawlog");
	const std::string outBeam = tempPath("beamground.rawlog");
	EXPECT_TRUE(runDatasetGen(outExact, "--duration 0.5 --seed 3 --labels") == 0);
	// A separation far below the range spread of the footprint on the ground:
	setEnv("sensor_beam_divergence_deg", "0.3");
	setEnv("sensor_echo_separation", "0.05");
	EXPECT_TRUE(runDatasetGen(outBeam, "--duration 0.5 --seed 3 --labels") == 0);
	resetErrorModelEnv();
	setEnv("sensor_std_noise", "0.005");

	const auto exact = loadSweeps(outExact);
	const auto beam = loadSweeps(outBeam);
	EXPECT_TRUE(exact.size() == beam.size() && !exact.empty());
	// Per ring: count, sum and sum of squares of the range error, and the
	// relative errors, over the ground beyond 8 m. Errors over 1 m are returns
	// from another object at the edge of the footprint, counted apart.
	struct RingStats
	{
		size_t n = 0;
		double sum = 0;
		double sumSq = 0;
		double sumRel = 0;
	};
	std::map<int, RingStats> rings;
	size_t nOther = 0;
	size_t nGround = 0;
	for (size_t s = 0; s < std::min(exact.size(), beam.size()); s++)
	{
		const auto& a = exact[s].pts;
		const auto& b = beam[s].pts;
		// The same rays return, in the same order:
		EXPECT_TRUE(a && b && a->size() == b->size());
		if (!a || !b || a->size() != b->size())
		{
			continue;
		}
		for (size_t i = 0; i < a->size(); i++)
		{
			if (a->getPointField_uint16(i, "label") != 1)
			{
				continue;
			}
			float ax = 0;
			float ay = 0;
			float az = 0;
			float bx = 0;
			float by = 0;
			float bz = 0;
			a->getPoint(i, ax, ay, az);
			b->getPoint(i, bx, by, bz);
			const double ra = std::sqrt(ax * ax + ay * ay + az * az);
			if (ra < 8.0)
			{
				continue;
			}
			nGround++;
			const double d = std::sqrt(bx * bx + by * by + bz * bz) - ra;
			if (std::abs(d) > 1.0)
			{
				nOther++;
				continue;
			}
			RingStats& rs = rings[a->getPointField_uint16(i, "ring")];
			rs.n++;
			rs.sum += d;
			rs.sumSq += d * d;
			rs.sumRel += d / ra;
		}
	}
	EXPECT_GT(nGround, 100U);
	EXPECT_LT(nOther, nGround / 50);
	size_t nRings = 0;
	for (const auto& [ring, rs] : rings)
	{
		if (rs.n < 50)
		{
			continue;
		}
		nRings++;
		const double mean = rs.sum / static_cast<double>(rs.n);
		const double stdDev = std::sqrt(std::max(0.0, rs.sumSq / rs.n - mean * mean));
		std::printf(
			"Beam model, ground ring %d: %zu points, range error mean %.3f m, std %.3f m\n", ring,
			rs.n, mean, stdDev);
		// The footprint spread (capped at 0.15 m: std 0.043 m) is still there:
		EXPECT_GT(stdDev, 0.03);
		EXPECT_LT(stdDev, 0.1);
		// No range jump to a part of the footprint, beyond the power weighting:
		EXPECT_LT(std::abs(rs.sumRel / static_cast<double>(rs.n)), 0.03);
	}
	EXPECT_GT(nRings, 1U);

	removeOutputs(outExact);
	removeOutputs(outBeam);
}

/** Mixed echoes at depth edges: labeled as such if asked for, a small share
 * of the points, and labels do not change the samples. */
void test_beam_model_mixed_labels()
{
	resetErrorModelEnv();
	const std::string outMixed = tempPath("beammixed.rawlog");
	const std::string outMixedPlain = tempPath("beammixedplain.rawlog");
	setEnv("sensor_beam_divergence_deg", "0.3");
	setEnv("sensor_echo_separation", "1.0");
	EXPECT_TRUE(runDatasetGen(outMixedPlain, "--duration 0.5 --seed 9") == 0);
	EXPECT_TRUE(
		runDatasetGen(outMixed, "--duration 0.5 --seed 9 --labels --mixed-return-label 99") == 0);
	resetErrorModelEnv();
	EXPECT_TRUE(samePoints(outMixedPlain, outMixed));
	size_t nMixed = 0;
	size_t nPoints = 0;
	for (const auto& sweep : loadSweeps(outMixed))
	{
		const auto& pts = sweep.pts;
		for (size_t i = 0; pts && i < pts->size(); i++)
		{
			nPoints++;
			if (pts->getPointField_uint16(i, "label") == 99)
			{
				nMixed++;
				EXPECT_TRUE(pts->getPointField_uint16(i, "instance") == 0);
			}
		}
	}
	EXPECT_GT(nMixed, 0U);
	EXPECT_LT(nMixed, nPoints / 20);
	removeOutputs(outMixed);
	removeOutputs(outMixedPlain);
}

// The demo world labels the ground 1, the pillars 2 (instances 1-3) and the
// crates 3 (instance 4 for crate1, none for crate2).
void test_semantic_labels()
{
	const std::string outPlain = tempPath("plain.rawlog");
	const std::string outLabels = tempPath("labels.rawlog");
	EXPECT_TRUE(runDatasetGen(outPlain, "--duration 0.5 --seed 9 --noiseless") == 0);
	EXPECT_TRUE(runDatasetGen(outLabels, "--duration 0.5 --seed 9 --noiseless --labels") == 0);

	// Labels do not change the samples:
	EXPECT_TRUE(samePoints(outPlain, outLabels));
	const auto plain = loadSweeps(outPlain);
	EXPECT_TRUE(!plain.empty() && plain.front().pts && !plain.front().pts->hasPointField("label"));

	size_t nPoints = 0;
	size_t nWrong = 0;
	size_t nPillar = 0;
	size_t nCrate = 0;
	for (const auto& sweep : loadSweeps(outLabels))
	{
		const auto& pts = sweep.pts;
		EXPECT_TRUE(pts && pts->hasPointField("label") && pts->hasPointField("instance"));
		if (!pts || !pts->hasPointField("label"))
		{
			continue;
		}
		for (size_t i = 0; i < pts->size(); i++)
		{
			float x, y, z;
			pts->getPoint(i, x, y, z);
			// Each point at the vehicle pose of its own firing time (1 m/s along +X):
			const double vehX = (sweep.t + pts->getPointField_float(i, "t")) - kTrajT0;
			const double wx = x + vehX;
			const double wz = z + kLidarZ;
			const uint16_t label = pts->getPointField_uint16(i, "label");
			const uint16_t instance = pts->getPointField_uint16(i, "instance");
			nPoints++;

			// The object the point lies on: a pillar (by distance to its axis),
			// else the ground (z=0) or a crate. Within 1 mm of the ground, the
			// base of a pillar or crate is accepted too.
			const bool nearGround = wz < 1e-3;
			const uint16_t crateInstance = std::hypot(wx - 2.0, y - 2.0) < 2.5 ? 4 : 0;
			bool ok = nearGround ? (label == 1 && instance == 0)
								 : (label == 3 && instance == crateInstance);
			if (nearGround)
			{
				ok = ok || (label == 3 && instance == crateInstance);
			}
			else
			{
				nCrate++;
			}
			for (size_t p = 0; p < kPillarCenters.size(); p++)
			{
				const auto& [cx, cy] = kPillarCenters[p];
				if (std::hypot(wx - cx, y - cy) < kPillarRadius + 1e-3)
				{
					const bool onPillar = label == 2 && instance == p + 1;
					ok = nearGround ? (ok || onPillar) : onPillar;
					if (!nearGround)
					{
						nPillar++;
						nCrate--;
					}
				}
			}
			nWrong += ok ? 0 : 1;
		}
	}
	EXPECT_GT(nPoints, 1000U);
	EXPECT_GT(nPillar, 100U);
	EXPECT_GT(nCrate, 100U);
	EXPECT_TRUE(nWrong == 0);

	// Mixed returns get their own class if asked for, and the samples are
	// still the same as without labels:
	const std::string outMixed = tempPath("mixed.rawlog");
	const std::string outMixedPlain = tempPath("mixedplain.rawlog");
	setEnv("sensor_mixed_pixel_probability", "1.0");
	EXPECT_TRUE(runDatasetGen(outMixedPlain, "--duration 0.5 --seed 9") == 0);
	EXPECT_TRUE(
		runDatasetGen(outMixed, "--duration 0.5 --seed 9 --labels --mixed-return-label 99") == 0);
	setEnv("sensor_mixed_pixel_probability", "0");
	EXPECT_TRUE(samePoints(outMixedPlain, outMixed));
	size_t nMixed = 0;
	for (const auto& sweep : loadSweeps(outMixed))
	{
		const auto& pts = sweep.pts;
		for (size_t i = 0; pts && i < pts->size(); i++)
		{
			if (pts->getPointField_uint16(i, "label") == 99)
			{
				nMixed++;
				EXPECT_TRUE(pts->getPointField_uint16(i, "instance") == 0);
			}
		}
	}
	EXPECT_GT(nMixed, 0U);

	// --mixed-return-label requires --labels:
	EXPECT_TRUE(runDatasetGen(outMixed, "--duration 0.5 --mixed-return-label 99") != 0);

	for (const auto& f : {outPlain, outLabels, outMixed, outMixedPlain})
	{
		removeOutputs(f);
	}
}

// ---------------------------------------------------------------
// Moving blocks (--moving-block): the demo world blocks "crate1" (a box of
// 1.5 x 1.0 x 1.2 m, with its frame at a bottom corner) and "pillar1" (a
// cylinder of radius 0.4 m, with its frame at the center of its base) follow
// trajectories written by each test.

/** Writes a TUM trajectory with one knot every 10 ms for t in [tFirst, tLast]
 * (relative to kTrajT0), and returns its path. */
std::string writeTum(
	const std::string& suffix, const std::function<mrpt::poses::CPose3D(double)>& poseAt,
	double tFirst, double tLast)
{
	const std::string path = tempPath(suffix);
	std::ofstream f(path);
	f << std::fixed << std::setprecision(9);
	const int n = static_cast<int>(std::round((tLast - tFirst) / 0.01));
	for (int i = 0; i <= n; i++)
	{
		const double t = tFirst + 0.01 * i;
		const mrpt::poses::CPose3D p = poseAt(t);
		mrpt::math::CQuaternionDouble q;
		p.getAsQuaternion(q);
		f << kTrajT0 + t << " " << p.x() << " " << p.y() << " " << p.z() << " " << q.x() << " "
		  << q.y() << " " << q.z() << " " << q.r() << "\n";
	}
	return path;
}

/** The command-line option that makes `block` follow the trajectory file. */
std::string movingBlockArg(const std::string& block, const std::string& trajFile)
{
	return " --moving-block \"" + block + "=" + trajFile + "\"";
}

struct RigidBoxStats
{
	size_t nFace = 0;  //!< Points on the box face towards the sensor
	double minFaceX = std::numeric_limits<double>::max();
	double maxFaceX = std::numeric_limits<double>::lowest();
	size_t nCylinder = 0;  //!< Points on the cylinder, above the box
	double maxCylinderErr = 0;
};

/** For test_moving_block_rigid_with_vehicle(): the box seen from the sensor
 * (sensor frame) in the region where it is if it moves with the vehicle. */
RigidBoxStats inspectRigidBox(const std::string& rawlogPath)
{
	RigidBoxStats st;
	for (const auto& sweep : loadSweeps(rawlogPath))
	{
		for (size_t i = 0; sweep.pts && i < sweep.pts->size(); i++)
		{
			float x = 0;
			float y = 0;
			float z = 0;
			sweep.pts->getPoint(i, x, y, z);
			if (std::abs(y) < 0.6 && z > -0.6 && z < 0.4 && x > 1.5 && x < 4.5)
			{
				st.nFace++;
				st.minFaceX = std::min<double>(st.minFaceX, x);
				st.maxFaceX = std::max<double>(st.maxFaceX, x);
			}
			const double d = std::hypot(x - 4.0, y + 0.75);
			if (z > 0.5 && d < 0.6)
			{
				st.nCylinder++;
				st.maxCylinderErr = std::max(st.maxCylinderErr, std::abs(d - kPillarRadius));
			}
		}
	}
	return st;
}

// A box and a cylinder sharing one trajectory, the one of the vehicle (which
// moves forward while turning and rolling) plus a constant offset, are seen
// at a constant place of the sensor frame; a box parked where the former
// starts is not.
void test_moving_block_rigid_with_vehicle()
{
	const auto vehiclePose = [](double t)
	{ return mrpt::poses::CPose3D(1.0 * t, 0, 0, 0.2 * t /*yaw*/, 0, 0.1 * t /*roll*/); };
	// In the sensor frame, the box then spans x=[3,4], y=[-0.75,0.75],
	// z=[-0.75,0.45], and the cylinder axis is at (4,-0.75):
	const mrpt::poses::CPose3D offset(4.0, -0.75, 0, mrpt::DEG2RAD(90.0), 0, 0);
	const auto blockPose = [&](double t) { return vehiclePose(t) + offset; };

	const std::string trajVehicle = writeTum("mbveh.tum", vehiclePose, 0, 1.2);
	const std::string trajRigid = writeTum("mbrigid.tum", blockPose, 0, 1.2);
	// Ends before the run starts: the block stays at the last pose.
	const std::string trajParked = writeTum(
		"mbparked.tum", [&](double) { return blockPose(0); }, -2.0, -1.0);

	const std::string outRigid = tempPath("mbrigid.rawlog");
	const std::string outParked = tempPath("mbparked.rawlog");
	const std::string outIncidence = tempPath("mbincidence.rawlog");
	const std::string args = "--duration 1.0 --seed 1";
	const std::string rigidBlocks =
		movingBlockArg("crate1", trajRigid) + movingBlockArg("pillar1", trajRigid);
	EXPECT_TRUE(
		runDatasetGen(kWorld, trajVehicle, outRigid, args + " --noiseless" + rigidBlocks) == 0);
	EXPECT_TRUE(
		runDatasetGen(
			kWorld, trajVehicle, outParked,
			args + " --noiseless" + movingBlockArg("crate1", trajParked)) == 0);
	// The error model needs the surface normals: with a wrong one, the face
	// (seen head-on) would be dropped as a grazing hit.
	setEnv("sensor_max_incidence_deg", "60");
	EXPECT_TRUE(runDatasetGen(kWorld, trajVehicle, outIncidence, args + rigidBlocks) == 0);
	setEnv("sensor_max_incidence_deg", "0");

	const RigidBoxStats rigid = inspectRigidBox(outRigid);
	EXPECT_GT(rigid.nFace, 1000U);
	EXPECT_NEAR(rigid.minFaceX, 3.0, 1e-3);
	EXPECT_NEAR(rigid.maxFaceX, 3.0, 1e-3);
	EXPECT_GT(rigid.nCylinder, 300U);
	EXPECT_LT(rigid.maxCylinderErr, 1e-3);

	const RigidBoxStats parked = inspectRigidBox(outParked);
	EXPECT_GT(parked.nFace, 1000U);
	EXPECT_GT(parked.maxFaceX - parked.minFaceX, 0.5);

	const RigidBoxStats incidence = inspectRigidBox(outIncidence);
	EXPECT_GT(incidence.nFace, rigid.nFace * 9 / 10);
	EXPECT_GT(incidence.minFaceX, 2.9);
	EXPECT_LT(incidence.maxFaceX, 3.1);

	for (const auto& f : {outRigid, outParked, outIncidence})
	{
		removeOutputs(f);
	}
	for (const auto& f : {trajVehicle, trajRigid, trajParked})
	{
		std::remove(f.c_str());
	}
}

// The vehicle moves at 1 m/s along +X, and a box ahead of it at 5 m/s: its
// face towards the sensor is at 4 + 4*t [m] in the sensor frame, t being the
// firing time of each point (rolling shutter) or the sweep start (global).
void test_moving_block_shutter_timing()
{
	const std::string trajBlock = writeTum(
		"mbfast.tum",
		[](double t) { return mrpt::poses::CPose3D(4.0 + 5.0 * t, -0.5, 0, 0, 0, 0); }, 0, 1.0);
	const std::string outRolling = tempPath("mbrolling.rawlog");
	const std::string outGlobal = tempPath("mbglobal.rawlog");
	const std::string args =
		"--duration 0.5 --seed 1 --noiseless" + movingBlockArg("crate1", trajBlock);
	EXPECT_TRUE(runDatasetGen(outRolling, args) == 0);
	EXPECT_TRUE(runDatasetGen(outGlobal, args + " --shutter global") == 0);

	for (const bool rolling : {true, false})
	{
		size_t nFace = 0;
		double maxErr = 0;
		// What the pose of the box at the sweep start would give:
		double minErrSweepStart = std::numeric_limits<double>::max();
		float minT = std::numeric_limits<float>::max();
		float maxT = 0;
		for (const auto& sweep : loadSweeps(rolling ? outRolling : outGlobal))
		{
			for (size_t i = 0; sweep.pts && i < sweep.pts->size(); i++)
			{
				float x = 0;
				float y = 0;
				float z = 0;
				sweep.pts->getPoint(i, x, y, z);
				if (std::abs(y) > 0.45 || z < -0.6 || x < 3.5 || x > 7.0)
				{
					continue;
				}
				const float t = sweep.pts->getPointField_float(i, "t");
				const double tSweep = sweep.t - kTrajT0;
				nFace++;
				maxErr = std::max(maxErr, std::abs(x - (4.0 + 4.0 * (tSweep + t))));
				minErrSweepStart =
					std::min(minErrSweepStart, std::abs(x - (4.0 + 4.0 * tSweep - 1.0 * t)));
				minT = std::min(minT, t);
				maxT = std::max(maxT, t);
			}
		}
		EXPECT_GT(nFace, 500U);
		EXPECT_LT(maxErr, 1e-3);
		if (rolling)
		{
			// The face is seen about half a sweep after the sweep start, and
			// skewed: its columns fire at different times.
			EXPECT_GT(minErrSweepStart, 0.2);
			EXPECT_GT(4.0 * (maxT - minT), 0.01);
		}
		else
		{
			EXPECT_TRUE(maxT == 0);
		}
	}

	removeOutputs(outRolling);
	removeOutputs(outGlobal);
	std::remove(trajBlock.c_str());
}

// Blocks following a trajectory that keeps them at their <init_pose> give
// the same points as the static blocks, up to rounding.
void test_moving_block_at_init_pose()
{
	const std::string trajCrate = writeTum(
		"mbcrate.tum",
		[](double) { return mrpt::poses::CPose3D(2, 2, 0, mrpt::DEG2RAD(30.0), 0, 0); }, 0, 0.1);
	const std::string trajPillar = writeTum(
		"mbpillar.tum", [](double) { return mrpt::poses::CPose3D(5, -3, 0, 0, 0, 0); }, 0, 0.1);
	const std::string outStatic = tempPath("mbstatic.rawlog");
	const std::string outMoving = tempPath("mbsame.rawlog");
	EXPECT_TRUE(runDatasetGen(outStatic, "--duration 0.5 --seed 1 --noiseless") == 0);
	EXPECT_TRUE(
		runDatasetGen(
			outMoving, "--duration 0.5 --seed 1 --noiseless" + movingBlockArg("crate1", trajCrate) +
						   movingBlockArg("pillar1", trajPillar)) == 0);

	const auto sa = loadSweeps(outStatic);
	const auto sb = loadSweeps(outMoving);
	EXPECT_TRUE(!sa.empty() && sa.size() == sb.size());
	double maxDiff = 0;
	for (size_t k = 0; k < sa.size() && k < sb.size(); k++)
	{
		const auto& pa = sa[k].pts;
		const auto& pb = sb[k].pts;
		EXPECT_TRUE(pa && pb && pa->size() == pb->size());
		for (size_t i = 0; pa && pb && i < pa->size() && i < pb->size(); i++)
		{
			float xa = 0;
			float ya = 0;
			float za = 0;
			float xb = 0;
			float yb = 0;
			float zb = 0;
			pa->getPoint(i, xa, ya, za);
			pb->getPoint(i, xb, yb, zb);
			maxDiff = std::max<double>(
				maxDiff, std::abs(xa - xb) + std::abs(ya - yb) + std::abs(za - zb));
		}
	}
	EXPECT_LT(maxDiff, 1e-4);

	removeOutputs(outStatic);
	removeOutputs(outMoving);
	std::remove(trajCrate.c_str());
	std::remove(trajPillar.c_str());
}

void test_moving_block_errors()
{
	const std::string out = tempPath("mberr.rawlog");
	const std::string traj = writeTum(
		"mbok.tum", [](double t) { return mrpt::poses::CPose3D(4.0 + t, 2.0, 0, 0, 0, 0); }, 0,
		0.5);
	const std::string emptyTraj = tempPath("mbempty.tum");
	{
		std::ofstream f(emptyTraj);
	}
	const std::string args = "--duration 0.2";
	// The option itself works:
	EXPECT_TRUE(runDatasetGen(out, args + movingBlockArg("crate1", traj)) == 0);
	// Unknown block, block given twice, missing and empty trajectory files:
	EXPECT_TRUE(runDatasetGen(out, args + movingBlockArg("nosuchblock", traj)) != 0);
	EXPECT_TRUE(
		runDatasetGen(
			out, args + movingBlockArg("crate1", traj) + movingBlockArg("crate1", traj)) != 0);
	EXPECT_TRUE(runDatasetGen(out, args + movingBlockArg("crate1", traj + ".missing")) != 0);
	EXPECT_TRUE(runDatasetGen(out, args + movingBlockArg("crate1", emptyTraj)) != 0);
	// Malformed arguments:
	EXPECT_TRUE(runDatasetGen(out, args + " --moving-block crate1") != 0);
	EXPECT_TRUE(runDatasetGen(out, args + movingBlockArg("crate1", "")) != 0);
	EXPECT_TRUE(runDatasetGen(out, args + movingBlockArg("", traj)) != 0);

	removeOutputs(out);
	std::remove(traj.c_str());
	std::remove(emptyTraj.c_str());
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
		// The observation carries the mounting pose on the vehicle:
		EXPECT_LT(std::abs(obsImu->sensorPose.z() - kImuZ), 1e-9);
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

/** The sensor ground truth YAML file of a rawlog. */
mrpt::containers::yaml loadGroundTruth(const std::string& rawlog)
{
	return mrpt::containers::yaml::FromFile(gtPath(rawlog, "gt.yaml"));
}

/** Every observation of a rawlog, in file order. */
std::vector<mrpt::obs::CObservation::Ptr> loadObservations(const std::string& rawlogPath)
{
	std::vector<mrpt::obs::CObservation::Ptr> out;
	mrpt::io::CFileGZInputStream f(rawlogPath);
	auto arch = mrpt::serialization::archiveFrom(f);
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
		if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj); obs)
		{
			out.push_back(obs);
		}
	}
	return out;
}

void test_ground_truth_beam_errors()
{
	// Per-beam errors only, no range noise, so that ground returns follow
	// the ground truth exactly:
	resetErrorModelEnv();
	setEnv("sensor_std_noise", "0");
	setEnv("sensor_beam_elevation_std_deg", "0.5");
	setEnv("sensor_beam_azimuth_std_deg", "0.2");
	setEnv("sensor_beam_range_bias_std", "0.05");
	setEnv("sensor_calibration_seed", "3");
	const std::string out = tempPath("beamgt.rawlog");
	EXPECT_TRUE(runDatasetGen(out, "--duration 0.3 --seed 1 --shutter global") == 0);
	resetErrorModelEnv();
	unsetEnv("sensor_std_noise");

	const auto gt = loadGroundTruth(out);
	EXPECT_TRUE(gt["format"].as<std::string>() == "mvsim/dataset_gt/1");
	const auto& lidar = gt["sensors"]["lidar1"];
	EXPECT_TRUE(lidar["type"].as<std::string>() == "lidar3d");
	EXPECT_NEAR(lidar["pose_in_vehicle"]["xyz"](2).as<double>(), kLidarZ, 1e-12);
	const auto& intr = lidar["intrinsics"];
	const int nRings = intr["rings"].as<int>();
	EXPECT_TRUE(nRings == 16);
	EXPECT_NEAR(intr["beam_error_std"]["elevation_deg"].as<double>(), 0.5, 1e-12);
	EXPECT_TRUE(intr["calibration_seed"].as<int>() == 3);

	std::vector<double> elev;
	std::vector<double> dElev;
	std::vector<double> dRange;
	double maxAbsDAz = 0;
	for (int r = 0; r < nRings; r++)
	{
		elev.push_back(intr["elevations_deg"](r).as<double>());
		const auto& row = intr["beam_errors"](r);
		EXPECT_TRUE(row["ring"].as<int>() == r);
		dElev.push_back(row["d_elevation_deg"].as<double>());
		dRange.push_back(row["d_range_m"].as<double>());
		maxAbsDAz = std::max(maxAbsDAz, std::abs(row["d_azimuth_deg"].as<double>()));
	}
	EXPECT_GT(maxAbsDAz, 0.0);

	// Ground returns: the ray samples the ground at the perturbed elevation
	// and reports that range plus the bias, along the nominal direction:
	size_t nGround = 0;
	size_t nMatchGt = 0;
	size_t nMatchNominal = 0;
	for (const auto& sw : loadSweeps(out))
	{
		for (size_t i = 0; i < sw.pts->size(); i++)
		{
			const auto ring = sw.pts->getPointField_uint16(i, "ring");
			if (elev.at(ring) > -5.0)
			{
				continue;
			}
			float x;
			float y;
			float z;
			sw.pts->getPoint(i, x, y, z);
			const double range = std::sqrt(double(x) * x + double(y) * y + double(z) * z);
			const double eTrue = mrpt::DEG2RAD(elev[ring] + dElev[ring]);
			const double expected = kLidarZ / std::sin(-eTrue) + dRange[ring];
			const double nominal = kLidarZ / std::sin(-mrpt::DEG2RAD(elev[ring]));
			nGround++;
			nMatchGt += std::abs(range - expected) < 1e-4 ? 1 : 0;
			nMatchNominal += std::abs(range - nominal) < 1e-4 ? 1 : 0;
		}
	}
	EXPECT_GT(nGround, 1000);
	EXPECT_GT(nMatchGt, 0.8 * nGround);
	EXPECT_LT(nMatchNominal, 0.05 * nGround);

	removeOutputs(out);
}

void test_ground_truth_imu_errors()
{
	const std::vector<std::pair<const char*, const char*>> vars = {
		{"sensor_angular_velocity_white_noise_std_noise", "0"},
		{"sensor_linear_acceleration_white_noise_std_noise", "0"},
		{"sensor_angular_velocity_bias", "0.01 -0.02 0.03"},
		{"sensor_linear_acceleration_bias", "0.1 0.2 -0.3"},
		{"sensor_angular_velocity_scale", "0.9"},
		{"sensor_linear_acceleration_scale", "1.0 1.0 1.05"},
		{"sensor_linear_acceleration_misalignment", "0 0.01 0 -0.02 0 0"}};
	for (const auto& [name, value] : vars)
	{
		setEnv(name, value);
	}
	const std::string out = tempPath("imugt.rawlog");
	const std::string outIdeal = tempPath("imugtideal.rawlog");
	EXPECT_TRUE(runDatasetGen(out, "--duration 0.5 --seed 1 --shutter global") == 0);
	EXPECT_TRUE(runDatasetGen(outIdeal, "--duration 0.5 --seed 1 --noiseless") == 0);
	for (const auto& [name, value] : vars)
	{
		unsetEnv(name);
	}

	// The demo trajectory has constant velocity: the true readings are
	// w = 0 and f = (0, 0, g).
	constexpr double kG = 9.81;
	const mrpt::math::TVector3D fExpected(0.01 * kG + 0.1, -0.02 * kG + 0.2, 1.05 * kG - 0.3);
	int nImu = 0;
	double maxErr = 0;
	for (const auto& obs : loadObservations(out))
	{
		auto imu = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(obs);
		if (!imu)
		{
			continue;
		}
		nImu++;
		maxErr = std::max(maxErr, std::abs(imu->get(mrpt::obs::IMU_WX) - 0.01));
		maxErr = std::max(maxErr, std::abs(imu->get(mrpt::obs::IMU_WY) + 0.02));
		maxErr = std::max(maxErr, std::abs(imu->get(mrpt::obs::IMU_WZ) - 0.03));
		maxErr = std::max(maxErr, std::abs(imu->get(mrpt::obs::IMU_X_ACC) - fExpected.x));
		maxErr = std::max(maxErr, std::abs(imu->get(mrpt::obs::IMU_Y_ACC) - fExpected.y));
		maxErr = std::max(maxErr, std::abs(imu->get(mrpt::obs::IMU_Z_ACC) - fExpected.z));
	}
	EXPECT_GT(nImu, 5);
	EXPECT_LT(maxErr, 1e-6);

	const auto gt = loadGroundTruth(out);
	const auto& acc = gt["sensors"]["imu1"]["intrinsics"]["accelerometer"];
	EXPECT_NEAR(acc["scale"](2).as<double>(), 1.05, 1e-12);
	EXPECT_NEAR(acc["bias"](1).as<double>(), 0.2, 1e-12);
	EXPECT_NEAR(acc["misalignment"](0)(2).as<double>(), 0.01, 1e-12);
	EXPECT_NEAR(acc["sensitivity_matrix"](1)(2).as<double>(), -0.02, 1e-12);
	EXPECT_NEAR(acc["sensitivity_matrix"](2)(2).as<double>(), 1.05, 1e-12);
	// Without a bias random walk, no bias time series is written:
	EXPECT_FALSE(mrpt::system::fileExists(gtPath(out, "imu1.bias.txt")));

	// --noiseless removes the deterministic errors too:
	const auto gtIdeal = loadGroundTruth(outIdeal);
	const auto& accIdeal = gtIdeal["sensors"]["imu1"]["intrinsics"]["accelerometer"];
	EXPECT_NEAR(accIdeal["scale"](2).as<double>(), 1.0, 1e-12);
	EXPECT_NEAR(accIdeal["bias"](1).as<double>(), 0.0, 1e-12);

	for (const auto& f : {out, outIdeal})
	{
		removeOutputs(f);
	}
}

void test_clock_offsets()
{
	const std::string outA = tempPath("clockA.rawlog");
	const std::string outB = tempPath("clockB.rawlog");
	EXPECT_TRUE(runDatasetGen(outA, "--duration 0.5 --seed 2") == 0);
	EXPECT_TRUE(
		runDatasetGen(
			outB,
			"--duration 0.5 --seed 2 --clock-offset lidar1=-0.002 --clock-offset imu1=0.004 "
			"--clock-offset odom=0.01") == 0);

	// Same data, stamps shifted per sensor, and still in stamp order:
	const auto obsA = loadObservations(outA);
	const auto obsB = loadObservations(outB);
	std::map<std::string, std::vector<mrpt::obs::CObservation::Ptr>> byLabelA;
	std::map<std::string, std::vector<mrpt::obs::CObservation::Ptr>> byLabelB;
	for (const auto& o : obsA)
	{
		byLabelA[o->sensorLabel].push_back(o);
	}
	double lastT = -1e300;
	bool chronological = true;
	for (const auto& o : obsB)
	{
		byLabelB[o->sensorLabel].push_back(o);
		const double t = mrpt::Clock::toDouble(o->timestamp);
		chronological = chronological && t >= lastT;
		lastT = t;
	}
	EXPECT_TRUE(chronological);
	const std::map<std::string, double> offsets = {
		{"lidar1", -0.002}, {"imu1", 0.004}, {"odom", 0.01}};
	for (const auto& [label, offset] : offsets)
	{
		const auto& a = byLabelA[label];
		const auto& b = byLabelB[label];
		EXPECT_TRUE(!a.empty() && a.size() == b.size());
		for (size_t i = 0; i < std::min(a.size(), b.size()); i++)
		{
			const double dt =
				1e-7 * static_cast<double>((b[i]->timestamp - a[i]->timestamp).count());
			EXPECT_NEAR(dt, offset, 1e-7);
		}
	}
	const auto& imuA = byLabelA["imu1"];
	const auto& imuB = byLabelB["imu1"];
	if (!imuA.empty() && imuA.size() == imuB.size())
	{
		auto a = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(imuA.back());
		auto b = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(imuB.back());
		EXPECT_TRUE(a && b && a->get(mrpt::obs::IMU_Z_ACC) == b->get(mrpt::obs::IMU_Z_ACC));
	}
	EXPECT_NEAR(
		loadGroundTruth(outB)["sensors"]["imu1"]["clock_offset_s"].as<double>(), 0.004, 1e-15);

	// Errors: unknown stream, malformed value:
	const std::string outC = tempPath("clockC.rawlog");
	EXPECT_FALSE(runDatasetGen(outC, "--duration 0.2 --clock-offset nope=0.1") == 0);
	EXPECT_FALSE(runDatasetGen(outC, "--duration 0.2 --clock-offset imu1=abc") == 0);

	for (const auto& f : {outA, outB, outC})
	{
		removeOutputs(f);
	}
}

void test_gnss_generator()
{
	// Exact readings (zero noise), an RTK float episode (with a position jump
	// inside), a false fix and an outage. Event times are relative to the
	// trajectory start, and away from the 0.1 s sample times.
	const std::string world = tempPath("gnss.world.xml");
	{
		std::ofstream f(world);
		f << R"(<mvsim_world version="1.0">
  <georeference> <latitude>36.0</latitude> <longitude>-2.0</longitude> <height>100.0</height> </georeference>
  <element class="horizontal_plane">
    <x_min>-20</x_min> <y_min>-20</y_min> <x_max>20</x_max> <y_max>20</y_max> <z>0</z>
  </element>
  <vehicle name="r1">
    <dynamics class="differential">
      <l_wheel pos="0  0.3" mass="1.0" width="0.05" diameter="0.20" />
      <r_wheel pos="0 -0.3" mass="1.0" width="0.05" diameter="0.20" />
      <chassis mass="10.0" zmin="0.05" zmax="0.25" />
    </dynamics>
    <init_pose>0 0 0</init_pose>
    <sensor class="lidar3d" name="lidar1">
      <pose_3d>0 0 0.75 0 0 0</pose_3d>
      <vert_nrays>8</vert_nrays> <vert_fov_degrees>30</vert_fov_degrees> <horz_nrays>360</horz_nrays>
      <sensor_period>0.1</sensor_period> <range_std_noise>0</range_std_noise>
    </sensor>
    <sensor class="gnss" name="gps1">
      <pose_3d>0.2 0.1 1.5 0 0 0</pose_3d>
      <sensor_period>0.1</sensor_period>
      <horizontal_std_noise>0</horizontal_std_noise> <vertical_std_noise>0</vertical_std_noise>
      <fix_type>rtk_fixed</fix_type>
      <event start="0.25" end="0.55" fix_type="rtk_float" horizontal_std_noise="0.3" vertical_std_noise="0.5" />
      <event start="0.35" end="0.45" offset="0 0 1.0" />
      <event start="0.55" end="0.75" offset="2.0 -1.0 0.5" />
      <event start="0.75" end="0.95" outage="true" />
    </sensor>
  </vehicle>
</mvsim_world>
)";
	}
	const std::string out = tempPath("gnss.rawlog");
	const std::string outIdeal = tempPath("gnssideal.rawlog");
	EXPECT_TRUE(runDatasetGen(world, kTraj, out, "--duration 1.2 --seed 3") == 0);
	EXPECT_TRUE(runDatasetGen(world, kTraj, outIdeal, "--duration 1.2 --seed 3 --noiseless") == 0);

	const mrpt::topography::TGeodeticCoords ref(36.0, -2.0, 100.0);
	const auto check = [&](const std::string& rawlog, bool falseFix)
	{
		int nGnss = 0;
		int nFloat = 0;
		int nFalse = 0;
		for (const auto& obs : loadObservations(rawlog))
		{
			auto gps = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(obs);
			if (!gps)
			{
				continue;
			}
			nGnss++;
			EXPECT_NEAR(gps->sensorPose.z(), 1.5, 1e-12);
			const double tRel = mrpt::Clock::toDouble(gps->timestamp) - kTrajT0;
			const auto& gga = gps->getMsgByClass<mrpt::obs::gnss::Message_NMEA_GGA>().fields;
			mrpt::math::TPoint3D enu;
			mrpt::topography::geodeticToENU_WGS84(
				mrpt::topography::TGeodeticCoords(
					gga.latitude_degrees, gga.longitude_degrees, gga.altitude_meters),
				enu, ref);
			// The vehicle moves along +X at 1 m/s from the origin:
			const mrpt::math::TPoint3D antenna(tRel + 0.2, 0.1, 1.5);
			if (tRel > 0.25 && tRel < 0.55)
			{
				nFloat++;
				EXPECT_TRUE(gps->fix_type == mrpt::obs::GnssFixType::RTK_FLOAT);
				EXPECT_TRUE(gga.fix_quality == 5);
				EXPECT_NEAR((*gps->covariance_enu)(0, 0), 0.09, 1e-12);
				EXPECT_NEAR((*gps->covariance_enu)(2, 2), 0.25, 1e-12);
				continue;
			}
			EXPECT_TRUE(gps->fix_type == mrpt::obs::GnssFixType::RTK_FIXED);
			EXPECT_TRUE(gga.fix_quality == 4);
			EXPECT_NEAR((*gps->covariance_enu)(0, 0), 0.0, 1e-12);
			mrpt::math::TPoint3D err = enu - antenna;
			if (tRel > 0.55 && tRel < 0.75 && falseFix)
			{
				nFalse++;
				err = err - mrpt::math::TPoint3D(2.0, -1.0, 0.5);
			}
			EXPECT_LT(err.norm(), 1e-3);
		}
		// 13 samples in [0, 1.2] s, minus 2 in the outage:
		EXPECT_TRUE(nGnss == 11);
		EXPECT_TRUE(nFloat == 3);
		EXPECT_TRUE(nFalse == (falseFix ? 2 : 0));
	};
	check(out, true);
	check(outIdeal, false);

	const auto gt = loadGroundTruth(out);
	const auto& gnss = gt["sensors"]["gps1"];
	EXPECT_TRUE(gnss["type"].as<std::string>() == "gnss");
	EXPECT_NEAR(gnss["pose_in_vehicle"]["xyz"](0).as<double>(), 0.2, 1e-12);
	EXPECT_NEAR(gnss["rate_hz"].as<double>(), 10.0, 1e-9);
	// The values in effect over each interval, overlaps included:
	const auto& episodes = gnss["intrinsics"]["episodes"];
	EXPECT_TRUE(episodes.size() == 5);
	const std::vector<std::string> qualities = {
		"rtk_float", "rtk_float", "rtk_float", "rtk_fixed", "outage"};
	for (size_t i = 0; i < std::min<size_t>(episodes.size(), qualities.size()); i++)
	{
		EXPECT_TRUE(episodes(i)["quality"].as<std::string>() == qualities[i]);
	}
	EXPECT_NEAR(episodes(1)["t_start"].as<double>(), kTrajT0 + 0.35, 1e-6);
	EXPECT_NEAR(episodes(1)["horizontal_std"].as<double>(), 0.3, 1e-12);
	EXPECT_NEAR(episodes(1)["injected_error_m"](2).as<double>(), 1.0, 1e-12);
	EXPECT_NEAR(episodes(2)["injected_error_m"](2).as<double>(), 0.0, 1e-12);
	EXPECT_NEAR(episodes(3)["t_start"].as<double>(), kTrajT0 + 0.55, 1e-6);
	EXPECT_NEAR(episodes(3)["horizontal_std"].as<double>(), 0.0, 1e-12);
	EXPECT_NEAR(episodes(3)["injected_error_m"](0).as<double>(), 2.0, 1e-12);

	for (const auto& f : {out, outIdeal})
	{
		removeOutputs(f);
	}
	std::remove(world.c_str());
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
	test_beam_model();
	test_beam_model_oblique_ground();
	test_beam_model_mixed_labels();
	test_semantic_labels();
	test_moving_block_rigid_with_vehicle();
	test_moving_block_shutter_timing();
	test_moving_block_at_init_pose();
	test_moving_block_errors();
	test_2d_trajectory_terrain_following();
	test_imu_lever_arm();
	test_ground_truth_beam_errors();
	test_ground_truth_imu_errors();
	test_clock_offsets();
	test_gnss_generator();

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
