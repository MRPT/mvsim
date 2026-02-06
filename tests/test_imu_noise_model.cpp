/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/Sensors/ImuNoiseModel.h>

#include <cmath>
#include <cstdio>

// Minimal test framework (no external deps required):
static int g_failures = 0;

#define EXPECT_NEAR(a, b, tol)                                                      \
	do                                                                              \
	{                                                                               \
		const double va = static_cast<double>(a);                                   \
		const double vb = static_cast<double>(b);                                   \
		if (std::abs(va - vb) > static_cast<double>(tol))                           \
		{                                                                           \
			std::fprintf(                                                           \
				stderr, "FAIL %s:%d  |%f - %f| > %f\n", __FILE__, __LINE__, va, vb, \
				static_cast<double>(tol));                                          \
			++g_failures;                                                           \
		}                                                                           \
	} while (false)

#define EXPECT_GT(a, b)                                                                    \
	do                                                                                     \
	{                                                                                      \
		const double va = static_cast<double>(a);                                          \
		const double vb = static_cast<double>(b);                                          \
		if (!(va > vb))                                                                    \
		{                                                                                  \
			std::fprintf(stderr, "FAIL %s:%d  %f not > %f\n", __FILE__, __LINE__, va, vb); \
			++g_failures;                                                                  \
		}                                                                                  \
	} while (false)

#define EXPECT_LT(a, b)                                                                    \
	do                                                                                     \
	{                                                                                      \
		const double va = static_cast<double>(a);                                          \
		const double vb = static_cast<double>(b);                                          \
		if (!(va < vb))                                                                    \
		{                                                                                  \
			std::fprintf(stderr, "FAIL %s:%d  %f not < %f\n", __FILE__, __LINE__, va, vb); \
			++g_failures;                                                                  \
		}                                                                                  \
	} while (false)

namespace
{

// ---------------------------------------------------------------
// Test 1: With zero noise params, output == input (passthrough)
// ---------------------------------------------------------------
void test_zero_noise()
{
	mvsim::ImuNoiseModel model;
	// default params are 0 => no noise
	const mrpt::math::TVector3D trueGyro{0.1, -0.2, 0.3};
	const mrpt::math::TVector3D trueAccel{0.0, 0.0, 9.81};
	constexpr double dt = 0.01;

	for (int i = 0; i < 100; ++i)
	{
		const auto g = model.applyGyroscope(trueGyro, dt);
		EXPECT_NEAR(g.x, trueGyro.x, 1e-15);
		EXPECT_NEAR(g.y, trueGyro.y, 1e-15);
		EXPECT_NEAR(g.z, trueGyro.z, 1e-15);

		const auto a = model.applyAccelerometer(trueAccel, dt);
		EXPECT_NEAR(a.x, trueAccel.x, 1e-15);
		EXPECT_NEAR(a.y, trueAccel.y, 1e-15);
		EXPECT_NEAR(a.z, trueAccel.z, 1e-15);
	}

	// Bias should remain zero:
	EXPECT_NEAR(model.gyroBias().x, 0.0, 1e-15);
	EXPECT_NEAR(model.accelBias().x, 0.0, 1e-15);
}

// ---------------------------------------------------------------
// Test 2: White noise only — check mean ≈ true, std ≈ sigma
// ---------------------------------------------------------------
void test_white_noise_statistics()
{
	mvsim::ImuNoiseModel model;
	model.gyroscope.white_noise_std = 0.05;
	model.accelerometer.white_noise_std = 0.1;

	const mrpt::math::TVector3D trueGyro{1.0, 0.0, 0.0};
	const mrpt::math::TVector3D trueAccel{0.0, 0.0, 9.81};
	constexpr double dt = 0.01;
	constexpr int N = 50000;

	double sumGx = 0.0;
	double sumGx2 = 0.0;
	double sumAz = 0.0;
	double sumAz2 = 0.0;

	for (int i = 0; i < N; ++i)
	{
		const auto g = model.applyGyroscope(trueGyro, dt);
		const double errGx = g.x - trueGyro.x;
		sumGx += errGx;
		sumGx2 += errGx * errGx;

		const auto a = model.applyAccelerometer(trueAccel, dt);
		const double errAz = a.z - trueAccel.z;
		sumAz += errAz;
		sumAz2 += errAz * errAz;
	}

	const double meanGx = sumGx / static_cast<double>(N);
	const double stdGx = std::sqrt(sumGx2 / static_cast<double>(N) - meanGx * meanGx);

	const double meanAz = sumAz / static_cast<double>(N);
	const double stdAz = std::sqrt(sumAz2 / static_cast<double>(N) - meanAz * meanAz);

	// Mean error should be near zero (allow 4-sigma of the sampling dist)
	EXPECT_NEAR(meanGx, 0.0, 4.0 * 0.05 / std::sqrt(static_cast<double>(N)));
	EXPECT_NEAR(meanAz, 0.0, 4.0 * 0.1 / std::sqrt(static_cast<double>(N)));

	// Std should be close to the configured value (10% tolerance)
	EXPECT_NEAR(stdGx, 0.05, 0.05 * 0.1);
	EXPECT_NEAR(stdAz, 0.1, 0.1 * 0.1);

	// No random walk => bias stays zero
	EXPECT_NEAR(model.gyroBias().x, 0.0, 1e-15);
	EXPECT_NEAR(model.accelBias().z, 0.0, 1e-15);
}

// ---------------------------------------------------------------
// Test 3: Random walk — bias drifts away from zero over time
// ---------------------------------------------------------------
void test_random_walk_drift()
{
	mvsim::ImuNoiseModel model;
	model.gyroscope.random_walk_std = 0.01;	 // rad/s / sqrt(s)
	model.accelerometer.random_walk_std = 0.005;  // m/s² / sqrt(s)

	const mrpt::math::TVector3D zero{0.0, 0.0, 0.0};
	constexpr double dt = 0.001;
	constexpr int steps = 10000;  // 10 s of simulation

	for (int i = 0; i < steps; ++i)
	{
		model.applyGyroscope(zero, dt);
		model.applyAccelerometer(zero, dt);
	}

	// After 10 s, expected bias std = sigma_rw * sqrt(T)
	//   gyro: 0.01 * sqrt(10) ≈ 0.0316 rad/s
	//   accel: 0.005 * sqrt(10) ≈ 0.0158 m/s²
	// The actual realisation can be anywhere, but the magnitude
	// of the bias vector should very likely be > 0 (virtually
	// impossible to stay at exact zero after 10 k steps).
	const auto& gb = model.gyroBias();
	const auto& ab = model.accelBias();
	const double gyroBiasNorm = std::sqrt(gb.x * gb.x + gb.y * gb.y + gb.z * gb.z);
	const double accelBiasNorm = std::sqrt(ab.x * ab.x + ab.y * ab.y + ab.z * ab.z);

	EXPECT_GT(gyroBiasNorm, 1e-6);
	EXPECT_GT(accelBiasNorm, 1e-6);
}

// ---------------------------------------------------------------
// Test 4: Reset clears bias
// ---------------------------------------------------------------
void test_reset()
{
	mvsim::ImuNoiseModel model;
	model.gyroscope.random_walk_std = 0.1;
	model.accelerometer.random_walk_std = 0.1;

	const mrpt::math::TVector3D zero{0.0, 0.0, 0.0};
	for (int i = 0; i < 1000; ++i)
	{
		model.applyGyroscope(zero, 0.01);
		model.applyAccelerometer(zero, 0.01);
	}

	model.reset();

	EXPECT_NEAR(model.gyroBias().x, 0.0, 1e-15);
	EXPECT_NEAR(model.gyroBias().y, 0.0, 1e-15);
	EXPECT_NEAR(model.gyroBias().z, 0.0, 1e-15);
	EXPECT_NEAR(model.accelBias().x, 0.0, 1e-15);
	EXPECT_NEAR(model.accelBias().y, 0.0, 1e-15);
	EXPECT_NEAR(model.accelBias().z, 0.0, 1e-15);
}

// ---------------------------------------------------------------
// Test 5: Combined white noise + random walk produces larger
//          variance than white noise alone
// ---------------------------------------------------------------
void test_combined_noise()
{
	constexpr double dt = 0.01;
	constexpr int N = 10000;
	const mrpt::math::TVector3D trueVal{0.0, 0.0, 0.0};

	// White noise only:
	mvsim::ImuNoiseModel modelW;
	modelW.gyroscope.white_noise_std = 0.05;

	double sumSq_w = 0.0;
	for (int i = 0; i < N; ++i)
	{
		const auto g = modelW.applyGyroscope(trueVal, dt);
		sumSq_w += g.x * g.x;
	}
	const double var_w = sumSq_w / static_cast<double>(N);

	// White noise + random walk:
	mvsim::ImuNoiseModel modelWR;
	modelWR.gyroscope.white_noise_std = 0.05;
	modelWR.gyroscope.random_walk_std = 0.01;

	double sumSq_wr = 0.0;
	for (int i = 0; i < N; ++i)
	{
		const auto g = modelWR.applyGyroscope(trueVal, dt);
		sumSq_wr += g.x * g.x;
	}
	const double var_wr = sumSq_wr / static_cast<double>(N);

	// The combined model should have larger mean-square error
	EXPECT_GT(var_wr, var_w);
}
}  // namespace

// ---------------------------------------------------------------
int main()
{
	test_zero_noise();
	test_white_noise_statistics();
	test_random_walk_drift();
	test_reset();
	test_combined_noise();

	if (g_failures == 0)
	{
		std::printf("All ImuNoiseModel tests passed.\n");
	}
	else
	{
		std::fprintf(stderr, "%d test(s) FAILED.\n", g_failures);
	}
	return g_failures == 0 ? 0 : 1;
}
