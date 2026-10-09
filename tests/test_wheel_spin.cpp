/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Wheel spin angle integration: the stored angle is wrapped by whole turns,
// while the continuous angle must never jump.

#include <mvsim/Wheel.h>
#include <mvsim/World.h>

#include <iostream>

#include "test_utils.h"

int g_failures = 0;

namespace
{
void test_continuous_spin(double w)
{
	mvsim::World world;
	mvsim::Wheel wheel(&world);
	wheel.setW(w);

	const double dt = 1e-3;
	const size_t nSteps = 2'000'000;  // 2000 s
	double prev = wheel.getPhiContinuous();
	double maxStepJump = 0;
	double maxAbsPhi = 0;
	for (size_t i = 0; i < nSteps; i++)
	{
		wheel.integrateSpin(dt);
		const double cur = wheel.getPhiContinuous();
		maxStepJump = std::max(maxStepJump, std::abs(cur - prev - w * dt));
		maxAbsPhi = std::max(maxAbsPhi, std::abs(wheel.getPhi()));
		prev = cur;
	}

	// Beyond the old wrapping point (1e4 rad), no discontinuities:
	EXPECT_GT(std::abs(w * dt * nSteps), 1e4);
	EXPECT_LT(maxStepJump, 1e-6);
	EXPECT_NEAR(wheel.getPhiContinuous(), w * dt * nSteps, 1e-3);
	// The stored angle stays bounded:
	EXPECT_LT(maxAbsPhi, 1e4 + 1.0);

	// setPhi() resets the continuous angle too:
	wheel.setPhi(0.5);
	EXPECT_NEAR(wheel.getPhiContinuous(), 0.5, 1e-12);
}
}  // namespace

int main()
{
	try
	{
		test_continuous_spin(+10.0);
		test_continuous_spin(-7.3);
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
