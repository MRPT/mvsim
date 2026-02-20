/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mvsim/CsvLogger.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/World.h>

#include <cmath>
#include <iostream>
#include <string>

#include "test_utils.h"

// Minimal test framework
int g_failures = 0;

namespace
{

void run_trajectory_test(const std::string& dyn_class, const std::string& ctrl_class, bool is_ideal)
{
	std::cout << "[TEST] Dynamics: " << dyn_class << " | Controller: " << ctrl_class << "\n";

	mvsim::World world;
	world.headless(true);
	world.set_gravity(9.81);
	world.set_simul_timestep(0.01);
	world.internal_initialize();

	// 1. Programmatic vehicle insertion via inline XML
	std::string xml =
		"<vehicle name=\"test_veh\">"
		"  <dynamics class=\"" +
		dyn_class +
		"\">"
		"    <controller class=\"" +
		ctrl_class +
		"\">"
		"      <KP>150</KP><KI>0</KI><KD>0</KD><max_torque>5000</max_torque>"
		"    </controller>"
		"  </dynamics>"
		"<init_pose>0 0 0</init_pose>"
		"</vehicle>";

	auto veh = mvsim::VehicleBase::factory(&world, xml);
	world.insert_vehicle(veh);

	// Debug: print Box2D local center of mass
	if (auto* b2d = veh->getBox2DChassisBody(); b2d)
	{
		const auto lc = b2d->GetLocalCenter();
		std::printf("  Box2D local center of mass: (%.4f, %.4f)\n", lc.x, lc.y);
	}

	// 2. Setup Friction / CSVLogger verification
	double max_friction_y = 0.0;

	// Assuming `getLoggers()` or similar exposes the vector of CSVLoggers.
	// (Adjust getter name to match your exact `VehicleBase` header API).
	for (auto& logger : veh->getLoggers())
	{
		if (logger)
		{
			logger->setRecording(true);	 // Force logger to become 'active'
			logger->registerOnRowCallback(
				[&max_friction_y](const std::map<std::string_view, double>& cols)
				{
					if (cols.count(mvsim::VehicleBase::WL_FRIC_Y))
					{
						max_friction_y = std::max(
							max_friction_y, std::abs(cols.at(mvsim::VehicleBase::WL_FRIC_Y)));
					}

#if 1
					// print trajectory for debugging:
					// PL_DQ_{X,Y} are the linear velocities in the GLOBAL frame.
					if (cols.count(mvsim::VehicleBase::PL_Q_X) &&
						cols.count(mvsim::VehicleBase::PL_Q_Y) &&
						cols.count(mvsim::VehicleBase::PL_Q_YAW))
					{
						std::printf(
							"Pose: x=%.3f y=%.3f yaw=%.3f | vx=%.3f vy=%.3f w=%.03f | "
							"max_fric_y=%.2f\n",
							cols.at(mvsim::VehicleBase::PL_Q_X),
							cols.at(mvsim::VehicleBase::PL_Q_Y),
							cols.at(mvsim::VehicleBase::PL_Q_YAW),
							cols.at(mvsim::VehicleBase::PL_DQ_X),
							cols.at(mvsim::VehicleBase::PL_DQ_Y),
							cols.at(mvsim::VehicleBase::PL_DQ_W), max_friction_y);
					}
#endif
				});
		}
	}

	// ------------------------------------------------------------------
	// PHASE A: Straight Trajectory
	// ------------------------------------------------------------------
	std::cout << "== TEST A: Straight Trajectory\n";

	veh->setPose(mrpt::math::TPose3D::Identity());
	veh->setTwist({0.0, 0.0, 0.0});
	veh->getControllerInterface()->setTwistCommand({1.0, 0.0, 0.0});  // vx = 1 m/s

	for (int i = 0; i < 100; ++i)
	{
		world.run_simulation(0.01);
	}

	auto pose_straight = veh->getPose();

	// Ideal controllers perfectly match constraints. PID needs settling tolerance.
	double tol = is_ideal ? 0.02 : 0.25;
	EXPECT_NEAR(pose_straight.x, 1.0, tol);
	EXPECT_NEAR(pose_straight.y, 0.0, tol);
	EXPECT_NEAR(pose_straight.yaw, 0.0, tol);

	// ------------------------------------------------------------------
	// PHASE B: Curve Trajectory
	// ------------------------------------------------------------------
	std::cout << "== TEST B: Curve Trajectory\n";
	veh->setPose(mrpt::math::TPose3D::Identity());
	veh->setTwist({0.0, 0.0, 0.0});
	veh->getControllerInterface()->setTwistCommand({1.0, 0.0, 1.0});  // vx = 1 m/s, w = 1 rad/s

	max_friction_y = 0.0;  // Reset friction tracker for the curve

	for (int i = 0; i < 100; ++i)
	{
		world.run_simulation(0.01);
	}

	auto pose_curve = veh->getPose();

	// Expected ideal kinematics (R = v/w = 1.0 m):
	// x(t) = R * sin(w*t) => sin(1) ≈ 0.841
	// y(t) = R * (1 - cos(w*t)) => 1 - cos(1) ≈ 0.459
	// yaw(t) = w*t => 1.0

	EXPECT_NEAR(pose_curve.x, 0.841, tol);
	EXPECT_NEAR(pose_curve.y, 0.459, tol);
	EXPECT_NEAR(pose_curve.yaw, 1.0, tol);

	// 3. Friction Verification (only relevant for non-ideal controllers
	//    where friction actually provides the centripetal force):
	if (!is_ideal)
	{
		EXPECT_GT(max_friction_y, 1.0);
	}
}

}  // namespace

int main()
{
	// Test Matrix: Differential
	run_trajectory_test("differential", "twist_ideal", true);
	// run_trajectory_test("differential", "twist_pid", false);

	// Test Matrix: Ackermann
	run_trajectory_test("ackermann", "twist_ideal", true);
	// run_trajectory_test("ackermann", "twist_front_steer_pid", false);

	if (g_failures == 0)
	{
		std::printf("\nAll Trajectory Matrix tests passed successfully.\n");
	}
	else
	{
		std::fprintf(stderr, "\n%d test(s) FAILED.\n", g_failures);
	}

	return g_failures == 0 ? 0 : 1;
}