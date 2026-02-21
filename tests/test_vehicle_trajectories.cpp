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

	std::string wheels_config;

	if (dyn_class == "differential_4_wheels")
	{
		wheels_config = R"(
        <lf_wheel pos="0.13  0.16" mass="1.0" width="0.03" diameter="0.20" />
        <rf_wheel pos="0.13 -0.16" mass="1.0" width="0.03" diameter="0.20" />
        <lr_wheel pos="-0.13  0.16" mass="1.0" width="0.03" diameter="0.20" />
        <rr_wheel pos="-0.13 -0.16" mass="1.0" width="0.03" diameter="0.20" />
		)";
	}
	else if (dyn_class == "ackermann")
	{
		wheels_config = R"(	
	    <!-- Rear wheels -->
        <rl_wheel pos="0  1" mass="6.0" width="0.30" diameter="0.62" />
        <rr_wheel pos="0 -1" mass="6.0" width="0.30" diameter="0.62" />
        
        <!-- Front wheels -->
        <fl_wheel mass="6.0" width="0.30" diameter="0.62" />
        <fr_wheel mass="6.0" width="0.30" diameter="0.62" />
        <f_wheels_x>1.3</f_wheels_x>
        <f_wheels_d>2.0</f_wheels_d>
        <max_steer_ang_deg>60.0</max_steer_ang_deg>
	)";
	}

	// 1. Programmatic vehicle insertion via inline XML
	std::string xml =
		"<vehicle name=\"test_veh\">"
		"  <dynamics class=\"" +
		dyn_class + "\">" + wheels_config + "    <controller class=\"" + ctrl_class +
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
	{
		veh->getChassisCenterOfMass();
		std::printf(
			"  Vehicle COM (from getChassisCenterOfMass): (%.4f, %.4f)\n",
			veh->getChassisCenterOfMass().x, veh->getChassisCenterOfMass().y);
	}

	// 2. Setup Friction / CSVLogger verification
	double max_friction_y = 0.0;

	// Assuming `getLoggers()` or similar exposes the vector of CSVLoggers.
	// (Adjust getter name to match your exact `VehicleBase` header API).
	double simul_time = 0.0;

	std::cout << "Vehicle loggers count: " << veh->getLoggers().size() << "\n";

	for (auto& logger : veh->getLoggers())
	{
		if (!logger)
		{
			continue;
		}
		logger->setRecording(true);	 // Force logger to become 'active'
		logger->registerOnRowCallback(
			[&max_friction_y, &simul_time](const std::map<std::string_view, double>& cols)
			{
				if (cols.count(mvsim::VehicleBase::WL_FRIC_Y))
				{
					max_friction_y =
						std::max(max_friction_y, std::abs(cols.at(mvsim::VehicleBase::WL_FRIC_Y)));
				}

#if 1
				// print trajectory for debugging:
				// PL_DQ_{X,Y} are the linear velocities in the GLOBAL frame.
				static int print_step = 0;
				if (++print_step % 20 != 0)
				{
					return;
				}
				if (cols.count(mvsim::VehicleBase::PL_Q_X) &&
					cols.count(mvsim::VehicleBase::PL_Q_Y) &&
					cols.count(mvsim::VehicleBase::PL_Q_YAW))
				{
					std::printf(
						"t=%.03lf Pose: x=%.3f y=%.3f yaw=%.3f | vx=%.3f vy=%.3f w=%.03f | "
						"max_fric_y=%.2f\n",
						simul_time, cols.at(mvsim::VehicleBase::PL_Q_X),
						cols.at(mvsim::VehicleBase::PL_Q_Y), cols.at(mvsim::VehicleBase::PL_Q_YAW),
						cols.at(mvsim::VehicleBase::PL_DQ_X), cols.at(mvsim::VehicleBase::PL_DQ_Y),
						cols.at(mvsim::VehicleBase::PL_DQ_W), max_friction_y);
				}
#endif
			});
	}

	// ------------------------------------------------------------------
	// PHASE A: Straight Trajectory
	// ------------------------------------------------------------------
	std::cout << "== TEST A: Straight Trajectory\n";

	veh->setPose(mrpt::math::TPose3D::Identity());
	veh->setRefVelocityLocal({0.0, 0.0, 0.0});
	veh->getControllerInterface()->setTwistCommand({1.0, 0.0, 0.0});  // vx = 1 m/s

	const double simulStep = 0.001;	 // 10 ms step
	const double final_time = 1.0;	// Run for 1 second to reach steady state
	const int steps = static_cast<int>(final_time / simulStep);

	for (int i = 0; i < steps; ++i)
	{
		simul_time = i * simulStep;	 // Update simul_time for logging callback
		world.run_simulation(simulStep);
	}

	auto pose_straight = veh->getPose();
	const auto gt_pose_straight = mrpt::math::TPose3D(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	std::cout << "Final pose   : " << pose_straight << "\n";
	std::cout << "Expected pose: " << gt_pose_straight << "\n";

	// Ideal controllers perfectly match constraints. PID needs settling tolerance.
	double tol = is_ideal ? 0.04 : 0.25;
	EXPECT_NEAR(pose_straight.x, gt_pose_straight.x, tol);
	EXPECT_NEAR(pose_straight.y, gt_pose_straight.y, tol);
	EXPECT_NEAR(pose_straight.yaw, gt_pose_straight.yaw, tol);

	// ------------------------------------------------------------------
	// PHASE B: Curve Trajectory
	// ------------------------------------------------------------------
	std::cout << "== TEST B: Curve Trajectory\n";
	veh->setPose(mrpt::math::TPose3D::Identity());
	veh->setRefVelocityLocal({0.0, 0.0, 0.0});
	veh->getControllerInterface()->setTwistCommand({1.0, 0.0, 1.0});  // vx = 1 m/s, w = 1 rad/s

	max_friction_y = 0.0;  // Reset friction tracker for the curve

	for (int i = 0; i < steps; ++i)
	{
		simul_time = i * simulStep;	 // Update simul_time for logging callback
		world.run_simulation(simulStep);
	}

	const auto gt_pose_curve = mrpt::math::TPose3D(0.841, 0.459, 0.0, 1.0, 0.0, 0.0);
	auto pose_curve = veh->getPose();
	std::cout << "Final pose   : " << pose_curve << "\n";
	std::cout << "Expected pose: " << gt_pose_curve << "\n";

	// Expected ideal kinematics (R = v/w = 1.0 m):
	// x(t) = R * sin(w*t) => sin(1) ≈ 0.841
	// y(t) = R * (1 - cos(w*t)) => 1 - cos(1) ≈ 0.459
	// yaw(t) = w*t => 1.0

	EXPECT_NEAR(pose_curve.x, gt_pose_curve.x, tol);
	EXPECT_NEAR(pose_curve.y, gt_pose_curve.y, tol);
	EXPECT_NEAR(pose_curve.yaw, gt_pose_curve.yaw, tol);

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
	run_trajectory_test("differential_4_wheels", "twist_ideal", true);
	run_trajectory_test("differential_4_wheels", "twist_pid", false);

	// Test Matrix: Ackermann
	run_trajectory_test("ackermann", "twist_ideal", true);
	run_trajectory_test("ackermann", "twist_front_steer_pid", false);

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