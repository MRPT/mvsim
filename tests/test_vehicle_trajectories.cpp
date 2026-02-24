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

void run_single_test(
	const std::string& dyn_class, const std::string& ctrl_class, bool is_ideal,
	const std::string& test_name, const mrpt::math::TTwist2D& twist_cmd,
	const mrpt::math::TPose3D& expected_pose, double tol_override = -1.0)
{
	std::cout << "[TEST] " << test_name << " | " << dyn_class << " | " << ctrl_class << "\n";

	const double simulStep = 0.001;
	const double final_time = 1.0;
	const int steps = static_cast<int>(final_time / simulStep);

	mvsim::World world;
	world.headless(true);
	world.set_gravity(9.81);
	world.set_simul_timestep(simulStep);
	world.internal_initialize();

	std::string wheels_config;

	if (dyn_class == "differential")
	{
		wheels_config = R"(
        <l_wheel pos="0  0.5" mass="1.0" width="0.03" diameter="0.20" />
        <r_wheel pos="0 -0.5" mass="1.0" width="0.03" diameter="0.20" />
		)";
	}
	else if (dyn_class == "differential_4w")
	{
		// 4-wheel differential: front/rear pairs at small x offsets.
		// Lateral friction at non-zero x creates anti-yaw moment during turns,
		// so curves will have larger tracking error than 2-wheel differential.
		wheels_config = R"(
        <l_wheel  pos=" 0.3  0.5" mass="1.0" width="0.03" diameter="0.20" />
        <r_wheel  pos=" 0.3 -0.5" mass="1.0" width="0.03" diameter="0.20" />
        <lr_wheel pos="-0.3  0.5" mass="1.0" width="0.03" diameter="0.20" />
        <rr_wheel pos="-0.3 -0.5" mass="1.0" width="0.03" diameter="0.20" />
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

	// Controller PID parameters depend on vehicle type:
	// - Differential (small wheels R=0.1, I_yy=0.005): KP must be low enough
	//   that KP*max_error < max_friction_torque (~3.73 Nm) to avoid wheel slip.
	//   max_torque set high enough to not clip the differential between wheels.
	// - Ackermann (large wheels R=0.31, I_yy=0.289): KP can be high (no slip),
	//   max_torque capped below friction limit for fast ramp-up then precise tracking.
	std::string pid_config;
	// Both "differential" and "differential_4w" use the same PID params
	const std::string actual_dyn_class =
		(dyn_class == "differential_4w") ? std::string("differential") : dyn_class;

	if (actual_dyn_class == "differential")
		pid_config = "<KP>4</KP><KI>3</KI><KD>0</KD><max_torque>20</max_torque>";
	else
		pid_config = "<KP>1000</KP><KI>0</KI><KD>0</KD><max_torque>300</max_torque>";

	std::string xml =
		"<vehicle name=\"test_veh\">"
		"  <dynamics class=\"" +
		actual_dyn_class +
		"\">"
		"    <chassis linear_damping=\"0\" angular_damping=\"0\" />" +
		wheels_config + "    <controller class=\"" + ctrl_class +
		"\">"
		"      " +
		pid_config +
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
		std::printf(
			"  Vehicle COM (from getChassisCenterOfMass): (%.4f, %.4f)\n",
			veh->getChassisCenterOfMass().x, veh->getChassisCenterOfMass().y);
	}

	// Setup Friction / CSVLogger verification
	double max_friction_y = 0.0;
	double simul_time = 0.0;

	for (auto& logger : veh->getLoggers())
	{
		if (!logger) continue;
		logger->setRecording(true);
		logger->setFileWritingEnabled(false);  // callbacks only, no .csv files
		logger->registerOnRowCallback(
			[&max_friction_y, &simul_time](const std::map<std::string_view, double>& cols)
			{
				if (cols.count(mvsim::VehicleBase::WL_FRIC_Y))
				{
					max_friction_y =
						std::max(max_friction_y, std::abs(cols.at(mvsim::VehicleBase::WL_FRIC_Y)));
				}
			});
	}

	// Run simulation
	veh->getControllerInterface()->setTwistCommand(twist_cmd);

	for (int i = 0; i < steps; ++i)
	{
		simul_time = i * simulStep;
		world.run_simulation(simulStep);
	}

	auto pose = veh->getPose();
	std::cout << "Final pose   : " << pose << "\n";
	std::cout << "Expected pose: " << expected_pose << "\n";

	double tol = (tol_override > 0) ? tol_override : (is_ideal ? 0.04 : 0.25);
	EXPECT_NEAR(pose.x, expected_pose.x, tol);
	EXPECT_NEAR(pose.y, expected_pose.y, tol);
	EXPECT_NEAR(pose.yaw, expected_pose.yaw, tol);

	// Friction verification (only for non-ideal curve tests)
	if (!is_ideal && twist_cmd.omega != 0.0)
	{
		EXPECT_GT(max_friction_y, 1.0);
	}
}

}  // namespace

int main()
{
	const auto straight_cmd = mrpt::math::TTwist2D(1.0, 0.0, 0.0);
	const auto curve_cmd = mrpt::math::TTwist2D(1.0, 0.0, 1.0);
	const auto gt_straight = mrpt::math::TPose3D(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	const auto gt_curve = mrpt::math::TPose3D(0.841, 0.459, 0.0, 1.0, 0.0, 0.0);

	// Differential tests (2-wheel: no lateral slip during turns)
	run_single_test("differential", "twist_ideal", true, "Straight", straight_cmd, gt_straight);
	run_single_test("differential", "twist_ideal", true, "Curve", curve_cmd, gt_curve);
	run_single_test("differential", "twist_pid", false, "Straight", straight_cmd, gt_straight);
	run_single_test("differential", "twist_pid", false, "Curve", curve_cmd, gt_curve);

	// 4-wheel differential tests:
	// Lateral friction at front/rear wheels (x!=0) creates anti-yaw torque during
	// turns, so curve tracking is less precise than 2-wheel differential.
	run_single_test("differential_4w", "twist_ideal", true, "Straight", straight_cmd, gt_straight);
	run_single_test("differential_4w", "twist_ideal", true, "Curve", curve_cmd, gt_curve);
	run_single_test("differential_4w", "twist_pid", false, "Straight", straight_cmd, gt_straight);
	run_single_test("differential_4w", "twist_pid", false, "Curve", curve_cmd, gt_curve, 0.35);

	// Ackermann tests
	run_single_test("ackermann", "twist_ideal", true, "Straight", straight_cmd, gt_straight);
	run_single_test("ackermann", "twist_ideal", true, "Curve", curve_cmd, gt_curve);
	run_single_test(
		"ackermann", "twist_front_steer_pid", false, "Straight", straight_cmd, gt_straight);
	run_single_test("ackermann", "twist_front_steer_pid", false, "Curve", curve_cmd, gt_curve);

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
