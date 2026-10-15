/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Motion tests with full World + Box2D: twist commands executed by ideal and
// PID controllers, stopping, rolling resistance and stability on low friction.

#include <mrpt/math/TTwist2D.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <cmath>
#include <iostream>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{
const char* kSkidSteer = R"(
	<dynamics class="differential_4_wheels">
		<lf_wheel pos=" 0.13  0.16" mass="1.0" width="0.03" diameter="0.20" />
		<rf_wheel pos=" 0.13 -0.16" mass="1.0" width="0.03" diameter="0.20" />
		<lr_wheel pos="-0.13  0.16" mass="1.0" width="0.03" diameter="0.20" />
		<rr_wheel pos="-0.13 -0.16" mass="1.0" width="0.03" diameter="0.20" />
		<chassis mass="10.0" zmin="0.05" zmax="0.25" />
		CONTROLLER
	</dynamics>
)";

const char* kDifferential = R"(
	<dynamics class="differential">
		<l_wheel pos="0.0  0.5" mass="4.0" width="0.20" diameter="0.40" />
		<r_wheel pos="0.0 -0.5" mass="4.0" width="0.20" diameter="0.40" />
		<chassis mass="15.0" zmin="0.05" zmax="0.6" />
		CONTROLLER
	</dynamics>
)";

const char* kAckermann = R"(
	<dynamics class="ackermann">
		<rl_wheel pos="-1.8  0.9" mass="6.0" width="0.20" diameter="0.8" />
		<rr_wheel pos="-1.8 -0.9" mass="6.0" width="0.20" diameter="0.8" />
		<fl_wheel mass="6.0" width="0.20" diameter="0.8" />
		<fr_wheel mass="6.0" width="0.20" diameter="0.8" />
		<f_wheels_x>1.7</f_wheels_x>
		<f_wheels_d>1.85</f_wheels_d>
		<max_steer_ang_deg>40.0</max_steer_ang_deg>
		<chassis mass="2000.0" zmin="0.01" zmax="1.9" />
		CONTROLLER
	</dynamics>
)";

const char* kDrivetrain = R"(
	<dynamics class="ackermann_drivetrain">
		<rl_wheel pos="0  0.20" mass="5.35" width="0.164" diameter="0.406" />
		<rr_wheel pos="0 -0.20" mass="5.35" width="0.164" diameter="0.406" />
		<fl_wheel mass="5.35" width="0.164" diameter="0.406" />
		<fr_wheel mass="5.35" width="0.164" diameter="0.406" />
		<f_wheels_x>0.62</f_wheels_x>
		<f_wheels_d>0.40</f_wheels_d>
		<max_steer_ang_deg>35.0</max_steer_ang_deg>
		<chassis mass="97.0" zmin="0" zmax="1.20" />
		<drivetrain type="open_rear" />
		CONTROLLER
	</dynamics>
)";

struct Sim
{
	mvsim::World world;
	mvsim::VehicleBase::Ptr veh;
	double dt;

	Sim(const std::string& dynamics, const std::string& controller, const std::string& friction,
		double timestep, const std::string& initVel = "0 0 0")
		: dt(timestep)
	{
		std::string dyn = dynamics;
		dyn.replace(dyn.find("CONTROLLER"), 10, controller);

		world.headless(true);
		world.setMinLoggingLevel(mrpt::system::LVL_WARN);
		world.set_simul_timestep(dt);
		world.internal_initialize();

		const std::string xml = "<vehicle name=\"veh\">" + dyn + friction +
								"<init_pose>0 0 0</init_pose><init_vel>" + initVel +
								"</init_vel></vehicle>";
		veh = mvsim::VehicleBase::factory(&world, xml);
		world.insert_vehicle(veh);
	}

	void run(double seconds, const mrpt::math::TTwist2D& cmd)
	{
		veh->getControllerInterface()->setTwistCommand(cmd);
		const int n = static_cast<int>(std::round(seconds / dt));
		for (int i = 0; i < n; i++)
		{
			world.run_simulation(dt);
		}
	}

	double speed() const
	{
		const auto v = veh->getRefVelocityLocal();
		return std::hypot(v.vx, v.vy);
	}
};

std::string friction(const std::string& cls, const std::string& mu, const std::string& extra = "")
{
	return "<friction class=\"" + cls + "\"><mu>" + mu + "</mu>" + extra + "</friction>";
}

// The ideal controller must follow the commanded twist exactly, also for
// skid-steer vehicles (whose wheel friction must not be applied to the chassis).
void test_ideal_circle()
{
	std::cout << "[TEST] twist_ideal: full circle, skid-steer\n";
	Sim s(kSkidSteer, "<controller class=\"twist_ideal\" />", friction("default", "0.8"), 1e-3);
	const double v = 1.0;
	const double w = 0.5;
	s.run(2 * M_PI / w, {v, 0, w});
	const auto p = s.veh->getPose();
	std::cout << "  final pose: " << p << "\n";
	// Back at the starting point:
	EXPECT_NEAR(p.x, 0.0, 0.01);
	EXPECT_NEAR(p.y, 0.0, 0.01);
	EXPECT_NEAR(std::remainder(p.yaw, 2 * M_PI), 0.0, 0.01);
	EXPECT_LT(std::abs(s.veh->getRefVelocityLocal().vy), 1e-4);
}

// PID controllers must bring the vehicle to a full stop with a zero twist.
void test_pid_stops(
	const char* name, const char* dynamics, const std::string& controller, const std::string& fric,
	double v)
{
	std::cout << "[TEST] zero twist stops the vehicle: " << name << "\n";
	Sim s(dynamics, controller, fric, 5e-3);
	s.run(4.0, {v, 0, 0});
	EXPECT_NEAR(s.veh->getRefVelocityLocal().vx, v, 0.05 * v);
	s.run(3.0, {0, 0, 0});
	const auto p0 = s.veh->getPose();
	s.run(1.0, {0, 0, 0});
	const auto p1 = s.veh->getPose();
	const double drift = std::hypot(p1.x - p0.x, p1.y - p0.y);
	std::cout << "  speed=" << s.speed() << " drift_last_1s=" << drift
			  << " vel=" << s.veh->getRefVelocityLocal().asString() << "\n";
	EXPECT_LT(s.speed(), 1e-3);
	EXPECT_LT(drift, 1e-3);
}

// Ward-Iagnemma rolling resistance must decelerate a free-rolling vehicle.
void test_ward_iagnemma_coast_down()
{
	std::cout << "[TEST] Ward-Iagnemma rolling resistance, coast down\n";
	Sim s(
		kDrivetrain, "<controller class=\"raw\" />",
		friction("wardiagnemma", "0.9", "<C_damping>0</C_damping>"), 1e-3, "1.0 0 0");
	s.run(1.0, {0, 0, 0});
	const double v0 = s.veh->getRefVelocityLocal().vx;
	s.run(1.0, {0, 0, 0});
	const double v1 = s.veh->getRefVelocityLocal().vx;
	// Expected: g*(R1 + R2*v) ~ 0.2 m/s^2, plus the chassis linear damping:
	const double decel = v0 - v1;
	std::cout << "  v0=" << v0 << " v1=" << v1 << " decel=" << decel << " m/s^2\n";
	EXPECT_GT(decel, 0.15);
	EXPECT_LT(decel, 0.40);
}

// The ellipse friction model with a PID must not run away.
void test_ellipse_pid()
{
	std::cout << "[TEST] ellipse friction + twist_pid\n";
	Sim s(
		kSkidSteer,
		"<controller class=\"twist_pid\"><KP>18</KP><KI>470</KI><max_torque>100</max_torque>"
		"</controller>",
		"<friction class=\"ellipse\"><C_damping>0.05</C_damping></friction>", 5e-3);
	s.run(4.0, {1.0, 0, 0});
	const auto p = s.veh->getPose();
	std::cout << "  vx=" << s.veh->getRefVelocityLocal().vx << " pose=" << p << "\n";
	EXPECT_NEAR(s.veh->getRefVelocityLocal().vx, 1.0, 0.05);
	EXPECT_NEAR(p.y, 0.0, 0.01);
}

// A stiff wheel velocity PID must remain stable while the wheels slip.
void test_stiff_pid_low_friction()
{
	std::cout << "[TEST] stiff Ackermann PID on low friction, 5 ms steps\n";
	Sim s(
		kAckermann,
		"<controller class=\"twist_front_steer_pid\"><KP>8000</KP><KI>100</KI>"
		"<max_torque>1250</max_torque></controller>",
		friction("default", "0.3"), 5e-3);
	s.run(3.0, {2.0, 0, 0});
	const double v = s.veh->getRefVelocityLocal().vx;
	// Traction limited acceleration (~1.5 m/s^2) until reaching the setpoint:
	std::cout << "  vx=" << v << "\n";
	EXPECT_NEAR(v, 2.0, 0.1);
	s.run(4.0, {0, 0, 0});
	EXPECT_LT(s.speed(), 1e-3);
}

}  // namespace

int main()
{
	try
	{
		test_ideal_circle();

		test_pid_stops(
			"differential", kDifferential,
			"<controller class=\"twist_pid\"><KP>64</KP><KI>10</KI><max_torque>14.44</max_torque>"
			"</controller>",
			friction("default", "0.7"), 1.0);
		test_pid_stops(
			"ackermann_drivetrain + Ward-Iagnemma", kDrivetrain,
			"<controller class=\"twist_front_steer_pid\"><KP>800</KP><KI>100</KI>"
			"<max_torque>100</max_torque></controller>",
			friction("wardiagnemma", "0.9", "<C_damping>0.05</C_damping>"), 0.5);
		test_pid_stops(
			"ackermann", kAckermann,
			"<controller class=\"twist_front_steer_pid\"><KP>8000</KP><KI>100</KI>"
			"<max_torque>1250</max_torque></controller>",
			friction("default", "0.7"), 2.0);

		test_ward_iagnemma_coast_down();
		test_ellipse_pid();
		test_stiff_pid_low_friction();
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
		return 1;
	}

	if (g_failures == 0)
	{
		std::printf("\nAll vehicle motion tests passed.\n");
	}
	else
	{
		std::fprintf(stderr, "\n%d test(s) FAILED.\n", g_failures);
	}
	return g_failures == 0 ? 0 : 1;
}
