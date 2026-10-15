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
		<chassis mass="10.0" zmin="0.05" zmax="0.25">
			<shape> <pt>-0.25 -0.2</pt> <pt>0.25 -0.2</pt> <pt>0.25 0.2</pt> <pt>-0.25 0.2</pt> </shape>
		</chassis>
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

	// From a complete world XML, with a vehicle named "veh":
	Sim(const std::string& worldXml, double timestep) : dt(timestep)
	{
		world.headless(true);
		world.setMinLoggingLevel(mrpt::system::LVL_WARN);
		world.load_from_XML(
			"<mvsim_world version=\"1.0\"><simul_timestep>" + std::to_string(dt) +
			"</simul_timestep>" + worldXml + "</mvsim_world>");
		veh = world.getListOfVehicles().find("veh")->second;
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

// With isotropic Coulomb friction, a skid-steer robot whose wheels spin at the
// commanded speeds turns slower: the yaw moment balance of the sliding wheels
// at (+-a, +-b) gives w = w_cmd * b^2 / (a^2 + b^2).
void test_skid_steer_spin_rate()
{
	std::cout << "[TEST] skid-steer spin rate (Coulomb friction)\n";
	Sim s(
		kSkidSteer,
		"<controller class=\"twist_pid\"><KP>18</KP><KI>470</KI><max_torque>100</max_torque>"
		"</controller>",
		friction("default", "0.8"), 5e-3);
	s.run(4.0, {0, 0, 1.0});
	const double expected = 0.16 * 0.16 / (0.13 * 0.13 + 0.16 * 0.16);
	std::cout << "  w=" << s.veh->getRefVelocityLocal().omega << " expected=" << expected << "\n";
	EXPECT_NEAR(s.veh->getRefVelocityLocal().omega, expected, 0.02);
}

// A caster wheel swivels, so it must not resist turning: same turn rate as
// without it.
double turn_rate_with_and_without_caster(bool withCaster)
{
	std::string dyn = R"(
	<dynamics class="DYN">
		<l_wheel pos="0.0  0.5" mass="4.0" width="0.20" diameter="0.40" />
		<r_wheel pos="0.0 -0.5" mass="4.0" width="0.20" diameter="0.40" />
		CASTER
		<chassis mass="15.0" zmin="0.05" zmax="0.6" />
		CONTROLLER
	</dynamics>
)";
	dyn.replace(dyn.find("DYN"), 3, withCaster ? "differential_3_wheels" : "differential");
	dyn.replace(
		dyn.find("CASTER"), 6,
		withCaster ? R"(<caster_wheel pos="0.5 0.0" mass="0.5" width="0.05" diameter="0.10" />)"
				   : "");
	Sim s(
		dyn,
		"<controller class=\"twist_pid\"><KP>64</KP><KI>100</KI><max_torque>14.44</max_torque>"
		"</controller>",
		friction("default", "0.7"), 5e-3);
	s.run(6.0, {0.5, 0, 0.5});
	return s.veh->getRefVelocityLocal().omega;
}

void test_caster()
{
	std::cout << "[TEST] differential_3_wheels: the caster does not resist turning\n";
	const double w3 = turn_rate_with_and_without_caster(true);
	const double w2 = turn_rate_with_and_without_caster(false);
	std::cout << "  w (caster)=" << w3 << " w (no caster)=" << w2 << "\n";
	EXPECT_NEAR(w3, w2, 0.02 * w2);
}

// With one driven wheel on ice, a Torsen differential transfers more torque to
// the other one than an open differential.
double drivetrain_speed_one_wheel_on_ice(const std::string& diffType)
{
	const std::string xml = R"(
	<element class="property_region">
		<property_name>friction_mu</property_name> <value_double>0.05</value_double>
		<x_min>-100</x_min> <y_min>0</y_min> <z_min>-10</z_min>
		<x_max>100</x_max> <y_max>100</y_max> <z_max>10</z_max>
	</element>
	<vehicle name="veh">
	<dynamics class="ackermann_drivetrain">
		<rl_wheel pos="0  0.20" mass="5.35" width="0.164" diameter="0.406" />
		<rr_wheel pos="0 -0.20" mass="5.35" width="0.164" diameter="0.406" />
		<fl_wheel mass="5.35" width="0.164" diameter="0.406" />
		<fr_wheel mass="5.35" width="0.164" diameter="0.406" />
		<f_wheels_x>0.62</f_wheels_x>
		<f_wheels_d>0.40</f_wheels_d>
		<chassis mass="97.0" zmin="0" zmax="1.20" />
		<drivetrain type=")" +
							diffType +
							R"(" ><rear_left_right_bias>3.0</rear_left_right_bias></drivetrain>
		<controller class="twist_front_steer_pid">
			<KP>800</KP><KI>100</KI><max_torque>100</max_torque>
		</controller>
	</dynamics>
	<friction class="default"><mu>0.9</mu></friction>
	<init_pose>0 0 0</init_pose>
	</vehicle>
	)";
	Sim s(xml, 5e-3);
	s.run(2.0, {1.0, 0, 0});
	return s.veh->getRefVelocityLocal().vx;
}

void test_torsen()
{
	std::cout << "[TEST] Torsen vs open differential, one wheel on ice\n";
	const double vOpen = drivetrain_speed_one_wheel_on_ice("open_rear");
	const double vTorsen = drivetrain_speed_one_wheel_on_ice("torsen_rear");
	std::cout << "  vx: open=" << vOpen << " torsen=" << vTorsen << "\n";
	EXPECT_GT(vTorsen, 1.5 * vOpen);
}

// The steering angle follows the commands at the configured rate.
void test_steering_rate()
{
	std::cout << "[TEST] Ackermann steering rate limit\n";
	std::string dyn = kAckermann;
	dyn.replace(dyn.find("<max_steer_ang_deg>"), 0, "<max_steer_rate_deg>30</max_steer_rate_deg>");
	Sim s(dyn, "<controller class=\"twist_ideal\" />", friction("default", "0.7"), 5e-3);
	s.run(0.5, {2.0, 0, 0.5});
	// 30 deg/s during 0.5 s: the equivalent (central) steering angle is 15 deg
	const double fl = s.veh->getWheelInfo(2).yaw;
	const double fr = s.veh->getWheelInfo(3).yaw;
	std::cout << "  front wheels yaw: " << mrpt::RAD2DEG(fl) << " " << mrpt::RAD2DEG(fr)
			  << " deg\n";
	EXPECT_GT(mrpt::RAD2DEG(fl), 14.0);
	EXPECT_LT(mrpt::RAD2DEG(fr), 15.0);

	// Commands beyond the steering limit must not delay a later reversal:
	std::string dt = kDrivetrain;
	dt.replace(dt.find("<max_steer_ang_deg>"), 0, "<max_steer_rate_deg>30</max_steer_rate_deg>");
	Sim s2(
		dt,
		"<controller class=\"twist_front_steer_pid\"><KP>800</KP><KI>100</KI>"
		"<max_torque>100</max_torque></controller>",
		friction("default", "0.9"), 5e-3);
	s2.run(3.0, {0.3, 0, 3.0});	 // asks for ~80 deg, the limit is 35 deg
	s2.run(1.0, {0.3, 0, -3.0});
	// From full lock at 30 deg/s, the equivalent angle is now ~5 deg
	const double fl2 = mrpt::RAD2DEG(s2.veh->getWheelInfo(2).yaw);
	std::cout << "  after reversal, front left wheel yaw: " << fl2 << " deg\n";
	EXPECT_LT(fl2, 10.0);
}

// Bearing damping is integrated implicitly: no blow-up for large values.
void test_large_wheel_damping()
{
	std::cout << "[TEST] large wheel bearing damping\n";
	Sim s(
		kSkidSteer,
		"<controller class=\"twist_pid\"><KP>18</KP><KI>470</KI><max_torque>100</max_torque>"
		"</controller>",
		friction("default", "0.8", "<C_damping>50</C_damping>"), 5e-3);
	s.run(4.0, {1.0, 0, 0});
	const double vx = s.veh->getRefVelocityLocal().vx;
	std::cout << "  vx=" << vx << "\n";
	// Limited by the damping: max_torque/C_damping*R = 100/50*0.1
	EXPECT_TRUE(std::isfinite(vx));
	EXPECT_NEAR(vx, 0.2, 0.01);
}

// Wheel loads from the force and moment balance; a semi-trailer rests on its
// kingpin.
void test_wheel_loads()
{
	std::cout << "[TEST] wheel loads\n";
	{
		Sim s(kAckermann, "<controller class=\"twist_ideal\" />", friction("default", "0.7"), 5e-3);
		s.run(0.1, {0, 0, 0});
		const auto& n = s.veh->getWheelLoads();
		const auto* b = s.veh->getBox2DChassisBody();
		const double W = b->GetMass() * 9.81;
		// Rear (0,1) at x=-1.8, front (2,3) at x=1.7: moments about the COM
		const double xc = b->GetLocalCenter().x;
		const double expectedFront = W * (xc + 1.8) / 3.5;
		std::cout << "  ackermann loads: " << n[0] << " " << n[1] << " " << n[2] << " " << n[3]
				  << " (expected front axle: " << expectedFront << ")\n";
		EXPECT_NEAR(n[0] + n[1] + n[2] + n[3], W, 1e-3 * W);
		EXPECT_NEAR(n[2] + n[3], expectedFront, 1e-3 * W);
	}
	{
		const std::string xml = R"(
		<vehicle name="tractor">
			<dynamics class="differential">
				<l_wheel pos="0  0.9" mass="10" width="0.3" diameter="0.8" />
				<r_wheel pos="0 -0.9" mass="10" width="0.3" diameter="0.8" />
				<chassis mass="1000" zmin="0.3" zmax="2.0" />
				<controller class="twist_ideal" />
			</dynamics>
			<init_pose>0 0 0</init_pose>
		</vehicle>
		<vehicle name="veh">
			<dynamics class="differential_4_wheels">
				<lf_wheel pos="-4.25  0.8" mass="10" width="0.35" diameter="0.75" />
				<rf_wheel pos="-4.25 -0.8" mass="10" width="0.35" diameter="0.75" />
				<lr_wheel pos="-5.25  0.8" mass="10" width="0.35" diameter="0.75" />
				<rr_wheel pos="-5.25 -0.8" mass="10" width="0.35" diameter="0.75" />
				<chassis mass="4000" zmin="0.5" zmax="4.0">
					<shape> <pt>-6.2 1.25</pt> <pt>0.5 1.25</pt> <pt>0.5 -1.25</pt> <pt>-6.2 -1.25</pt> </shape>
				</chassis>
				<controller class="raw" />
			</dynamics>
			<init_pose>-0.2 0 0</init_pose>
		</vehicle>
		<joint type="revolute" body_a="tractor" anchor_a="-0.2 0" body_b="veh" anchor_b="0 0" />
		)";
		Sim s(xml, 5e-3);
		s.run(0.1, {0, 0, 0});
		const auto& n = s.veh->getWheelLoads();
		const double W = s.veh->getBox2DChassisBody()->GetMass() * 9.81;
		const double sum = n[0] + n[1] + n[2] + n[3];
		std::cout << "  trailer loads: " << n[0] << " " << n[1] << " " << n[2] << " " << n[3]
				  << " (weight: " << W << ")\n";
		EXPECT_LT(sum, 0.9 * W);  // the rest is on the kingpin
		EXPECT_GT(sum, 0.1 * W);
	}
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
		test_skid_steer_spin_rate();
		test_caster();
		test_torsen();
		test_steering_rate();
		test_large_wheel_damping();
		test_wheel_loads();
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
