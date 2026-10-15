/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Suspension tests: the model alone against its analytic solutions, and in a
// full World with a vehicle accelerating, braking and turning.

#include <mrpt/math/TTwist2D.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mvsim/Suspension.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{
using mvsim::Suspension;

// A 4-corner car-like chassis, symmetric about its center of mass:
Suspension::Params carParams(double f, double zeta)
{
	Suspension::Params p;
	p.sprungMass = 1200;
	p.com = {0.3, 0.0};
	p.comHeight = 0.55;
	p.I_xx = 450;
	p.I_yy = 1800;
	const double w = 2 * M_PI * f;
	const mrpt::math::TPoint2D corners[4] = {{1.6, 0.8}, {1.6, -0.8}, {-1.0, 0.8}, {-1.0, -0.8}};
	for (const auto& pos : corners)
	{
		Suspension::Corner c;
		c.pos = pos;
		c.stiffness = 0.25 * p.sprungMass * w * w;
		c.damping = 2 * zeta * w * 0.25 * p.sprungMass;
		p.corners.push_back(c);
	}
	return p;
}

Suspension::Vector3 vec3(double a, double b, double c)
{
	Suspension::Vector3 v;
	v[0] = a;
	v[1] = b;
	v[2] = c;
	return v;
}

// Free heave vibration: the exact discretization matches the analytic
// solution for any time step.
void test_free_heave()
{
	std::cout << "[TEST] free heave vibration vs analytic solution\n";
	const double f = 1.5;
	const double zeta = 0.3;
	const double w = 2 * M_PI * f;
	const double wd = w * std::sqrt(1 - zeta * zeta);
	const double z0 = 0.02;

	for (const double dt : {1e-3, 5e-3, 50e-3})
	{
		Suspension s;
		s.init(carParams(f, zeta));
		s.setState(vec3(z0, 0, 0), vec3(0, 0, 0));
		double maxErr = 0;
		const int n = static_cast<int>(std::round(2.0 / dt));
		for (int k = 1; k <= n; k++)
		{
			s.step(dt, vec3(0, 0, 0), {0, 0});
			const double t = k * dt;
			const double z =
				z0 * std::exp(-zeta * w * t) *
				(std::cos(wd * t) + zeta / std::sqrt(1 - zeta * zeta) * std::sin(wd * t));
			maxErr = std::max(maxErr, std::abs(s.displacement()[Suspension::HEAVE] - z));
		}
		std::cout << "  dt=" << dt << " max error=" << maxErr << " m\n";
		EXPECT_LT(maxErr, 1e-9);
	}

	Suspension s;
	s.init(carParams(f, zeta));
	std::cout << "  modes: " << s.modesAsString() << "\n";
	bool foundHeave = false;
	for (const auto& m : s.modes())
	{
		if (m.shape == "heave")
		{
			foundHeave = true;
			EXPECT_NEAR(m.frequency_hz, f, 1e-6);
			EXPECT_NEAR(m.damping_ratio, zeta, 1e-6);
		}
	}
	EXPECT_TRUE(foundHeave);
}

// Constant braking and cornering: steady pitch and roll from K*x = F.
void test_steady_state()
{
	std::cout << "[TEST] steady pitch (braking) and roll (cornering)\n";
	const auto p = carParams(1.5, 0.3);
	double Kpitch = 0;
	double Kroll = 0;
	for (const auto& c : p.corners)
	{
		Kpitch += c.stiffness * (c.pos.x - p.com.x) * (c.pos.x - p.com.x);
		Kroll += c.stiffness * (c.pos.y - p.com.y) * (c.pos.y - p.com.y);
	}
	const double mgh = p.sprungMass * p.gravity * p.comHeight;
	const double mh = p.sprungMass * p.comHeight;

	Suspension s;
	s.init(p);
	for (int k = 0; k < 2000; k++)
	{
		s.step(5e-3, vec3(0, 0, 0), {-6.0, 4.0});  // braking, turning left
	}
	const auto& d = s.displacement();
	const double pitch = mh * 6.0 / (Kpitch - mgh);
	const double roll = mh * 4.0 / (Kroll - mgh);
	std::cout << "  pitch=" << d[Suspension::PITCH] << " (expected " << pitch
			  << ") roll=" << d[Suspension::ROLL] << " (expected " << roll << ") rad\n";
	EXPECT_NEAR(d[Suspension::PITCH], pitch, 1e-6);	 // nose down
	EXPECT_NEAR(d[Suspension::ROLL], roll, 1e-6);  // left side up
	EXPECT_GT(pitch, 0);
	EXPECT_GT(roll, 0);

	// The front corners carry more load, and the outer (right) ones too:
	const auto dN = s.cornerLoadChanges();
	EXPECT_GT(dN[0] + dN[1], 0);
	EXPECT_GT(dN[1] + dN[3], dN[0] + dN[2]);
	double sum = 0;
	for (const double n : dN)
	{
		sum += n;
	}
	EXPECT_NEAR(sum, 0, 1e-3);
}

// A step of the ground: the chassis lags behind, then settles on it.
void test_ground_step()
{
	std::cout << "[TEST] ground step\n";
	Suspension s;
	s.init(carParams(1.5, 0.3));
	const double dt = 1e-3;
	const double step = 0.05;
	s.step(dt, vec3(step / dt, 0, 0), {0, 0});
	const double z1 = s.displacement()[Suspension::HEAVE];
	double minZ = z1;
	double maxAbsZ = 0;
	for (int k = 0; k < 5000; k++)
	{
		s.step(dt, vec3(0, 0, 0), {0, 0});
		minZ = std::min(minZ, s.displacement()[Suspension::HEAVE]);
		if (k > 4000)
		{
			maxAbsZ = std::max(maxAbsZ, std::abs(s.displacement()[Suspension::HEAVE]));
		}
	}
	std::cout << "  right after the step: " << z1 << " m, settled: " << maxAbsZ << " m\n";
	EXPECT_LT(z1, -0.9 * step);	 // the wheels went up, the chassis did not yet
	EXPECT_GT(z1, -step - 1e-9);
	EXPECT_LT(maxAbsZ, 1e-4);
}

// With all corners on one axle, the pitch is locked.
void test_locked_pitch()
{
	std::cout << "[TEST] locked pitch with all wheels on one axle\n";
	Suspension::Params p;
	p.sprungMass = 15;
	p.com = {0, 0};
	p.comHeight = 0.3;
	p.I_xx = 0.5;
	p.I_yy = 0.5;
	for (const double y : {0.25, -0.25})
	{
		Suspension::Corner c;
		c.pos = {0, y};
		c.stiffness = 3000;
		c.damping = 60;
		p.corners.push_back(c);
	}
	Suspension s;
	s.init(p);
	std::cout << "  modes: " << s.modesAsString() << "\n";
	EXPECT_TRUE(s.modes().size() == 2);
	for (int k = 0; k < 500; k++)
	{
		s.step(5e-3, vec3(0, 0, 0.1), {-3.0, 0});
	}
	EXPECT_NEAR(s.displacement()[Suspension::PITCH], 0, 1e-12);
	EXPECT_NEAR(s.absoluteVelocity()[Suspension::PITCH], 0.1, 1e-12);
}

// Springs too soft for a tall chassis: it would tip over.
void test_unstable()
{
	std::cout << "[TEST] unstable (too soft) suspension is rejected\n";
	auto p = carParams(0.2, 0.3);
	p.comHeight = 3.0;
	bool thrown = false;
	try
	{
		Suspension s;
		s.init(p);
	}
	catch (const std::exception&)
	{
		thrown = true;
	}
	EXPECT_TRUE(thrown);
}

const char* kCar = R"(
<vehicle name="veh">
	<dynamics class="ackermann">
		<rl_wheel pos="-1.0  0.8" mass="15.0" width="0.20" diameter="0.6" />
		<rr_wheel pos="-1.0 -0.8" mass="15.0" width="0.20" diameter="0.6" />
		<fl_wheel mass="15.0" width="0.20" diameter="0.6" />
		<fr_wheel mass="15.0" width="0.20" diameter="0.6" />
		<f_wheels_x>1.6</f_wheels_x>
		<f_wheels_d>1.6</f_wheels_d>
		<max_steer_ang_deg>35.0</max_steer_ang_deg>
		<chassis mass="1200.0" zmin="0.2" zmax="1.4">
			<shape> <pt>-1.7 -0.9</pt> <pt>2.3 -0.9</pt> <pt>2.3 0.9</pt> <pt>-1.7 0.9</pt> </shape>
		</chassis>
		<controller class="twist_front_steer_pid">
			<KP>4000</KP><KI>80</KI><max_torque>900</max_torque>
		</controller>
	</dynamics>
	<friction class="default"><mu>0.9</mu></friction>
	SUSPENSION
	<sensor class="imu" name="imu1">
		<pose_3d>0.3 0 1.0 0 0 0</pose_3d>
		<sensor_period>0.005</sensor_period>
		<angular_velocity_white_noise_std_noise>0</angular_velocity_white_noise_std_noise>
		<linear_acceleration_white_noise_std_noise>0</linear_acceleration_white_noise_std_noise>
	</sensor>
	<init_pose>0 0 0</init_pose>
</vehicle>
)";

struct CarSim
{
	mvsim::World world;
	mvsim::VehicleBase::Ptr veh;
	double dt = 5e-3;
	double lastImuWy = 0;
	double maxAbsImuWy = 0;

	explicit CarSim(const std::string& suspension)
	{
		std::string xml = kCar;
		xml.replace(xml.find("SUSPENSION"), 10, suspension);
		world.headless(true);
		world.setMinLoggingLevel(mrpt::system::LVL_WARN);
		world.set_simul_timestep(dt);
		world.internal_initialize();
		veh = mvsim::VehicleBase::factory(&world, xml);
		world.insert_vehicle(veh);
		world.registerCallbackOnObservation(
			[this](const mvsim::Simulable&, const mrpt::obs::CObservation::Ptr& o)
			{
				if (auto imu = std::dynamic_pointer_cast<mrpt::obs::CObservationIMU>(o); imu)
				{
					lastImuWy = imu->get(mrpt::obs::IMU_WY);
					maxAbsImuWy = std::max(maxAbsImuWy, std::abs(lastImuWy));
				}
			});
	}

	// Runs and returns the min and max of the suspension pitch and roll
	struct Range
	{
		double minPitch = 1e9;
		double maxPitch = -1e9;
		double minRoll = 1e9;
		double maxRoll = -1e9;
	};
	Range run(double seconds, const mrpt::math::TTwist2D& cmd)
	{
		veh->getControllerInterface()->setTwistCommand(cmd);
		Range r;
		const int n = static_cast<int>(std::round(seconds / dt));
		for (int i = 0; i < n; i++)
		{
			world.run_simulation(dt);
			if (const auto* s = veh->getSuspension(); s)
			{
				const auto& d = s->displacement();
				r.minPitch = std::min(r.minPitch, d[Suspension::PITCH]);
				r.maxPitch = std::max(r.maxPitch, d[Suspension::PITCH]);
				r.minRoll = std::min(r.minRoll, d[Suspension::ROLL]);
				r.maxRoll = std::max(r.maxRoll, d[Suspension::ROLL]);
			}
		}
		return r;
	}
};

const char* kSuspension = R"(
	<suspension enabled="true">
		<frequency_hz>1.3</frequency_hz>
		<damping_ratio>0.3</damping_ratio>
	</suspension>)";

// Squat when accelerating, dive when braking, roll outwards in turns, and
// the wheel loads, chassis pose and IMU follow the chassis motion.
void test_vehicle()
{
	std::cout << "[TEST] vehicle with suspension: accelerate, brake, turn\n";
	CarSim sim(kSuspension);
	const auto* s = sim.veh->getSuspension();
	EXPECT_TRUE(s != nullptr);
	if (!s)
	{
		return;
	}
	std::cout << "  modes: " << s->modesAsString() << "\n";
	for (const auto& m : s->modes())
	{
		if (m.shape == "heave")
		{
			EXPECT_NEAR(m.frequency_hz, 1.3, 0.05);
		}
	}

	// Parked: no motion at all
	sim.run(1.0, {0, 0, 0});
	const auto loads0 = sim.veh->getWheelLoads();
	EXPECT_NEAR(s->displacement()[Suspension::PITCH], 0, 1e-9);

	const auto acc = sim.run(1.5, {5.0, 0, 0});
	std::cout << "  accelerating: min pitch=" << acc.minPitch << " rad\n";
	EXPECT_LT(acc.minPitch, -1e-3);	 // nose up

	sim.run(4.0, {5.0, 0, 0});
	sim.maxAbsImuWy = 0;
	const auto brk = sim.run(0.5, {0, 0, 0});
	std::cout << "  braking: max pitch=" << brk.maxPitch << " rad\n";
	EXPECT_GT(brk.maxPitch, 1e-3);	// nose down
	const auto& loads = sim.veh->getWheelLoads();
	std::cout << "  front loads: " << loads[2] + loads[3] << " N (static: " << loads0[2] + loads0[3]
			  << " N)\n";
	EXPECT_GT(loads[2] + loads[3], loads0[2] + loads0[3]);

	// The chassis pose includes the suspension; the IMU feels the pitch rate:
	const auto q = sim.veh->getPose();
	const auto qc = sim.veh->getChassisPose();
	EXPECT_NEAR(qc.pitch - q.pitch, s->displacement()[Suspension::PITCH], 1e-6);
	EXPECT_NEAR(qc.roll - q.roll, s->displacement()[Suspension::ROLL], 1e-6);
	std::cout << "  IMU max |wy| while braking: " << sim.maxAbsImuWy << " rad/s\n";
	EXPECT_GT(sim.maxAbsImuWy, 0.01);

	// Steady left turn: the chassis rolls to the right (left side up)
	sim.run(3.0, {0, 0, 0});
	const double v = 4.0;
	const double w = 0.4;
	sim.run(8.0, {v, 0, w});
	const double ay = sim.veh->getRefVelocityLocal().vx * sim.veh->getRefVelocityLocal().omega;
	const auto& p = s->params();
	double Kroll = 0;
	for (const auto& c : p.corners)
	{
		Kroll += c.stiffness * (c.pos.y - p.com.y) * (c.pos.y - p.com.y);
	}
	const double rollExpected =
		p.sprungMass * p.comHeight * ay / (Kroll - p.sprungMass * p.gravity * p.comHeight);
	const double roll = s->displacement()[Suspension::ROLL];
	std::cout << "  turning: ay=" << ay << " roll=" << roll << " (expected " << rollExpected
			  << ") rad\n";
	EXPECT_GT(roll, 0);
	EXPECT_NEAR(roll, rollExpected, 0.1 * std::abs(rollExpected));
}

// Without a <suspension> block, the chassis is the vehicle body.
void test_disabled_by_default()
{
	std::cout << "[TEST] suspension disabled by default\n";
	CarSim sim("");
	EXPECT_TRUE(sim.veh->getSuspension() == nullptr);
	sim.run(1.0, {3.0, 0, 0});
	const auto q = sim.veh->getPose();
	const auto qc = sim.veh->getChassisPose();
	EXPECT_NEAR(qc.x, q.x, 0);
	EXPECT_NEAR(qc.pitch, q.pitch, 0);
	EXPECT_NEAR(qc.roll, q.roll, 0);
}
}  // namespace

int main()
{
	try
	{
		test_free_heave();
		test_steady_state();
		test_ground_step();
		test_locked_pitch();
		test_unstable();
		test_vehicle();
		test_disabled_by_default();
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
		return 1;
	}

	if (g_failures == 0)
	{
		std::printf("\nAll suspension tests passed.\n");
		return 0;
	}
	std::fprintf(stderr, "\n%d test(s) FAILED.\n", g_failures);
	return 1;
}
