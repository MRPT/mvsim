/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Joint commands of Ackermann vehicles ("ros2_control" controller class,
// without ROS): joint layouts, steering commands (per wheel and central),
// limits, drivetrain traction joint, joint states, and determinism.

#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>
#include <mvsim/WheelJointsInterface.h>
#include <mvsim/World.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{
using mvsim::WheelJointsInterface;
using JointCommand = WheelJointsInterface::JointCommand;
using JointKind = WheelJointsInterface::JointKind;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kWheelbase = 2.5;
constexpr double kRadius = 0.3;

// ${DYN}: dynamics class; ${EXTRA}: more <controller> parameters
const char* kWorldXml = R"(
<mvsim_world version="1.0">
  <simul_timestep>0.005</simul_timestep>
  <element class="ground_grid"/>
  <vehicle name="car">
    <dynamics class="${DYN}">
      <rl_wheel pos="0  0.8" mass="10.0" width="0.25" diameter="0.6" />
      <rr_wheel pos="0 -0.8" mass="10.0" width="0.25" diameter="0.6" joint_name="custom_rr" />
      <fl_wheel mass="10.0" width="0.25" diameter="0.6" />
      <fr_wheel mass="10.0" width="0.25" diameter="0.6" />
      <f_wheels_x>2.5</f_wheels_x>
      <f_wheels_d>1.6</f_wheels_d>
      <max_steer_ang_deg>30.0</max_steer_ang_deg>
      <max_steer_rate_deg>60</max_steer_rate_deg>
      <chassis mass="800.0" zmin="0.15" zmax="1.2" />
      <drivetrain type="open_rear"/>
      <controller class="ros2_control">
        <KP>2550</KP> <KI>51</KI> <max_torque>287</max_torque>
        ${EXTRA}
      </controller>
    </dynamics>
    <friction class="default"> <mu>0.8</mu> <C_damping>0.5</C_damping> </friction>
    <init_pose>0 0 0</init_pose>
  </vehicle>
</mvsim_world>
)";

void replace(std::string& s, const std::string& tag, const std::string& value)
{
	s.replace(s.find(tag), tag.size(), value);
}

struct Sim
{
	mvsim::World world;
	mvsim::VehicleBase::Ptr veh;
	WheelJointsInterface* joints = nullptr;

	Sim(const std::string& dynamics, const std::string& extra)
	{
		std::string xml = kWorldXml;
		replace(xml, "${DYN}", dynamics);
		replace(xml, "${EXTRA}", extra);
		world.headless(true);
		world.load_from_XML(xml, ".");
		veh = world.getListOfVehicles().begin()->second;
		joints = veh->getControllerInterface()->wheelJointsInterface();
		ASSERT_(joints);
	}

	size_t joint(const std::string& name) const
	{
		const auto& js = joints->joints();
		for (size_t i = 0; i < js.size(); i++)
		{
			if (js[i].name == name)
			{
				return i;
			}
		}
		THROW_EXCEPTION_FMT("No joint '%s'", name.c_str());
	}

	void command(const std::map<std::string, double>& cmds)
	{
		for (const auto& [name, value] : cmds)
		{
			joints->setCommand(joint(name), value);
		}
	}

	WheelJointsInterface::JointState state(const std::string& name) const
	{
		return joints->getStates().at(joint(name));
	}

	void run(double duration)
	{
		const auto n = std::lround(duration / 0.01);
		for (long i = 0; i < n; i++)
		{
			world.run_simulation(0.01);
		}
	}
};

// Expected joints: name, kind, command interface
using JointList = std::vector<std::tuple<std::string, JointKind, JointCommand>>;

void checkJoints(const Sim& sim, const JointList& expected)
{
	const auto& js = sim.joints->joints();
	EXPECT_TRUE(js.size() == expected.size());
	for (size_t i = 0; i < std::min(js.size(), expected.size()); i++)
	{
		const auto& [name, kind, cmd] = expected[i];
		EXPECT_TRUE(js[i].name == name);
		EXPECT_TRUE(js[i].kind == kind);
		EXPECT_TRUE(js[i].command == cmd);
	}
}

void test_layouts()
{
	const auto V = JointCommand::Velocity;
	const auto P = JointCommand::Position;
	const auto N = JointCommand::None;
	const auto Spin = JointKind::WheelSpin;
	const auto Steer = JointKind::WheelSteering;
	{
		Sim sim("ackermann", "");
		checkJoints(
			sim, {{"rl_wheel_joint", Spin, V},
				  {"custom_rr", Spin, V},
				  {"fl_wheel_joint", Spin, V},
				  {"fr_wheel_joint", Spin, V},
				  {"fl_steer_joint", Steer, P},
				  {"fr_steer_joint", Steer, P}});
		// Steering limits: the inner wheel angle when turning toward the side
		// of the wheel, the outer one otherwise
		auto& ack = dynamic_cast<mvsim::DynamicsAckermann&>(*sim.veh);
		double inner = 0;
		double outer = 0;
		ack.computeFrontWheelAngles(ack.getMaxSteeringAngle(), inner, outer);
		EXPECT_GT(inner, ack.getMaxSteeringAngle());
		EXPECT_LT(outer, ack.getMaxSteeringAngle());
		const auto& fl = sim.joints->joints()[4];
		const auto& fr = sim.joints->joints()[5];
		EXPECT_NEAR(fl.positionMin, -outer, 1e-12);
		EXPECT_NEAR(fl.positionMax, inner, 1e-12);
		EXPECT_NEAR(fr.positionMin, -inner, 1e-12);
		EXPECT_NEAR(fr.positionMax, outer, 1e-12);
		EXPECT_NEAR(fl.velocityLimit, mrpt::DEG2RAD(60.0), 1e-12);
	}
	{
		Sim sim(
			"ackermann",
			"<steering_joints>central</steering_joints><steer_joint_name>my_steer"
			"</steer_joint_name><command_interface>effort</command_interface>");
		const auto E = JointCommand::Effort;
		checkJoints(
			sim, {{"rl_wheel_joint", Spin, E},
				  {"custom_rr", Spin, E},
				  {"fl_wheel_joint", Spin, E},
				  {"fr_wheel_joint", Spin, E},
				  {"my_steer", JointKind::CentralSteering, P},
				  {"fl_steer_joint", Steer, N},
				  {"fr_steer_joint", Steer, N}});
		EXPECT_NEAR(sim.joints->joints()[4].positionMin, -mrpt::DEG2RAD(30.0), 1e-12);
		EXPECT_NEAR(sim.joints->joints()[4].positionMax, mrpt::DEG2RAD(30.0), 1e-12);
	}
	{
		Sim sim("ackermann_drivetrain", "");
		checkJoints(
			sim, {{"rl_wheel_joint", Spin, N},
				  {"custom_rr", Spin, N},
				  {"fl_wheel_joint", Spin, N},
				  {"fr_wheel_joint", Spin, N},
				  {"traction_joint", JointKind::CentralTraction, V},
				  {"steer_joint", JointKind::CentralSteering, P},
				  {"fl_steer_joint", Steer, N},
				  {"fr_steer_joint", Steer, N}});
	}
}

// Wheel speeds of the rear axle (left, right) for a speed at its center and a
// steering angle, as ackermann_steering_controller commands them
std::pair<double, double> rearWheelSpeeds(double v, double delta)
{
	const double halfTrack = 0.8;
	const double turningRadius = kWheelbase / std::tan(delta);
	return {
		v * (turningRadius - halfTrack) / (turningRadius * kRadius),
		v * (turningRadius + halfTrack) / (turningRadius * kRadius)};
}

// Rear wheel drive, steering per wheel with the Ackermann geometry (as
// ackermann_steering_controller does): the vehicle turns with the radius of
// a bicycle model.
mrpt::math::TPose3D per_wheel_steering(double delta)
{
	Sim sim("ackermann", "");
	auto& ack = dynamic_cast<mvsim::DynamicsAckermann&>(*sim.veh);

	double fl = 0;
	double fr = 0;
	ack.computeFrontWheelAngles(delta, fl, fr);

	const double v = 2.0;  // m/s
	const auto [wl, wr] = rearWheelSpeeds(v, delta);
	sim.command(
		{{"rl_wheel_joint", wl},
		 {"custom_rr", wr},
		 {"fl_wheel_joint", kNaN},
		 {"fr_wheel_joint", kNaN},
		 {"fl_steer_joint", fl},
		 {"fr_steer_joint", fr}});
	sim.run(6.0);

	EXPECT_NEAR(sim.state("fl_steer_joint").position, fl, 1e-9);
	EXPECT_NEAR(sim.state("fr_steer_joint").position, fr, 1e-9);
	EXPECT_NEAR(sim.state("fl_steer_joint").velocity, 0.0, 1e-9);
	EXPECT_NEAR(sim.state("rl_wheel_joint").velocity, wl, 0.05 * wl);
	EXPECT_NEAR(sim.state("custom_rr").velocity, wr, 0.05 * wr);
	// Free front wheels roll, with no torque:
	EXPECT_GT(sim.state("fl_wheel_joint").velocity, 0.5 * v / kRadius);
	EXPECT_NEAR(sim.state("fl_wheel_joint").effort, 0.0, 1e-12);

	const auto vel = sim.veh->getRefVelocityLocal();
	std::cout << "per-wheel steering: vel " << vel.asString() << "\n";
	EXPECT_NEAR(vel.vx, v, 0.1);
	EXPECT_NEAR(vel.omega, v * std::tan(delta) / kWheelbase, 0.05 * v * std::tan(std::abs(delta)));
	return sim.veh->getPose();
}

void test_per_wheel_steering()
{
	// Turning right is the mirror image of turning left:
	const auto left = per_wheel_steering(mrpt::DEG2RAD(15.0));
	const auto right = per_wheel_steering(mrpt::DEG2RAD(-15.0));
	EXPECT_GT(left.y, 1.0);
	EXPECT_NEAR(left.x, right.x, 1e-3);
	EXPECT_NEAR(left.y, -right.y, 1e-3);
	EXPECT_NEAR(left.yaw, -right.yaw, 1e-4);
}

// Steering limits: angle and rate
void test_steering_limits()
{
	Sim sim("ackermann", "");
	auto& ack = dynamic_cast<mvsim::DynamicsAckermann&>(*sim.veh);
	const double maxRate = mrpt::DEG2RAD(60.0);

	double inner = 0;
	double outer = 0;
	ack.computeFrontWheelAngles(ack.getMaxSteeringAngle(), inner, outer);

	// A left turn beyond the limits:
	sim.command({{"fl_steer_joint", 1.2}, {"fr_steer_joint", 1.2}});
	sim.run(0.1);  // 20 steps
	// Rate limited:
	EXPECT_NEAR(sim.state("fl_steer_joint").velocity, maxRate, 1e-6);
	EXPECT_NEAR(sim.state("fr_steer_joint").velocity, maxRate, 1e-6);
	EXPECT_NEAR(sim.state("fl_steer_joint").position, maxRate * 0.1, 1e-9);
	sim.run(2.0);
	// Angle limited, keeping the Ackermann geometry:
	EXPECT_NEAR(sim.state("fl_steer_joint").position, inner, 1e-9);
	EXPECT_NEAR(sim.state("fr_steer_joint").position, outer, 1e-9);
	// And a right one:
	sim.command({{"fl_steer_joint", -1.2}, {"fr_steer_joint", -1.2}});
	sim.run(3.0);
	EXPECT_NEAR(sim.state("fl_steer_joint").position, -outer, 1e-9);
	EXPECT_NEAR(sim.state("fr_steer_joint").position, -inner, 1e-9);

	// No command (NaN): the steering keeps its angle
	sim.command({{"fl_steer_joint", kNaN}, {"fr_steer_joint", 0.1}});
	sim.run(2.0);
	EXPECT_NEAR(sim.state("fl_steer_joint").position, -outer, 1e-9);
	EXPECT_NEAR(sim.state("fr_steer_joint").position, 0.1, 1e-9);
}

// One central steering joint: the front wheels follow the Ackermann geometry
void test_central_steering()
{
	Sim sim("ackermann", "<steering_joints>central</steering_joints>");
	auto& ack = dynamic_cast<mvsim::DynamicsAckermann&>(*sim.veh);

	const double delta = mrpt::DEG2RAD(-12.0);
	const double v = 2.0;
	const auto [wl, wr] = rearWheelSpeeds(v, delta);
	sim.command(
		{{"rl_wheel_joint", wl},
		 {"custom_rr", wr},
		 {"fl_wheel_joint", kNaN},
		 {"fr_wheel_joint", kNaN},
		 {"steer_joint", delta},
		 // State-only joints: ignored
		 {"fl_steer_joint", 0.3},
		 {"fr_steer_joint", 0.3}});
	sim.run(6.0);

	double fl = 0;
	double fr = 0;
	ack.computeFrontWheelAngles(delta, fl, fr);
	EXPECT_NEAR(sim.state("steer_joint").position, delta, 1e-9);
	EXPECT_NEAR(sim.state("fl_steer_joint").position, fl, 1e-9);
	EXPECT_NEAR(sim.state("fr_steer_joint").position, fr, 1e-9);

	const auto vel = sim.veh->getRefVelocityLocal();
	std::cout << "central steering: vel " << vel.asString() << "\n";
	EXPECT_NEAR(vel.vx, v, 0.1);
	EXPECT_NEAR(vel.omega, v * std::tan(delta) / kWheelbase, 0.05 * v * std::abs(std::tan(delta)));
}

// Drivetrain: one traction joint (bicycle model) and one steering joint
void test_drivetrain()
{
	Sim sim("ackermann_drivetrain", "");
	auto& dt = dynamic_cast<mvsim::DynamicsAckermannDrivetrain&>(*sim.veh);
	EXPECT_NEAR(dt.driveshaftRadius(), kRadius, 1e-12);

	const double delta = mrpt::DEG2RAD(10.0);
	const double v = 1.5;
	sim.command({{"traction_joint", v / kRadius}, {"steer_joint", delta}});
	sim.run(8.0);

	const auto tr = sim.state("traction_joint");
	EXPECT_NEAR(tr.velocity, v / kRadius, 0.05 * v / kRadius);
	EXPECT_GT(std::abs(tr.effort), 0.0);
	// Driven (rear) wheels spin, with their joint position:
	EXPECT_GT(sim.state("rl_wheel_joint").position, 1.0);
	EXPECT_NEAR(sim.state("steer_joint").position, delta, 1e-9);

	const auto vel = sim.veh->getRefVelocityLocal();
	std::cout << "drivetrain: vel " << vel.asString() << "\n";
	EXPECT_NEAR(vel.vx, v, 0.1);
	EXPECT_NEAR(vel.omega, v * std::tan(delta) / kWheelbase, 0.05 * v * std::tan(delta));

	// The traction joint position is the integral of its velocity:
	const double p0 = sim.state("traction_joint").position;
	sim.run(1.0);
	const double p1 = sim.state("traction_joint").position;
	EXPECT_NEAR((p1 - p0) / 1.0, v / kRadius, 0.05 * v / kRadius);

	// Stop:
	sim.command({{"traction_joint", 0.0}});
	sim.run(5.0);
	EXPECT_NEAR(sim.veh->getRefVelocityLocal().vx, 0.0, 1e-3);
}

void test_drivetrain_effort()
{
	Sim sim("ackermann_drivetrain", "<command_interface>effort</command_interface>");
	sim.command({{"traction_joint", 100.0}});
	sim.run(2.0);
	EXPECT_NEAR(sim.state("traction_joint").effort, 100.0, 1e-9);
	EXPECT_GT(sim.veh->getRefVelocityLocal().vx, 0.1);
}

void test_determinism()
{
	const auto runOnce = []()
	{
		Sim sim("ackermann", "");
		sim.command(
			{{"rl_wheel_joint", 5.0},
			 {"custom_rr", 6.0},
			 {"fl_steer_joint", 0.2},
			 {"fr_steer_joint", 0.15}});
		sim.run(3.0);
		return sim.veh->getPose();
	};
	const auto a = runOnce();
	const auto b = runOnce();
	EXPECT_NEAR(a.x, b.x, 1e-12);
	EXPECT_NEAR(a.y, b.y, 1e-12);
	EXPECT_NEAR(a.yaw, b.yaw, 1e-12);
}
}  // namespace

int main()
{
	try
	{
		test_layouts();
		test_per_wheel_steering();
		test_steering_limits();
		test_central_steering();
		test_drivetrain();
		test_drivetrain_effort();
		test_determinism();
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
