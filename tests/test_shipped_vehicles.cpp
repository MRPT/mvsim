/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// The shipped vehicle definitions, with their own twist PID controllers, must
// follow the commanded twist: reach the commanded linear speed and turn rate
// quickly and without large overshoots, travel the distance and turn the angle
// that the command implies, and stop.
//
// Each definition is used as shipped (dynamics, wheels, masses, controller
// gains and friction), without its sensors and 3D models: a chassis shape
// taken from a (downloaded) 3D model is replaced by a box around the wheels,
// which only changes the inertia of the chassis.

#include <mrpt/math/TTwist2D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "test_utils.h"

int g_failures = 0;

namespace
{
struct ShippedVehicle
{
	std::string file;  //!< Relative to the repository root
	std::string className;
	std::string pidController;
	bool spinsInPlace;	//!< Differential vehicles
	double v;  //!< [m/s] Test linear speed
	double w;  //!< [rad/s] Test turn rate
};

const std::vector<ShippedVehicle> kVehicles = {
	{"definitions/small_robot.vehicle.xml", "small_robot", "twist_pid", true, 0.5, 0.5},
	{"definitions/turtlebot3_burger.vehicle.xml", "turtlebot3_burger", "twist_pid", true, 0.2, 1.0},
	{"definitions/jackal.vehicle.xml", "jackal", "twist_pid", true, 0.5, 0.5},
	{"definitions/jackal-ellipse.vehicle.xml", "jackal", "twist_pid", true, 0.5, 0.5},
	{"mvsim_tutorial/urdf2xml/skid_steer_4wd.vehicle.xml", "skid_steer_4wd", "twist_pid", true, 0.5,
	 0.5},
	{"mvsim_tutorial/urdf2xml/jackal_from_urdf.vehicle.xml", "jackal_from_urdf", "twist_pid", true,
	 0.5, 0.5},
	{"definitions/ackermann.vehicle.xml", "car_ackermann", "twist_front_steer_pid", false, 3.0,
	 0.2},
	{"definitions/pickup.vehicle.xml", "pickup", "twist_front_steer_pid", false, 3.0, 0.2},
	{"definitions/agricobiot2.vehicle.xml", "agricobiot2", "twist_front_steer_pid", false, 0.5,
	 0.25},
	{"definitions/scania_truck.vehicle.xml", "scania_truck", "twist_front_steer_pid", false, 3.0,
	 0.1},
};

// Acceptance criteria:
constexpr double kSteadyTol = 0.05;	 // relative error of the steady-state twist
constexpr double kMaxRiseTime = 2.0;  // [s] to reach 90% of the commanded twist
// Differential vehicles must also reach 90% of the commanded turn rate within:
constexpr double kMaxTurnRiseTime = 1.0;  // [s]
constexpr double kMaxOvershoot = 0.20;	// relative to the commanded twist
constexpr double kMaxSpinDisplacement = 0.15;  // [m] spinning in place
constexpr double kStopSpeed = 0.01;	 // [m/s] after braking
constexpr double kTrajectoryTol = 0.15;	 // relative to the commanded distance / turn

constexpr double kDt = 2e-3;  // [s] simulation time step
constexpr double kPhaseTime = 6.0;	// [s] duration of each command

std::string readFile(const std::string& path)
{
	std::ifstream f(path);
	std::stringstream ss;
	ss << f.rdbuf();
	return ss.str();
}

// The vehicle class definition, as shipped, without its sensors and 3D models:
std::string vehicleClassXml(const ShippedVehicle& sv, const std::string& controller)
{
	const std::string path = mrpt::system::pathJoin({MVSIM_TEST_DIR, "..", sv.file});
	std::string s = readFile(path);
	EXPECT_TRUE(!s.empty());

	// Only the requested class (some files define several, e.g. a trailer):
	const auto start = s.find("<vehicle:class name=\"" + sv.className + "\"");
	EXPECT_TRUE(start != std::string::npos);
	const auto end = s.find("</vehicle:class>", start);
	s = s.substr(start, end + std::string("</vehicle:class>").size() - start);

	// Include variables: the controller under test, and the defaults of the rest:
	s = std::regex_replace(s, std::regex(R"(\$\{controller\|[^}]*\})"), controller);
	s = std::regex_replace(s, std::regex(R"(\$\{[A-Za-z_][A-Za-z0-9_]*\|([^}]*)\})"), "$1");
	s = std::regex_replace(
		s, std::regex(R"(\$\{MVSIM_CURRENT_FILE_DIRECTORY\})"),
		mrpt::system::extractFileDirectory(path));

	// No sensors nor 3D models:
	s = std::regex_replace(s, std::regex(R"(<include[^>]*/>)"), "");
	s = std::regex_replace(s, std::regex(R"(<visual>[\s\S]*?</visual>)"), "");

	// Chassis shape from the (removed) 3D model: a box around the wheels.
	double maxX = 0;
	double maxY = 0;
	const std::regex posRe(R"(pos="\s*([-0-9.eE+]+)\s+([-0-9.eE+]+))");
	for (std::sregex_iterator it(s.begin(), s.end(), posRe), itEnd; it != itEnd; ++it)
	{
		maxX = std::max(maxX, std::abs(std::stod((*it)[1])));
		maxY = std::max(maxY, std::abs(std::stod((*it)[2])));
	}
	std::smatch m;
	if (std::regex_search(s, m, std::regex(R"(<f_wheels_x>\s*([-0-9.eE+]+))")))
	{
		maxX = std::max(maxX, std::abs(std::stod(m[1])));
	}
	if (std::regex_search(s, m, std::regex(R"(<f_wheels_d>\s*([-0-9.eE+]+))")))
	{
		maxY = std::max(maxY, 0.5 * std::stod(m[1]));
	}
	const double bx = maxX + 0.1;
	const double by = maxY + 0.05;
	std::ostringstream box;
	box << "<shape><pt>" << -bx << " " << -by << "</pt><pt>" << bx << " " << -by << "</pt><pt>"
		<< bx << " " << by << "</pt><pt>" << -bx << " " << by << "</pt></shape>";
	s = std::regex_replace(s, std::regex(R"(<shape_from_visual\s*/>)"), box.str());
	return s;
}

struct Sample
{
	double t;
	mrpt::math::TPose3D pose;
	mrpt::math::TTwist2D vel;  //!< Ground truth, in the vehicle frame
};

// Runs the vehicle with the given controller and commands, sampling its motion.
std::vector<Sample> simulate(
	const ShippedVehicle& sv, const std::string& controller,
	const std::vector<mrpt::math::TTwist2D>& commands)
{
	mvsim::World world;
	world.headless(true);
	world.setMinLoggingLevel(mrpt::system::LVL_WARN);
	world.load_from_XML(
		"<mvsim_world version=\"1.0\"><simul_timestep>" + std::to_string(kDt) +
		"</simul_timestep>" + vehicleClassXml(sv, controller) + "<vehicle name=\"veh\" class=\"" +
		sv.className + "\"><init_pose>0 0 0</init_pose></vehicle></mvsim_world>");
	auto veh = world.getListOfVehicles().find("veh")->second;

	std::vector<Sample> samples;
	double t = 0;
	const int n = static_cast<int>(std::round(kPhaseTime / kDt));
	for (const auto& cmd : commands)
	{
		veh->getControllerInterface()->setTwistCommand(cmd);
		for (int i = 0; i < n; i++)
		{
			world.run_simulation(kDt);
			t += kDt;
			samples.push_back({t, veh->getPose(), veh->getRefVelocityLocal()});
		}
	}
	return samples;
}

struct Response
{
	double steady;	//!< Mean over the last second of the phase
	double riseTime;  //!< To 90% of the command [s]
	double peak;
};

// Response of the linear (vx) or angular (omega) velocity in a phase:
Response response(const std::vector<Sample>& s, size_t phase, bool angular, double cmd)
{
	const size_t n = s.size() / 2;	// samples per phase (two phases per run)
	const size_t i0 = phase * n;
	const size_t i1 = i0 + n;
	const double t0 = s[i0].t;
	Response r{0, std::numeric_limits<double>::infinity(), 0};
	size_t count = 0;
	for (size_t i = i0; i < i1; i++)
	{
		const double x = (angular ? s[i].vel.omega : s[i].vel.vx) / cmd;
		r.peak = std::max(r.peak, x);
		if (x >= 0.9 && !std::isfinite(r.riseTime))
		{
			r.riseTime = s[i].t - t0;
		}
		if (s[i].t > s[i1 - 1].t - 1.0)
		{
			r.steady += x;
			count++;
		}
	}
	r.steady /= static_cast<double>(count);
	return r;
}

void checkResponse(const Response& r, const char* what, double maxRiseTime)
{
	std::cout << "    " << what << ": steady=" << r.steady << " rise90=" << r.riseTime
			  << "s peak=" << r.peak << "\n";
	EXPECT_NEAR(r.steady, 1.0, kSteadyTol);
	EXPECT_LT(r.riseTime, maxRiseTime);
	EXPECT_LT(r.peak, 1.0 + kMaxOvershoot);
}

// A command held for one phase, then a stop:
void test_twist(const ShippedVehicle& sv, const mrpt::math::TTwist2D& cmd, const char* label)
{
	std::cout << "[TEST] " << sv.file << " (" << sv.pidController << "): " << label << "\n";
	const auto s = simulate(sv, sv.pidController, {cmd, {0, 0, 0}});

	if (cmd.vx != 0)
	{
		checkResponse(response(s, 0, false, cmd.vx), "vx   ", kMaxRiseTime);
	}
	if (cmd.omega != 0)
	{
		checkResponse(
			response(s, 0, true, cmd.omega), "omega",
			sv.spinsInPlace ? kMaxTurnRiseTime : kMaxRiseTime);
	}

	// Comes to a stop:
	const auto& last = s.back().vel;
	EXPECT_LT(std::hypot(last.vx, last.vy), kStopSpeed);

	// Travelled distance and turned angle during the command, compared to the
	// command (skid-steer vehicles may also slide sideways in curves, so the
	// path is not compared point by point):
	const size_t n = s.size() / 2;
	double dist = 0;
	double turn = 0;
	for (size_t i = 0; i < n; i++)
	{
		dist += std::hypot(s[i].vel.vx, s[i].vel.vy) * kDt;
		turn += s[i].vel.omega * kDt;
	}
	const double expDist = std::abs(cmd.vx) * kPhaseTime;
	const double expTurn = cmd.omega * kPhaseTime;
	std::cout << "    trajectory: distance=" << dist << " m (command: " << expDist
			  << " m), turn=" << turn << " rad (command: " << expTurn
			  << " rad), displacement=" << std::hypot(s[n - 1].pose.x, s[n - 1].pose.y) << " m\n";
	if (cmd.vx != 0)
	{
		EXPECT_NEAR(dist, expDist, kTrajectoryTol * expDist + 0.05);
	}
	else
	{
		// Spin in place: the rotation center of a skid-steer vehicle may be
		// slightly offset from its frame, but the vehicle must not move away:
		EXPECT_LT(std::hypot(s[n - 1].pose.x, s[n - 1].pose.y), kMaxSpinDisplacement);
	}
	EXPECT_NEAR(turn, expTurn, kTrajectoryTol * std::abs(expTurn) + 0.02);
	if (cmd.omega == 0)
	{
		// Straight: no lateral drift
		EXPECT_LT(std::abs(s[n - 1].pose.y), 0.02 * expDist + 0.05);
	}
}

}  // namespace

int main()
{
	try
	{
		for (const auto& sv : kVehicles)
		{
			test_twist(sv, {sv.v, 0, 0}, "straight");
			test_twist(sv, {sv.v, 0, sv.w}, "arc");
			if (sv.spinsInPlace)
			{
				test_twist(sv, {0, 0, sv.w}, "spin in place");
			}
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
		return 1;
	}

	if (g_failures == 0)
	{
		std::printf("\nAll shipped vehicle tests passed.\n");
	}
	else
	{
		std::fprintf(stderr, "\n%d test(s) FAILED.\n", g_failures);
	}
	return g_failures == 0 ? 0 : 1;
}
