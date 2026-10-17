/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// mvsim-pid-tuner: tunes the twist PID controller of a vehicle definition.
//
// Each candidate set of gains drives the vehicle (in an empty world) through
// twist command profiles (stop-and-go, S-curve, forward/backward, and spinning
// in place for differential robots), at the requested time steps and also on a
// low-friction ground. The cost is the velocity tracking error (integral of
// the absolute error of the actual chassis velocity), penalizing overshoot,
// vehicles that do not stop, and instability. The best gains of a coarse grid
// are then refined around.

#include <mrpt/core/exceptions.h>
#include <mrpt/math/TTwist2D.h>
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/World.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
struct Gains
{
	double KP = 0;
	double KI = 0;
	double max_torque = 0;
	double separation_multiplier = 1.0;	 // skid-steer robots only
};

struct Segment
{
	double duration;
	double vx;
	double w;
};

struct Profile
{
	std::string name;
	std::vector<Segment> segments;
};

struct Condition
{
	double dt;
	std::optional<double> mu;  // ground friction override
};

struct RunResult
{
	double iaeVx = 0;  // integral of |vx - vx_cmd|
	double iaeW = 0;  // integral of |w - w_cmd|
	double overshoot = 0;  // max relative overshoot in the vx steps
	double endSpeed = 0;  // |v| + |w|*halfLength at the end (stopped?)
	bool valid = true;	// false on NaN or exceptions
};

struct Evaluation
{
	Gains gains;
	double cost = std::numeric_limits<double>::infinity();
	double trackingCost = 0;
	double worstOvershoot = 0;
	double worstEndSpeed = 0;
};

struct VehicleInfo
{
	std::string dynClass;  // "differential", "ackermann" or "ackermann_drivetrain"
	size_t numWheels = 0;
	double halfLength = 0;	// half track (differential) or wheelbase (Ackermann) [m]
	double minTurnRadius = 0;  // Ackermann only [m]
	double xmlKP = 0;
	double xmlKI = 0;
	double xmlMaxTorque = 0;
	double xmlSeparationMultiplier = 1.0;
	bool xmlHasTwistPid = false;
	double kpEstimate = 0;	// [N.m/(m/s)]
	double tractionTorque = 0;	// per driven wheel [N.m]
};

// World loading touches shared caches: done one at a time.
std::mutex worldLoadMtx;

std::string worldXml(
	const std::string& vehicleXmlFile, const std::string& vehicleClass, double dt,
	const std::optional<double>& mu)
{
	std::ostringstream ss;
	ss << "<mvsim_world version=\"1.0\">\n"
	   << "<simul_timestep>" << dt << "</simul_timestep>\n"
	   << "<include file=\"" << vehicleXmlFile
	   << "\" default_sensors=\"false\" headlights=\"false\" />\n";
	if (mu)
	{
		ss << "<element class=\"property_region\">"
		   << "<property_name>friction_mu</property_name>"
		   << "<value_double>" << *mu << "</value_double>"
		   << "<x_min>-1e4</x_min><y_min>-1e4</y_min><z_min>-1e4</z_min>"
		   << "<x_max>1e4</x_max><y_max>1e4</y_max><z_max>1e4</z_max>"
		   << "</element>\n";
	}
	ss << "<vehicle name=\"veh\" class=\"" << vehicleClass
	   << "\"><init_pose>0 0 0</init_pose></vehicle>\n"
	   << "</mvsim_world>\n";
	return ss.str();
}

std::shared_ptr<mvsim::World> loadWorld(
	const std::string& vehicleXmlFile, const std::string& vehicleClass, double dt,
	const std::optional<double>& mu)
{
	auto world = std::make_shared<mvsim::World>();
	world->headless(true);
	world->setMinLoggingLevel(mrpt::system::LVL_ERROR);
	std::lock_guard<std::mutex> lck(worldLoadMtx);
	world->load_from_XML(worldXml(vehicleXmlFile, vehicleClass, dt, mu), vehicleXmlFile);
	return world;
}

mvsim::VehicleBase::Ptr theVehicle(mvsim::World& world)
{
	auto& vehs = world.getListOfVehicles();
	ASSERT_(!vehs.empty());
	return vehs.begin()->second;
}

// Installs (if needed) the twist PID controller and sets its gains (if g is
// given). Fills info with the controller defined in the XML (if given).
template <class VEH, class CTRL>
bool setupPid(mvsim::VehicleBase& v, const Gains* g, VehicleInfo* info)
{
	auto* veh = dynamic_cast<VEH*>(&v);
	if (!veh)
	{
		return false;
	}
	auto ctrl = std::dynamic_pointer_cast<CTRL>(veh->getController());
	if (info)
	{
		info->xmlHasTwistPid = static_cast<bool>(ctrl);
		if (ctrl)
		{
			info->xmlKP = ctrl->KP;
			info->xmlKI = ctrl->KI;
			info->xmlMaxTorque = ctrl->max_torque;
		}
	}
	if (!g)
	{
		return true;
	}
	if (!ctrl)
	{
		ctrl = std::make_shared<CTRL>(*veh);
		veh->getController() = ctrl;
	}
	ctrl->KP = g->KP;
	ctrl->KI = g->KI;
	ctrl->KD = 0;
	ctrl->max_torque = g->max_torque;
	return true;
}

bool setupController(mvsim::VehicleBase& v, const Gains* g, VehicleInfo* info)
{
	using namespace mvsim;
	if (setupPid<DynamicsDifferential, DynamicsDifferential::ControllerTwistPID>(v, g, info))
	{
		auto& veh = dynamic_cast<DynamicsDifferential&>(v);
		auto ctrl = std::dynamic_pointer_cast<DynamicsDifferential::ControllerTwistPID>(
			veh.getController());
		if (info && ctrl)
		{
			info->xmlSeparationMultiplier = ctrl->wheel_separation_multiplier;
		}
		if (g && ctrl)
		{
			ctrl->wheel_separation_multiplier = g->separation_multiplier;
		}
		return true;
	}
	return setupPid<DynamicsAckermann, DynamicsAckermann::ControllerTwistFrontSteerPID>(
			   v, g, info) ||
		   setupPid<
			   DynamicsAckermannDrivetrain,
			   DynamicsAckermannDrivetrain::ControllerTwistFrontSteerPID>(v, g, info);
}

RunResult runProfile(
	const std::string& vehicleXmlFile, const std::string& vehicleClass, const Gains& g,
	const Profile& profile, const Condition& cond, double halfLength)
{
	RunResult res;
	try
	{
		auto world = loadWorld(vehicleXmlFile, vehicleClass, cond.dt, cond.mu);
		auto veh = theVehicle(*world);
		setupController(*veh, &g, nullptr);
		auto* ctrl = veh->getControllerInterface();

		for (const auto& s : profile.segments)
		{
			ctrl->setTwistCommand(mrpt::math::TTwist2D(s.vx, 0, s.w));
			const double step = s.vx - veh->getRefVelocityLocal().vx;
			const int n = static_cast<int>(std::round(s.duration / cond.dt));
			for (int k = 0; k < n; k++)
			{
				world->run_simulation(cond.dt);
				const auto vel = veh->getRefVelocityLocal();
				if (!std::isfinite(vel.vx) || !std::isfinite(vel.omega))
				{
					res.valid = false;
					return res;
				}
				res.iaeVx += std::abs(vel.vx - s.vx) * cond.dt;
				res.iaeW += std::abs(vel.omega - s.w) * cond.dt;
				if (std::abs(step) > 0.05)
				{
					res.overshoot = std::max(res.overshoot, (vel.vx - s.vx) / step);
				}
			}
		}
		const auto vel = veh->getRefVelocityLocal();
		res.endSpeed = std::hypot(vel.vx, vel.vy) + std::abs(vel.omega) * halfLength;
	}
	catch (const std::exception&)
	{
		res.valid = false;
	}
	return res;
}

Evaluation evaluate(
	const std::string& xmlFile, const std::string& cls, const Gains& g,
	const std::vector<Profile>& profiles, const std::vector<Condition>& conditions,
	double halfLength)
{
	Evaluation ev;
	ev.gains = g;
	for (const auto& cond : conditions)
	{
		for (const auto& p : profiles)
		{
			const auto r = runProfile(xmlFile, cls, g, p, cond, halfLength);
			if (!r.valid)
			{
				return ev;	// infinite cost
			}
			ev.trackingCost += r.iaeVx + halfLength * r.iaeW;
			ev.worstOvershoot = std::max(ev.worstOvershoot, r.overshoot);
			ev.worstEndSpeed = std::max(ev.worstEndSpeed, r.endSpeed);
		}
	}
	// Penalties: overshoot above 5%, and not coming to a full stop
	ev.cost = ev.trackingCost * (1.0 + 10.0 * std::max(0.0, ev.worstOvershoot - 0.05));
	if (ev.worstEndSpeed > 1e-3)
	{
		ev.cost += 1e3;
	}
	return ev;
}

std::vector<Evaluation> evaluateAll(
	const std::vector<Gains>& candidates, const std::string& xmlFile, const std::string& cls,
	const std::vector<Profile>& profiles, const std::vector<Condition>& conditions,
	double halfLength, unsigned numThreads)
{
	std::vector<Evaluation> results(candidates.size());
	std::atomic<size_t> next{0};
	std::atomic<size_t> done{0};
	std::mutex printMtx;
	const auto worker = [&]()
	{
		for (size_t i = next++; i < candidates.size(); i = next++)
		{
			results[i] = evaluate(xmlFile, cls, candidates[i], profiles, conditions, halfLength);
			std::lock_guard<std::mutex> lck(printMtx);
			std::printf("\r  Evaluated %zu/%zu", ++done, candidates.size());
			std::fflush(stdout);
		}
	};
	std::vector<std::thread> threads;
	for (unsigned t = 0; t < std::max(1u, numThreads); t++)
	{
		threads.emplace_back(worker);
	}
	for (auto& t : threads)
	{
		t.join();
	}
	std::printf("\n");
	std::sort(
		results.begin(), results.end(),
		[](const Evaluation& a, const Evaluation& b) { return a.cost < b.cost; });
	return results;
}

VehicleInfo inspectVehicle(const std::string& xmlFile, const std::string& cls, double dt)
{
	VehicleInfo info;
	auto world = loadWorld(xmlFile, cls, dt, std::nullopt);
	auto veh = theVehicle(*world);
	if (!setupController(*veh, nullptr, &info))
	{
		THROW_EXCEPTION(
			"Unsupported vehicle dynamics: differential, ackermann and "
			"ackermann_drivetrain are supported.");
	}
	info.numWheels = veh->getNumWheels();
	world->run_simulation(dt);	// computes the wheel loads

	std::vector<size_t> driven;
	if (dynamic_cast<mvsim::DynamicsDifferential*>(veh.get()))
	{
		info.dynClass = "differential";
		info.halfLength = 0.5 * std::abs(veh->getWheelInfo(0).y - veh->getWheelInfo(1).y);
		driven = {0, 1};
		if (info.numWheels == 4)
		{
			driven = {0, 1, 2, 3};
		}
	}
	else
	{
		double maxSteer = 0;
		if (auto* a = dynamic_cast<mvsim::DynamicsAckermann*>(veh.get()); a)
		{
			info.dynClass = "ackermann";
			maxSteer = a->getMaxSteeringAngle();
			driven = {2, 3};  // front wheels
		}
		else
		{
			auto& b = dynamic_cast<mvsim::DynamicsAckermannDrivetrain&>(*veh);
			info.dynClass = "ackermann_drivetrain";
			maxSteer = b.getMaxSteeringAngle();
			driven = {0, 1, 2, 3};
		}
		const double L = veh->getWheelInfo(2).x - veh->getWheelInfo(0).x;
		info.halfLength = 0.5 * L;
		info.minTurnRadius = L / std::tan(std::max(1e-3, maxSteer));
	}

	// Traction limit of the driven wheels, and a first estimate of KP (a
	// closed-loop time constant of ~0.1 s for the vehicle mass share of each
	// driven wheel):
	const auto& loads = veh->getWheelLoads();
	double minTraction = std::numeric_limits<double>::infinity();
	for (const size_t i : driven)
	{
		const double mu = veh->getFrictionModels().at(i)->peakFrictionCoefficient();
		if (loads.at(i) > 0)
		{
			minTraction =
				std::min(minTraction, mu * loads.at(i) * 0.5 * veh->getWheelInfo(i).diameter);
		}
	}
	info.tractionTorque = minTraction;
	const double mass = veh->getBox2DChassisBody()->GetMass();
	const double R = 0.5 * veh->getWheelInfo(driven.front()).diameter;
	info.kpEstimate = R * mass / static_cast<double>(driven.size()) / 0.1;
	return info;
}

std::vector<Profile> makeProfiles(const VehicleInfo& info, double v, double radius, double spinRate)
{
	const double w = v / radius;
	const double tq = 0.25 * 2 * M_PI / w;	// quarter of a turn
	std::vector<Profile> profiles = {
		{"stop_go",
		 {{3.0, v, 0}, {2.0, 0, 0}, {3.0, v, 0}, {2.0, 0, 0}, {3.0, -v, 0}, {3.0, 0, 0}}},
		{"s_curve",
		 {{2.0, v, 0}, {tq, v, w}, {2 * tq, v, -w}, {tq, v, w}, {2.0, v, 0}, {3.0, 0, 0}}},
		{"fwd_back", {{4.0, v, 0}, {4.0, -v, 0}, {3.0, 0, 0}}}};
	if (info.dynClass == "differential")
	{
		profiles.push_back({"spin", {{2 * M_PI / spinRate, 0, spinRate}, {3.0, 0, 0}}});
	}
	return profiles;
}

void printEvaluation(const char* label, const Evaluation& e, bool skidSteer)
{
	std::printf(
		"  %-9s KP=%-9.4g KI=%-9.4g max_torque=%-8.4g", label, e.gains.KP, e.gains.KI,
		e.gains.max_torque);
	if (skidSteer)
	{
		std::printf(" sep_mult=%-5.3g", e.gains.separation_multiplier);
	}
	if (!std::isfinite(e.cost))
	{
		std::printf(" | UNSTABLE\n");
		return;
	}
	std::printf(
		" | cost=%-8.4g tracking=%-8.4g overshoot=%4.1f%%%s\n", e.cost, e.trackingCost,
		100 * e.worstOvershoot, e.worstEndSpeed > 1e-3 ? " DOES NOT STOP" : "");
}

std::vector<double> parseList(const std::string& s)
{
	std::vector<double> out;
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, ','))
	{
		out.push_back(std::stod(item));
	}
	return out;
}

}  // namespace

int main(int argc, char** argv)
{
	try
	{
		CLI::App app(
			"mvsim-pid-tuner: tunes the twist PID controller of an MVSim vehicle definition");

		std::string xmlFile;
		app.add_option("vehicle_xml", xmlFile, "Vehicle definition XML file")->required();
		std::string cls;
		app.add_option("vehicle_class", cls, "Vehicle class name, as defined in the XML")
			->required();

		double speed = 1.0;
		app.add_option("-v,--speed", speed, "Linear speed of the test profiles [m/s]");
		double radius = 0;
		app.add_option(
			"-r,--radius", radius,
			"Turn radius of the S-curve profile [m]. Default: 2 times the minimum turn radius "
			"(Ackermann) or 1.5 m (differential).");
		double spinRate = 1.0;
		app.add_option(
			"--spin-rate", spinRate, "Angular speed of the spin profile (differential) [rad/s]");
		std::string dts = "0.001,0.005";
		app.add_option(
			"-s,--sim-steps", dts, "Comma-separated simulation time steps to evaluate [s]");
		double lowMu = 0.3;
		app.add_option(
			"--low-mu", lowMu,
			"Friction coefficient of the extra low-friction evaluation (0: disabled)");
		double maxTorque = 0;
		app.add_option(
			"--max-torque", maxTorque,
			"Fixed max_torque [N.m]. Default: the one of the XML twist PID controller, or 90% of "
			"the traction limit if there is none (3 times it for skid-steer robots).");
		unsigned numThreads = std::max(1u, std::thread::hardware_concurrency());
		app.add_option("-j,--threads", numThreads, "Number of parallel threads");

		CLI11_PARSE(app, argc, argv);

		const std::vector<double> simSteps = parseList(dts);
		ASSERT_(!simSteps.empty());

		std::printf("Vehicle '%s' from '%s'\n", cls.c_str(), xmlFile.c_str());
		const VehicleInfo info = inspectVehicle(xmlFile, cls, simSteps.front());
		const bool skidSteer = info.dynClass == "differential" && info.numWheels == 4;

		if (radius <= 0)
		{
			radius = info.dynClass == "differential" ? 1.5 : 2.0 * info.minTurnRadius;
		}
		if (maxTorque <= 0)
		{
			maxTorque = info.xmlHasTwistPid && info.xmlMaxTorque > 0
							? info.xmlMaxTorque
							: (skidSteer ? 3.0 : 0.9) * info.tractionTorque;
		}
		if (!std::isfinite(maxTorque))
		{
			THROW_EXCEPTION("No driven wheel has a load: set max_torque with --max-torque.");
		}

		std::printf(
			"  Dynamics: %s, %zu wheels. Traction limit per driven wheel: %.4g N.m\n",
			info.dynClass.c_str(), info.numWheels, info.tractionTorque);
		std::printf(
			"  Profiles: speed=%.3g m/s, turn radius=%.3g m%s. max_torque=%.4g N.m\n", speed,
			radius, info.dynClass == "differential" ? ", spin" : "", maxTorque);

		const auto profiles = makeProfiles(info, speed, radius, spinRate);
		std::vector<Condition> conditions;
		for (const double dt : simSteps)
		{
			conditions.push_back({dt, std::nullopt});
		}
		if (lowMu > 0)
		{
			conditions.push_back({*std::max_element(simSteps.begin(), simSteps.end()), lowMu});
		}

		// Coarse grid:
		std::vector<Gains> candidates;
		const std::vector<double> sepMults =
			skidSteer ? std::vector<double>{1.0, 1.4, 1.8, 2.2} : std::vector<double>{1.0};
		for (int i = -2; i <= 5; i++)
		{
			const double kp = info.kpEstimate * std::pow(2.0, i);
			for (const double kiRatio : {0.0, 0.02, 0.1, 0.5, 2.5, 12.5, 62.5})
			{
				for (const double sm : sepMults)
				{
					candidates.push_back({kp, kp * kiRatio, maxTorque, sm});
				}
			}
		}
		std::printf("\nCoarse search (%zu candidates):\n", candidates.size());
		auto results = evaluateAll(
			candidates, xmlFile, cls, profiles, conditions, info.halfLength, numThreads);

		if (!std::isfinite(results.front().cost))
		{
			THROW_EXCEPTION(
				"All candidates were unstable: check max_torque and the vehicle definition.");
		}

		// Refinement around the best one:
		const Gains best0 = results.front().gains;
		candidates.clear();
		const double f = std::sqrt(2.0);
		const std::vector<double> sepFactors =
			skidSteer ? std::vector<double>{0.9, 1.0, 1.1} : std::vector<double>{1.0};
		const std::vector<double> kiFactors =
			best0.KI > 0 ? std::vector<double>{0.5, 1 / f, 1.0, f, 2.0} : std::vector<double>{1.0};
		for (const double kpf : {1 / f, 1.0, f})
		{
			for (const double kif : kiFactors)
			{
				for (const double smf : sepFactors)
				{
					candidates.push_back(
						{best0.KP * kpf, best0.KI * kif, maxTorque,
						 best0.separation_multiplier * smf});
				}
			}
		}
		std::printf("Refinement (%zu candidates):\n", candidates.size());
		const auto refined = evaluateAll(
			candidates, xmlFile, cls, profiles, conditions, info.halfLength, numThreads);
		results.insert(results.end(), refined.begin(), refined.end());
		std::sort(
			results.begin(), results.end(),
			[](const Evaluation& a, const Evaluation& b) { return a.cost < b.cost; });

		// Among the candidates within 2% of the best cost, prefer the smallest
		// gains (tracking is often limited by max_torque, so larger gains only
		// make the controller stiffer):
		size_t selected = 0;
		for (size_t i = 0; i < results.size(); i++)
		{
			if (results[i].cost <= 1.02 * results.front().cost &&
				results[i].gains.KP < results[selected].gains.KP)
			{
				selected = i;
			}
		}

		std::printf("\nBest candidates:\n");
		for (size_t i = 0; i < std::min<size_t>(5, results.size()); i++)
		{
			printEvaluation("", results[i], skidSteer);
		}
		printEvaluation("SELECTED", results[selected], skidSteer);
		if (info.xmlHasTwistPid)
		{
			const Gains xmlGains{
				info.xmlKP, info.xmlKI, info.xmlMaxTorque, info.xmlSeparationMultiplier};
			printEvaluation(
				"XML", evaluate(xmlFile, cls, xmlGains, profiles, conditions, info.halfLength),
				skidSteer);
		}

		const auto& best = results[selected].gains;
		std::printf("\nXML snippet:\n");
		std::printf(
			"  <KP>%.4g</KP>\n  <KI>%.4g</KI>\n  <KD>0</KD>\n  <max_torque>%.4g</max_torque>\n",
			best.KP, best.KI, best.max_torque);
		if (skidSteer)
		{
			std::printf(
				"  <wheel_separation_multiplier>%.3g</wheel_separation_multiplier>\n",
				best.separation_multiplier);
		}
		return 0;
	}
	catch (const std::exception& e)
	{
		std::fprintf(stderr, "Error: %s\n", e.what());
		return 1;
	}
}
