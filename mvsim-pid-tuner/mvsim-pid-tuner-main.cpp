/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// mvsim-pid-tuner: Automatic PID parameter tuning tool for MVSim vehicles.
//
// Given a vehicle definition XML file, this tool:
// 1. Runs an open-loop step response test (constant torque → wheel velocity)
// 2. Identifies the first-order plant model: G(s) = K / (τs + 1)
// 3. Proposes optimal PID parameters using IMC (Internal Model Control) tuning
// 4. Validates the proposed parameters with a closed-loop simulation

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mvsim/CsvLogger.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/World.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

// ========================================================
// Data structures for step response recording
// ========================================================
struct TimeVelSample
{
	double time = 0;
	double velocity_x = 0;	// wheel ground-contact longitudinal velocity (m/s)
};

struct PlantModel
{
	double K = 0;  // steady-state gain: (m/s) per (Nm of torque)
	double tau = 0;	 // time constant (s)
	double v_ss = 0;  // steady-state velocity (m/s)
	double torque_applied = 0;	// torque used for identification
};

struct PIDParams
{
	double KP = 0;
	double KI = 0;
	double KD = 0;
	double max_torque = 0;
};

// ========================================================
// Run open-loop step response with "raw" controller
// ========================================================
static PlantModel run_open_loop_step(
	const std::string& vehicle_xml_path, const std::string& vehicle_class, double test_torque,
	double sim_duration, double sim_step)
{
	// Load vehicle definition
	mvsim::World world;
	world.headless(true);
	world.set_gravity(9.81);
	world.set_simul_timestep(sim_step);

	// Load the vehicle:class definition by wrapping it in a minimal world XML
	{
		// default_sensors="false" to avoid requiring sensor simulation in this tool.
		std::string world_xml =
			"<mvsim_world version=\"1.0\">"
			"  <include file=\"" +
			vehicle_xml_path +
			"\" default_sensors=\"false\" />"
			"</mvsim_world>";
		world.load_from_XML(world_xml, vehicle_xml_path);
	}

	// Instantiate the vehicle from its class, then swap controller to "raw"
	std::string veh_xml = "<vehicle name=\"tuner_veh\" class=\"" + vehicle_class +
						  "\">"
						  "  <init_pose>0 0 0</init_pose>"
						  "</vehicle>";

	auto veh = mvsim::VehicleBase::factory(&world, veh_xml);

	// Replace the controller with "raw" for open-loop torque control
	if (auto* diffVeh = dynamic_cast<mvsim::DynamicsDifferential*>(veh.get()))
	{
		diffVeh->getController() =
			std::make_shared<mvsim::DynamicsDifferential::ControllerRawForces>(*diffVeh);
	}
	else if (auto* ackVeh = dynamic_cast<mvsim::DynamicsAckermann*>(veh.get()))
	{
		ackVeh->getController() =
			std::make_shared<mvsim::DynamicsAckermann::ControllerRawForces>(*ackVeh);
	}
	else
	{
		THROW_EXCEPTION("Unsupported vehicle dynamics type for PID tuning.");
	}

	world.insert_vehicle(veh);

	// Print vehicle info
	const size_t nWheels = veh->getNumWheels();
	std::printf("Vehicle: %s (%zu wheels)\n", vehicle_class.c_str(), nWheels);
	for (size_t i = 0; i < nWheels; i++)
	{
		const auto& w = veh->getWheelInfo(i);
		std::printf(
			"  Wheel %zu: pos=(%.3f, %.3f) R=%.3f mass=%.2f\n", i, w.x, w.y, w.diameter * 0.5,
			w.mass);
	}
	std::printf("  Chassis mass: %.1f kg\n", veh->getChassisMass());

	// Compute friction limit for max_torque recommendation
	// max_friction_force = mu * partial_mass * g
	// max_torque = max_friction_force * R
	double total_mass = veh->getChassisMass();
	for (size_t i = 0; i < nWheels; i++)
	{
		total_mass += veh->getWheelInfo(i).mass;
	}

	const double partial_mass = total_mass / static_cast<double>(nWheels);
	const double R = veh->getWheelInfo(0).diameter * 0.5;
	const double mu = 0.8;	// typical default
	const double max_friction_force = mu * partial_mass * 9.81;
	const double max_friction_torque = max_friction_force * R;
	std::printf(
		"  Estimated max friction torque per wheel: %.2f Nm "
		"(mu=%.1f, partial_mass=%.1f kg, R=%.3f m)\n",
		max_friction_torque, mu, partial_mass, R);

	// Choose test torque: use a fraction of friction limit if not specified
	if (test_torque <= 0)
	{
		test_torque = max_friction_torque * 0.5;
	}

	std::printf("  Test torque: %.2f Nm\n\n", test_torque);

	// Setup data recording via CSVLogger callbacks
	// We record the average velocity_x across all wheels
	std::vector<TimeVelSample> samples;
	double current_avg_vel = 0;
	int wheel_count = 0;

	for (auto& logger : veh->getLoggers())
	{
		if (!logger)
		{
			continue;
		}
		logger->setRecording(true);
		logger->setFileWritingEnabled(false);
		logger->registerOnRowCallback(
			[&current_avg_vel, &wheel_count](const std::map<std::string_view, double>& cols)
			{
				auto it = cols.find(mvsim::VehicleBase::WL_VEL_X);
				if (it != cols.end())
				{
					current_avg_vel += it->second;
					wheel_count++;
				}
			});
	}

	// Apply constant torque via raw controller
	// The raw controller sets torques directly each step
	const int steps = static_cast<int>(sim_duration / sim_step);

	for (int i = 0; i < steps; i++)
	{
		// Set torques on all wheels via the raw controller
		// For differential: set both L and R to same torque (straight)
		// Motor torque sign convention: negative torque → forward motion
		if (auto* diffVeh = dynamic_cast<mvsim::DynamicsDifferential*>(veh.get()))
		{
			auto* rawCtrl = dynamic_cast<mvsim::DynamicsDifferential::ControllerRawForces*>(
				diffVeh->getController().get());
			if (rawCtrl)
			{
				rawCtrl->setpoint_wheel_torque_l = -test_torque;
				rawCtrl->setpoint_wheel_torque_r = -test_torque;
			}
		}
		else if (auto* ackVeh = dynamic_cast<mvsim::DynamicsAckermann*>(veh.get()))
		{
			auto* rawCtrl = dynamic_cast<mvsim::DynamicsAckermann::ControllerRawForces*>(
				ackVeh->getController().get());
			if (rawCtrl)
			{
				rawCtrl->setpoint_wheel_torque_l = -test_torque;
				rawCtrl->setpoint_wheel_torque_r = -test_torque;
				rawCtrl->setpoint_steer_ang = 0;
			}
		}

		// Reset accumulators
		current_avg_vel = 0;
		wheel_count = 0;

		world.run_simulation(sim_step);

		// Record average wheel velocity
		if (wheel_count > 0)
		{
			samples.push_back({i * sim_step, current_avg_vel / wheel_count});
		}
	}

	// ========================================================
	// Plant identification: fit first-order model v(t) = v_ss * (1 - e^(-t/τ))
	// ========================================================
	// Find steady-state velocity (average of last 20% of samples)
	PlantModel plant;
	plant.torque_applied = test_torque;

	if (samples.size() < 10)
	{
		std::fprintf(stderr, "Error: Too few samples recorded (%zu)\n", samples.size());
		return plant;
	}

	const size_t ss_start = samples.size() * 80 / 100;
	double v_ss_sum = 0;
	for (size_t i = ss_start; i < samples.size(); i++)
	{
		v_ss_sum += samples[i].velocity_x;
	}
	plant.v_ss = v_ss_sum / static_cast<double>(samples.size() - ss_start);
	plant.K = plant.v_ss / test_torque;

	// Find time constant τ: time to reach 63.2% of v_ss
	const double v_63 = plant.v_ss * 0.632;
	plant.tau = sim_duration;  // fallback
	for (const auto& s : samples)
	{
		if (s.velocity_x >= v_63)
		{
			plant.tau = s.time;
			break;
		}
	}

	// Sanity check: if tau is very small (< 2*sim_step), the system responds
	// almost instantly (pure integrator with friction damping). Use a more
	// robust method: fit via least-squares on the exponential.
	if (plant.tau < 2 * sim_step)
	{
		// Use the time to reach 95% of v_ss instead, and derive τ from that
		// v(t_95) = v_ss * (1 - e^(-t_95/τ)) = 0.95 * v_ss
		// => τ = -t_95 / ln(0.05)
		const double v_95 = plant.v_ss * 0.95;
		for (const auto& s : samples)
		{
			if (s.velocity_x >= v_95)
			{
				plant.tau = -s.time / std::log(0.05);
				break;
			}
		}
	}

	std::printf("=== Open-Loop Step Response Analysis ===\n");
	std::printf("Applied torque:        %.3f Nm\n", test_torque);
	std::printf("Steady-state velocity: %.4f m/s\n", plant.v_ss);
	std::printf("Plant gain K:          %.4f (m/s)/Nm\n", plant.K);
	std::printf("Time constant τ:       %.4f s\n", plant.tau);

	return plant;
}

// ========================================================
// Compute PID parameters using IMC tuning
// ========================================================
static PIDParams compute_pid_imc(
	const PlantModel& plant, double max_friction_torque, double aggressiveness)
{
	PIDParams pid;

	// IMC tuning for first-order plant G(s) = K / (τs + 1):
	//   Desired closed-loop time constant: τ_cl = τ * aggressiveness
	//   (smaller aggressiveness = faster response)
	//
	//   PI controller: C(s) = KP * (1 + 1/(Ti*s))
	//     KP = τ / (K * τ_cl)
	//     Ti = τ  (integral time = plant time constant → pole-zero cancellation)
	//     KI = KP / Ti = 1 / (K * τ_cl)

	const double tau_cl = plant.tau * aggressiveness;

	pid.KP = plant.tau / (plant.K * tau_cl);
	pid.KI = pid.KP / plant.tau;  // = 1 / (K * tau_cl)
	pid.KD = 0;	 // not needed for first-order plants

	// max_torque: set to ~80% of friction limit to avoid wheel slip,
	// but at least enough for the PID to be effective
	pid.max_torque = max_friction_torque * 0.8;

	return pid;
}

// ========================================================
// Validate proposed PID with closed-loop simulation
// ========================================================
struct ValidationResult
{
	double rise_time = 0;  // time to reach 90% of setpoint
	double settling_time = 0;  // time to reach and stay within 2% of setpoint
	double overshoot_pct = 0;  // percent overshoot
	double steady_state_error = 0;
	double stop_settling_time = 0;	// time to stop after step-down
	double stop_overshoot = 0;	// velocity undershoot during stop (should be ~0)
};

static ValidationResult validate_pid(
	const std::string& vehicle_xml_path, const std::string& vehicle_class, const PIDParams& pid,
	double sim_step)
{
	ValidationResult result;

	mvsim::World world;
	world.headless(true);
	world.set_gravity(9.81);
	world.set_simul_timestep(sim_step);

	{
		std::string world_xml =
			"<mvsim_world version=\"1.0\">"
			"  <include file=\"" +
			vehicle_xml_path +
			"\" default_sensors=\"false\" />"
			"</mvsim_world>";
		world.load_from_XML(world_xml, vehicle_xml_path);
	}

	// Create vehicle from class definition (inherits its controller)
	std::string veh_xml = "<vehicle name=\"tuner_veh\" class=\"" + vehicle_class +
						  "\">"
						  "  <init_pose>0 0 0</init_pose>"
						  "</vehicle>";

	auto veh = mvsim::VehicleBase::factory(&world, veh_xml);

	// Programmatically set the proposed PID parameters on the controller
	if (auto* diffVeh = dynamic_cast<mvsim::DynamicsDifferential*>(veh.get()))
	{
		auto pidCtrl = std::make_shared<mvsim::DynamicsDifferential::ControllerTwistPID>(*diffVeh);
		pidCtrl->KP = pid.KP;
		pidCtrl->KI = pid.KI;
		pidCtrl->KD = pid.KD;
		pidCtrl->max_torque = pid.max_torque;
		diffVeh->getController() = pidCtrl;
	}
	else if (auto* ackVeh = dynamic_cast<mvsim::DynamicsAckermann*>(veh.get()))
	{
		auto pidCtrl =
			std::make_shared<mvsim::DynamicsAckermann::ControllerTwistFrontSteerPID>(*ackVeh);
		pidCtrl->KP = pid.KP;
		pidCtrl->KI = pid.KI;
		pidCtrl->KD = pid.KD;
		pidCtrl->max_torque = pid.max_torque;
		ackVeh->getController() = pidCtrl;
	}

	world.insert_vehicle(veh);

	// Record wheel velocity
	std::vector<TimeVelSample> samples;
	double current_avg_vel = 0;
	int wheel_count = 0;

	for (auto& logger : veh->getLoggers())
	{
		if (!logger)
		{
			continue;
		}
		logger->setRecording(true);
		logger->setFileWritingEnabled(false);
		logger->registerOnRowCallback(
			[&current_avg_vel, &wheel_count](const std::map<std::string_view, double>& cols)
			{
				auto it = cols.find(mvsim::VehicleBase::WL_VEL_X);
				if (it != cols.end())
				{
					current_avg_vel += it->second;
					wheel_count++;
				}
			});
	}

	// Phase 1: Step-up to 1.0 m/s for 3 seconds
	const double setpoint = 1.0;
	const double step_up_time = 3.0;
	const double step_down_time = 3.0;
	const mrpt::math::TTwist2D cmd_forward(setpoint, 0, 0);
	const mrpt::math::TTwist2D cmd_stop(0, 0, 0);

	veh->getControllerInterface()->setTwistCommand(cmd_forward);

	int total_steps = static_cast<int>((step_up_time + step_down_time) / sim_step);
	int step_down_at = static_cast<int>(step_up_time / sim_step);
	double max_vel = 0;

	for (int i = 0; i < total_steps; i++)
	{
		if (i == step_down_at)
		{
			veh->getControllerInterface()->setTwistCommand(cmd_stop);
		}

		current_avg_vel = 0;
		wheel_count = 0;
		world.run_simulation(sim_step);

		double avg_v = (wheel_count > 0) ? current_avg_vel / wheel_count : 0;
		samples.push_back({i * sim_step, avg_v});

		if (i < step_down_at)
		{
			max_vel = std::max(max_vel, avg_v);
		}
	}

	// Analyze step-up response
	// Rise time: time to reach 90% of setpoint
	result.rise_time = step_up_time;  // fallback: never reached
	for (const auto& s : samples)
	{
		if (s.time > step_up_time)
		{
			break;
		}
		if (s.velocity_x >= 0.9 * setpoint)
		{
			result.rise_time = s.time;
			break;
		}
	}

	// Overshoot
	result.overshoot_pct = std::max(0.0, (max_vel - setpoint) / setpoint * 100.0);

	// Steady-state error (average of last 20% before step-down)
	{
		const size_t ss_start = step_down_at * 80 / 100;
		double ss_sum = 0;
		int ss_count = 0;
		for (size_t i = ss_start; i < static_cast<size_t>(step_down_at); i++)
		{
			ss_sum += samples[i].velocity_x;
			ss_count++;
		}
		if (ss_count > 0)
		{
			result.steady_state_error = std::abs(setpoint - ss_sum / ss_count);
		}
	}

	// Settling time during step-up: time to enter and stay within 2% band
	{
		const double band = 0.02 * setpoint;
		result.settling_time = step_up_time;  // fallback
		// Scan backwards from step_down to find last exit from band
		for (int i = step_down_at - 1; i >= 0; i--)
		{
			if (std::abs(samples[i].velocity_x - setpoint) > band)
			{
				if (i + 1 < step_down_at)
				{
					result.settling_time = samples[i + 1].time;
				}
				break;
			}
		}
	}

	// Analyze step-down (stop) response
	{
		// Find settling time to reach <2% of setpoint
		const double stop_band = 0.02 * setpoint;
		result.stop_settling_time = step_down_time;
		double min_vel = setpoint;
		for (size_t i = step_down_at; i < samples.size(); i++)
		{
			min_vel = std::min(min_vel, samples[i].velocity_x);
			if (std::abs(samples[i].velocity_x) <= stop_band)
			{
				result.stop_settling_time = samples[i].time - step_up_time;
				break;
			}
		}
		result.stop_overshoot = std::max(0.0, -min_vel);  // negative velocity = rebound
	}

	return result;
}

// ========================================================
// Main
// ========================================================
int main(int argc, char** argv)
{
	try
	{
		TCLAP::CmdLine cmd("mvsim-pid-tuner: Automatic PID tuning for MVSim vehicles", ' ');

		TCLAP::UnlabeledValueArg<std::string> argVehicleXml(
			"vehicle_xml", "Path to vehicle definition XML file (e.g. small_robot.vehicle.xml)",
			true, "", "vehicle.xml", cmd);

		TCLAP::UnlabeledValueArg<std::string> argVehicleClass(
			"vehicle_class", "Vehicle class name as defined in the XML (e.g. small_robot)", true,
			"", "class_name", cmd);

		TCLAP::ValueArg<double> argTorque(
			"t", "torque",
			"Test torque (Nm) for open-loop identification. "
			"Default: 50% of estimated friction limit.",
			false, -1.0, "Nm", cmd);

		TCLAP::ValueArg<double> argDuration(
			"d", "duration", "Open-loop step duration (s). Default: 5.0", false, 5.0, "seconds",
			cmd);

		TCLAP::ValueArg<double> argStep(
			"s", "sim-step", "Simulation time step (s). Default: 0.001", false, 0.001, "seconds",
			cmd);

		TCLAP::ValueArg<double> argAggressiveness(
			"a", "aggressiveness",
			"Closed-loop aggressiveness factor (0.1=very aggressive, "
			"1.0=conservative). Default: 0.25",
			false, 0.25, "factor", cmd);

		if (!cmd.parse(argc, argv))
		{
			return 0;  // Help/version was printed
		}

		const std::string vehicle_xml = argVehicleXml.getValue();
		const std::string vehicle_class = argVehicleClass.getValue();
		const double test_torque = argTorque.getValue();
		const double sim_duration = argDuration.getValue();
		const double sim_step = argStep.getValue();
		const double aggressiveness = argAggressiveness.getValue();

		if (aggressiveness <= 0)
		{
			std::fprintf(stderr, "Error: aggressiveness must be > 0.\n");
			return 1;
		}

		std::printf(
			"======================================\n"
			" MVSim PID Auto-Tuner\n"
			"======================================\n\n");

		// Phase 1: Open-loop identification
		std::printf("--- Phase 1: Open-loop plant identification ---\n");
		PlantModel plant =
			run_open_loop_step(vehicle_xml, vehicle_class, test_torque, sim_duration, sim_step);

		if (plant.K <= 0 || plant.tau <= 0)
		{
			std::fprintf(stderr, "\nError: Could not identify plant model. Check vehicle XML.\n");
			return 1;
		}

		// Compute friction limit for max_torque
		// Re-derive from plant data
		// We can estimate: max_friction_torque ≈ v_ss / K_velocity_per_force * R
		// But simpler: just use the same estimation as in the open-loop test
		// For that we need vehicle params again. Let's just load them.
		mvsim::World tmpWorld;
		tmpWorld.headless(true);
		tmpWorld.set_gravity(9.81);
		tmpWorld.set_simul_timestep(sim_step);
		{
			std::string world_xml =
				"<mvsim_world version=\"1.0\">"
				"  <include file=\"" +
				vehicle_xml +
				"\" default_sensors=\"false\" />"
				"</mvsim_world>";
			tmpWorld.load_from_XML(world_xml, vehicle_xml);
		}

		auto tmpVeh = mvsim::VehicleBase::factory(
			&tmpWorld, "<vehicle name=\"tmp\" class=\"" + vehicle_class +
						   "\">"
						   "  <init_pose>0 0 0</init_pose>"
						   "</vehicle>");

		double total_mass = tmpVeh->getChassisMass();
		for (size_t i = 0; i < tmpVeh->getNumWheels(); i++)
		{
			total_mass += tmpVeh->getWheelInfo(i).mass;
		}

		const double partial_mass = total_mass / static_cast<double>(tmpVeh->getNumWheels());
		const double R = tmpVeh->getWheelInfo(0).diameter * 0.5;
		const double mu = 0.8;
		const double max_friction_torque = mu * partial_mass * 9.81 * R;

		// Phase 2: Compute PID parameters
		std::printf("\n--- Phase 2: IMC PID tuning (aggressiveness=%.2f) ---\n", aggressiveness);

		PIDParams pid = compute_pid_imc(plant, max_friction_torque, aggressiveness);

		std::printf("\nProposed PID parameters:\n");
		std::printf("  KP:         %.4f\n", pid.KP);
		std::printf("  KI:         %.4f\n", pid.KI);
		std::printf("  KD:         %.4f\n", pid.KD);
		std::printf("  max_torque: %.2f Nm\n", pid.max_torque);
		std::printf("\nXML snippet:\n");
		std::printf(
			"  <KP>%.4f</KP>\n"
			"  <KI>%.4f</KI>\n"
			"  <KD>%.4f</KD>\n"
			"  <max_torque>%.2f</max_torque>\n",
			pid.KP, pid.KI, pid.KD, pid.max_torque);

		// Phase 3: Validate with closed-loop simulation
		std::printf("\n--- Phase 3: Closed-loop validation ---\n");
		std::printf("Running step-up (1.0 m/s, 3s) then step-down (stop, 3s)...\n");

		ValidationResult val = validate_pid(vehicle_xml, vehicle_class, pid, sim_step);

		std::printf("\nStep-up (0 → 1.0 m/s):\n");
		std::printf("  Rise time (90%%):       %.3f s\n", val.rise_time);
		std::printf("  Settling time (2%%):    %.3f s\n", val.settling_time);
		std::printf("  Overshoot:             %.1f %%\n", val.overshoot_pct);
		std::printf("  Steady-state error:    %.4f m/s\n", val.steady_state_error);

		std::printf("\nStep-down (1.0 → 0 m/s):\n");
		std::printf("  Settling time (2%%):    %.3f s\n", val.stop_settling_time);
		if (val.stop_overshoot > 0.001)
		{
			std::printf(
				"  Rebound (neg velocity): %.4f m/s  *** WARNING: vehicle reverses!\n",
				val.stop_overshoot);
		}
		else
		{
			std::printf("  Rebound:               none\n");
		}

		// Quality assessment
		std::printf("\n--- Assessment ---\n");
		bool good = true;
		if (val.overshoot_pct > 20)
		{
			std::printf(
				"WARNING: High overshoot (%.1f%%). Consider increasing aggressiveness factor.\n",
				val.overshoot_pct);
			good = false;
		}
		if (val.steady_state_error > 0.05)
		{
			std::printf(
				"WARNING: Large steady-state error (%.4f). KI may need to be higher.\n",
				val.steady_state_error);
			good = false;
		}
		if (val.stop_overshoot > 0.01)
		{
			std::printf(
				"WARNING: Vehicle rebounds when stopping. KI may be too high "
				"or max_torque too large.\n");
			good = false;
		}
		if (val.rise_time > 2.0)
		{
			std::printf(
				"WARNING: Slow rise time (%.1fs). Consider decreasing aggressiveness factor.\n",
				val.rise_time);
			good = false;
		}
		if (good)
		{
			std::printf("All metrics look good!\n");
		}

		std::printf("\n");
		return 0;
	}
	catch (const std::exception& e)
	{
		std::fprintf(stderr, "Error: %s\n", e.what());
		return 1;
	}
}
