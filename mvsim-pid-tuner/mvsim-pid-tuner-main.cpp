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
// 1. Runs open-loop step response tests (linear and rotational)
// 2. Identifies the first-order plant models: G(s) = K / (τs + 1)
// 3. Proposes optimal PID parameters using IMC (Internal Model Control) tuning
// 4. Validates the proposed parameters with closed-loop simulations

#include <mrpt/core/exceptions.h>
#include <mrpt/math/TPose3D.h>
#include <mrpt/math/TTwist2D.h>
#include <mvsim/CsvLogger.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/World.h>

#include <CLI/CLI.hpp>
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
	double value = 0;  // velocity (m/s) or angular velocity (rad/s)
};

struct PlantModel
{
	double K = 0;	// steady-state gain
	double tau = 0;	 // time constant (s)
	double v_ss = 0;  // steady-state value
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
// Helper: create world + vehicle
// ========================================================
static std::pair<std::shared_ptr<mvsim::World>, std::shared_ptr<mvsim::VehicleBase>>
	create_world_and_vehicle(
		const std::string& vehicle_xml_path, const std::string& vehicle_class, double sim_step)
{
	auto world = std::make_shared<mvsim::World>();
	world->headless(true);
	world->set_gravity(9.81);
	world->set_simul_timestep(sim_step);

	{
		std::string world_xml =
			"<mvsim_world version=\"1.0\">"
			"  <include file=\"" +
			vehicle_xml_path +
			"\" default_sensors=\"false\" />"
			"</mvsim_world>";
		world->load_from_XML(world_xml, vehicle_xml_path);
	}

	std::string veh_xml = "<vehicle name=\"tuner_veh\" class=\"" + vehicle_class +
						  "\">"
						  "  <init_pose>0 0 0</init_pose>"
						  "</vehicle>";

	auto veh = mvsim::VehicleBase::factory(world.get(), veh_xml);
	return {world, veh};
}

// ========================================================
// Helper: compute vehicle params
// ========================================================
struct VehicleParams
{
	double total_mass = 0;
	double partial_mass = 0;
	double R = 0;  // wheel radius
	double max_friction_torque = 0;
	double distWheels = 0;	// distance between left and right wheels
	size_t nWheels = 0;
};

static VehicleParams get_vehicle_params(const mvsim::VehicleBase& veh)
{
	VehicleParams p;
	p.nWheels = veh.getNumWheels();
	p.total_mass = veh.getChassisMass();
	for (size_t i = 0; i < p.nWheels; i++)
		p.total_mass += veh.getWheelInfo(i).mass;

	p.partial_mass = p.total_mass / static_cast<double>(p.nWheels);
	p.R = veh.getWheelInfo(0).diameter * 0.5;

	const double mu = 0.8;
	p.max_friction_torque = mu * p.partial_mass * 9.81 * p.R;

	// Wheel track (for differential drive)
	if (p.nWheels >= 2)
		p.distWheels = std::abs(veh.getWheelInfo(0).y - veh.getWheelInfo(1).y);

	return p;
}

// ========================================================
// Plant identification from samples
// ========================================================
static PlantModel identify_plant(
	const std::vector<TimeVelSample>& samples, double test_input, double sim_duration,
	double sim_step, const char* label)
{
	PlantModel plant;
	plant.torque_applied = test_input;

	if (samples.size() < 10)
	{
		std::fprintf(stderr, "Error: Too few samples recorded (%zu) for %s\n", samples.size(), label);
		return plant;
	}

	// Steady-state value (average of last 20%)
	const size_t ss_start = samples.size() * 80 / 100;
	double ss_sum = 0;
	for (size_t i = ss_start; i < samples.size(); i++)
		ss_sum += samples[i].value;

	plant.v_ss = ss_sum / static_cast<double>(samples.size() - ss_start);
	plant.K = plant.v_ss / test_input;

	// Time constant τ: time to reach 63.2% of v_ss
	const double v_63 = plant.v_ss * 0.632;
	plant.tau = sim_duration;  // fallback
	for (const auto& s : samples)
	{
		if (s.value >= v_63)
		{
			plant.tau = s.time;
			break;
		}
	}

	// Sanity check: if tau is very small, use 95% method
	if (plant.tau < 2 * sim_step)
	{
		const double v_95 = plant.v_ss * 0.95;
		for (const auto& s : samples)
		{
			if (s.value >= v_95)
			{
				plant.tau = -s.time / std::log(0.05);
				break;
			}
		}
	}

	return plant;
}

// ========================================================
// Run open-loop step response with "raw" controller
// ========================================================
static PlantModel run_open_loop_step(
	const std::string& vehicle_xml_path, const std::string& vehicle_class, double test_torque,
	double sim_duration, double sim_step, bool rotation_mode)
{
	auto [world, veh] = create_world_and_vehicle(vehicle_xml_path, vehicle_class, sim_step);

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

	world->insert_vehicle(veh);

	auto params = get_vehicle_params(*veh);

	if (!rotation_mode)
	{
		// Print vehicle info (only once, during linear test)
		std::printf("Vehicle: %s (%zu wheels)\n", vehicle_class.c_str(), params.nWheels);
		for (size_t i = 0; i < params.nWheels; i++)
		{
			const auto& w = veh->getWheelInfo(i);
			std::printf(
				"  Wheel %zu: pos=(%.3f, %.3f) R=%.3f mass=%.2f\n", i, w.x, w.y,
				w.diameter * 0.5, w.mass);
		}
		std::printf("  Chassis mass: %.1f kg\n", veh->getChassisMass());
		std::printf("  Wheel track: %.3f m\n", params.distWheels);
		std::printf(
			"  Estimated max friction torque per wheel: %.2f Nm "
			"(mu=0.8, partial_mass=%.1f kg, R=%.3f m)\n",
			params.max_friction_torque, params.partial_mass, params.R);
	}

	// Choose test torque
	if (test_torque <= 0)
		test_torque = params.max_friction_torque * 0.5;

	std::printf("  Test torque: %.2f Nm (%s)\n\n", test_torque, rotation_mode ? "rotation" : "linear");

	// Setup data recording
	std::vector<TimeVelSample> samples;
	double current_avg_vel = 0;
	int wheel_count = 0;

	for (auto& logger : veh->getLoggers())
	{
		if (!logger) continue;
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

	const int steps = static_cast<int>(sim_duration / sim_step);

	for (int i = 0; i < steps; i++)
	{
		// For linear: both wheels get same torque (forward)
		// For rotation: opposite torques (spin in place)
		if (auto* diffVeh = dynamic_cast<mvsim::DynamicsDifferential*>(veh.get()))
		{
			auto* rawCtrl = dynamic_cast<mvsim::DynamicsDifferential::ControllerRawForces*>(
				diffVeh->getController().get());
			if (rawCtrl)
			{
				if (rotation_mode)
				{
					// Opposite torques: L forward, R backward → positive omega
					rawCtrl->setpoint_wheel_torque_l = -test_torque;
					rawCtrl->setpoint_wheel_torque_r = test_torque;
				}
				else
				{
					rawCtrl->setpoint_wheel_torque_l = -test_torque;
					rawCtrl->setpoint_wheel_torque_r = -test_torque;
				}
			}
		}
		else if (auto* ackVeh = dynamic_cast<mvsim::DynamicsAckermann*>(veh.get()))
		{
			auto* rawCtrl = dynamic_cast<mvsim::DynamicsAckermann::ControllerRawForces*>(
				ackVeh->getController().get());
			if (rawCtrl)
			{
				if (rotation_mode)
				{
					rawCtrl->setpoint_wheel_torque_l = -test_torque;
					rawCtrl->setpoint_wheel_torque_r = test_torque;
				}
				else
				{
					rawCtrl->setpoint_wheel_torque_l = -test_torque;
					rawCtrl->setpoint_wheel_torque_r = -test_torque;
				}
				rawCtrl->setpoint_steer_ang = 0;
			}
		}

		current_avg_vel = 0;
		wheel_count = 0;

		world->run_simulation(sim_step);

		if (wheel_count > 0)
		{
			if (rotation_mode)
			{
				// For rotation, measure the difference in wheel velocities
				// The callback accumulates all wheel vels. For differential with 2 wheels:
				// omega = (v_R - v_L) / distWheels
				// But we measure via odometry estimate instead for robustness
				// Actually, let's use the per-wheel approach:
				// With opposite torques, L goes forward (+vel_x) and R goes backward (-vel_x)
				// avg_vel_x will be ~0, but we want the differential.
				// Better: read omega from the vehicle's velocity estimate.
			}
			else
			{
				samples.push_back({i * sim_step, current_avg_vel / wheel_count});
			}
		}

		if (rotation_mode)
		{
			// Use vehicle's odometry estimate for angular velocity
			mrpt::math::TTwist2D vel;
			if (auto* diffVeh = dynamic_cast<mvsim::DynamicsDifferential*>(veh.get()))
				vel = diffVeh->getVelocityLocalOdoEstimate();
			else if (auto* ackVeh = dynamic_cast<mvsim::DynamicsAckermann*>(veh.get()))
				vel = ackVeh->getVelocityLocalOdoEstimate();

			samples.push_back({i * sim_step, std::abs(vel.omega)});
		}
	}

	const char* label = rotation_mode ? "rotation" : "linear";
	PlantModel plant = identify_plant(samples, test_torque, sim_duration, sim_step, label);

	std::printf("=== Open-Loop Step Response Analysis (%s) ===\n", label);
	std::printf("Applied torque:        %.3f Nm\n", test_torque);
	if (rotation_mode)
	{
		std::printf("Steady-state omega:    %.4f rad/s\n", plant.v_ss);
		std::printf("Plant gain K:          %.4f (rad/s)/Nm\n", plant.K);
	}
	else
	{
		std::printf("Steady-state velocity: %.4f m/s\n", plant.v_ss);
		std::printf("Plant gain K:          %.4f (m/s)/Nm\n", plant.K);
	}
	std::printf("Time constant tau:     %.4f s\n", plant.tau);

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
	//   PI controller:
	//     KP = τ / (K * τ_cl)
	//     KI = KP / τ = 1 / (K * τ_cl)
	//     KD = 0

	const double tau_cl = plant.tau * aggressiveness;

	pid.KP = plant.tau / (plant.K * tau_cl);
	pid.KI = pid.KP / plant.tau;
	pid.KD = 0;

	pid.max_torque = max_friction_torque * 0.8;

	return pid;
}

// ========================================================
// Validate proposed PID with closed-loop simulation
// ========================================================
struct ValidationResult
{
	double rise_time = 0;
	double settling_time = 0;
	double overshoot_pct = 0;
	double steady_state_error = 0;
	double stop_settling_time = 0;
	double stop_overshoot = 0;
};

static ValidationResult validate_pid(
	const std::string& vehicle_xml_path, const std::string& vehicle_class, const PIDParams& pid,
	double sim_step, bool rotation_mode)
{
	ValidationResult result;

	auto [world, veh] = create_world_and_vehicle(vehicle_xml_path, vehicle_class, sim_step);

	// Set proposed PID parameters on the controller
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

	world->insert_vehicle(veh);

	// Phase 1: Step-up, Phase 2: Step-down
	// For linear: 1.0 m/s
	// For rotation: 1.0 rad/s (~57 deg/s)
	const double setpoint = 1.0;
	const double step_up_time = 3.0;
	const double step_down_time = 3.0;

	const mrpt::math::TTwist2D cmd_active =
		rotation_mode ? mrpt::math::TTwist2D(0, 0, setpoint) : mrpt::math::TTwist2D(setpoint, 0, 0);
	const mrpt::math::TTwist2D cmd_stop(0, 0, 0);

	veh->getControllerInterface()->setTwistCommand(cmd_active);

	int total_steps = static_cast<int>((step_up_time + step_down_time) / sim_step);
	int step_down_at = static_cast<int>(step_up_time / sim_step);
	double max_val = 0;

	std::vector<TimeVelSample> samples;

	for (int i = 0; i < total_steps; i++)
	{
		if (i == step_down_at)
			veh->getControllerInterface()->setTwistCommand(cmd_stop);

		world->run_simulation(sim_step);

		// Read actual velocity
		mrpt::math::TTwist2D vel;
		if (auto* diffVeh = dynamic_cast<mvsim::DynamicsDifferential*>(veh.get()))
			vel = diffVeh->getVelocityLocalOdoEstimate();
		else if (auto* ackVeh = dynamic_cast<mvsim::DynamicsAckermann*>(veh.get()))
			vel = ackVeh->getVelocityLocalOdoEstimate();

		double measured = rotation_mode ? vel.omega : vel.vx;
		samples.push_back({i * sim_step, measured});

		if (i < step_down_at)
			max_val = std::max(max_val, measured);
	}

	// Rise time: time to reach 90% of setpoint
	result.rise_time = step_up_time;
	for (const auto& s : samples)
	{
		if (s.time > step_up_time) break;
		if (s.value >= 0.9 * setpoint)
		{
			result.rise_time = s.time;
			break;
		}
	}

	// Overshoot
	result.overshoot_pct = std::max(0.0, (max_val - setpoint) / setpoint * 100.0);

	// Steady-state error
	{
		const size_t ss_start = step_down_at * 80 / 100;
		double ss_sum = 0;
		int ss_count = 0;
		for (size_t i = ss_start; i < static_cast<size_t>(step_down_at); i++)
		{
			ss_sum += samples[i].value;
			ss_count++;
		}
		if (ss_count > 0)
			result.steady_state_error = std::abs(setpoint - ss_sum / ss_count);
	}

	// Settling time
	{
		const double band = 0.02 * setpoint;
		result.settling_time = step_up_time;
		for (int i = step_down_at - 1; i >= 0; i--)
		{
			if (std::abs(samples[i].value - setpoint) > band)
			{
				if (i + 1 < step_down_at)
					result.settling_time = samples[i + 1].time;
				break;
			}
		}
	}

	// Step-down analysis
	{
		const double stop_band = 0.02 * setpoint;
		result.stop_settling_time = step_down_time;
		double min_val = setpoint;
		for (size_t i = step_down_at; i < samples.size(); i++)
		{
			min_val = std::min(min_val, samples[i].value);
			if (std::abs(samples[i].value) <= stop_band)
			{
				result.stop_settling_time = samples[i].time - step_up_time;
				break;
			}
		}
		result.stop_overshoot = std::max(0.0, -min_val);
	}

	return result;
}

// ========================================================
// Print validation results
// ========================================================
static bool print_validation(
	const ValidationResult& val, const char* label, double setpoint, const char* unit)
{
	std::printf("\nStep-up (0 -> %.1f %s):\n", setpoint, unit);
	std::printf("  Rise time (90%%):       %.3f s\n", val.rise_time);
	std::printf("  Settling time (2%%):    %.3f s\n", val.settling_time);
	std::printf("  Overshoot:             %.1f %%\n", val.overshoot_pct);
	std::printf("  Steady-state error:    %.4f %s\n", val.steady_state_error, unit);

	std::printf("\nStep-down (%.1f -> 0 %s):\n", setpoint, unit);
	std::printf("  Settling time (2%%):    %.3f s\n", val.stop_settling_time);
	if (val.stop_overshoot > 0.001)
	{
		std::printf(
			"  Rebound: %.4f %s  *** WARNING: vehicle reverses!\n", val.stop_overshoot, unit);
	}
	else
	{
		std::printf("  Rebound:               none\n");
	}

	bool good = true;
	if (val.overshoot_pct > 20)
	{
		std::printf(
			"WARNING [%s]: High overshoot (%.1f%%). Consider increasing aggressiveness.\n", label,
			val.overshoot_pct);
		good = false;
	}
	if (val.steady_state_error > 0.05)
	{
		std::printf(
			"WARNING [%s]: Large steady-state error (%.4f). KI may need to be higher.\n", label,
			val.steady_state_error);
		good = false;
	}
	if (val.stop_overshoot > 0.01)
	{
		std::printf(
			"WARNING [%s]: Vehicle rebounds when stopping. KI may be too high.\n", label);
		good = false;
	}
	if (val.rise_time > 2.0)
	{
		std::printf(
			"WARNING [%s]: Slow rise time (%.1fs). Consider decreasing aggressiveness.\n", label,
			val.rise_time);
		good = false;
	}
	return good;
}

// ========================================================
// Main
// ========================================================
int main(int argc, char** argv)
{
	try
	{
		CLI::App app("mvsim-pid-tuner: Automatic PID tuning for MVSim vehicles");

		std::string vehicle_xml;
		app.add_option("vehicle_xml", vehicle_xml, "Path to vehicle definition XML file")
			->required();

		std::string vehicle_class;
		app.add_option("vehicle_class", vehicle_class, "Vehicle class name as defined in the XML")
			->required();

		double test_torque = -1.0;
		app.add_option("-t,--torque", test_torque,
			"Test torque (Nm) for open-loop identification. "
			"Default: 50% of estimated friction limit.");

		double sim_duration = 5.0;
		app.add_option("-d,--duration", sim_duration, "Open-loop step duration (s).");

		double sim_step = 0.001;
		app.add_option("-s,--sim-step", sim_step, "Simulation time step (s).");

		double aggressiveness = 0.25;
		app.add_option("-a,--aggressiveness", aggressiveness,
			"Closed-loop aggressiveness factor (0.1=very aggressive, 1.0=conservative).");

		CLI11_PARSE(app, argc, argv);

		if (aggressiveness <= 0)
		{
			std::fprintf(stderr, "Error: aggressiveness must be > 0.\n");
			return 1;
		}

		std::printf(
			"======================================\n"
			" MVSim PID Auto-Tuner\n"
			"======================================\n\n");

		// Phase 1a: Open-loop linear identification
		std::printf("--- Phase 1a: Open-loop LINEAR plant identification ---\n");
		PlantModel plantLin = run_open_loop_step(
			vehicle_xml, vehicle_class, test_torque, sim_duration, sim_step, false);

		if (plantLin.K <= 0 || plantLin.tau <= 0)
		{
			std::fprintf(stderr, "\nError: Could not identify linear plant model.\n");
			return 1;
		}

		// Phase 1b: Open-loop rotation identification
		std::printf("\n--- Phase 1b: Open-loop ROTATION plant identification ---\n");
		PlantModel plantRot = run_open_loop_step(
			vehicle_xml, vehicle_class, test_torque, sim_duration, sim_step, true);

		if (plantRot.K <= 0 || plantRot.tau <= 0)
		{
			std::fprintf(stderr, "\nError: Could not identify rotation plant model.\n");
			return 1;
		}

		// Get vehicle params for max_torque computation
		auto [tmpWorld, tmpVeh] =
			create_world_and_vehicle(vehicle_xml, vehicle_class, sim_step);
		auto vehParams = get_vehicle_params(*tmpVeh);

		// Phase 2: Compute PID parameters
		// Since the twist_pid controller uses the same KP/KI/KD for both wheels,
		// and rotation dynamics differ from linear dynamics, we need to find
		// parameters that work well for both.
		//
		// Strategy: compute PID for both linear and rotation plants, then pick
		// the more conservative (slower) parameters to ensure stability for both.
		std::printf("\n--- Phase 2: IMC PID tuning (aggressiveness=%.2f) ---\n", aggressiveness);

		PIDParams pidLin = compute_pid_imc(plantLin, vehParams.max_friction_torque, aggressiveness);
		PIDParams pidRot = compute_pid_imc(plantRot, vehParams.max_friction_torque, aggressiveness);

		std::printf("\n  Linear-only PID:  KP=%.4f  KI=%.4f\n", pidLin.KP, pidLin.KI);
		std::printf("  Rotation-only PID: KP=%.4f  KI=%.4f\n", pidRot.KP, pidRot.KI);

		// Use the geometric mean of both to balance linear and rotation performance.
		// This ensures neither mode is overly aggressive.
		PIDParams pid;
		pid.KP = std::sqrt(pidLin.KP * pidRot.KP);
		pid.KI = std::sqrt(pidLin.KI * pidRot.KI);
		pid.KD = 0;
		pid.max_torque = vehParams.max_friction_torque * 0.8;

		std::printf("\nProposed PID parameters (balanced linear + rotation):\n");
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

		std::printf("\n[Linear] Step-up (1.0 m/s, 3s) then step-down (stop, 3s)...\n");
		ValidationResult valLin = validate_pid(vehicle_xml, vehicle_class, pid, sim_step, false);
		bool goodLin = print_validation(valLin, "linear", 1.0, "m/s");

		std::printf("\n[Rotation] Step-up (1.0 rad/s, 3s) then step-down (stop, 3s)...\n");
		ValidationResult valRot = validate_pid(vehicle_xml, vehicle_class, pid, sim_step, true);
		bool goodRot = print_validation(valRot, "rotation", 1.0, "rad/s");

		std::printf("\n--- Assessment ---\n");
		if (goodLin && goodRot)
			std::printf("All metrics look good for both linear and rotation!\n");
		else if (goodLin)
			std::printf("Linear metrics OK, but rotation has warnings (see above).\n");
		else if (goodRot)
			std::printf("Rotation metrics OK, but linear has warnings (see above).\n");
		else
			std::printf("Both linear and rotation have warnings (see above).\n");

		std::printf("\n");
		return 0;
	}
	catch (const std::exception& e)
	{
		std::fprintf(stderr, "Error: %s\n", e.what());
		return 1;
	}
}
