/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Slopes load the motors of a rolling vehicle (the motor torque needed to
// climb at constant speed grows with the slope), while a vehicle stopped on a
// slope does not creep.

#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <cmath>
#include <iostream>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{
struct Result
{
	double meanTorque = 0;	//!< mean |torque| of the left motor while climbing
	double pitch = 0;  //!< vehicle pitch while climbing [rad]
	double roll = 0;
	double creep = 0;  //!< displacement after stopping [m]
};

Result run(double heightStep)
{
	const double dt = 0.002;
	// Ramp rising along +x (1 m cells, slope = heightStep / 1 m), at z=0 around the
	// start position (the vehicle only "falls" onto terrain below it):
	std::string matrix;
	const int n = 40;
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n; j++)
		{
			matrix += std::to_string(heightStep * (i - n / 2)) + (j + 1 < n ? " " : "");
		}
		matrix += (i + 1 < n ? ";" : "");
	}
	const std::string worldXml = R"(
<mvsim_world version="1.0">
  <simul_timestep>)" + std::to_string(dt) +
								 R"(</simul_timestep>
  <element class="elevation_map">
    <resolution>1.0</resolution>
    <elevation_data_matrix>[)" + matrix +
								 R"(]</elevation_data_matrix>
    <corner_min_x>-20</corner_min_x>
    <corner_min_y>-20</corner_min_y>
  </element>
  <vehicle name="veh">
    <dynamics class="differential_4_wheels">
      <lf_wheel pos="0.3  0.3" mass="2.0" width="0.05" diameter="0.30" />
      <rf_wheel pos="0.3 -0.3" mass="2.0" width="0.05" diameter="0.30" />
      <lr_wheel pos="-0.3  0.3" mass="2.0" width="0.05" diameter="0.30" />
      <rr_wheel pos="-0.3 -0.3" mass="2.0" width="0.05" diameter="0.30" />
      <chassis mass="60.0" zmin="0.05" zmax="0.30" />
      <controller class="twist_pid">
        <KP>70</KP><KI>40</KI><KD>0</KD>
        <max_torque>200</max_torque>
      </controller>
    </dynamics>
    <friction class="default">
      <mu>0.8</mu>
      <C_damping>0.1</C_damping>
    </friction>
    <init_pose>-1 0 0</init_pose>
  </vehicle>
</mvsim_world>
)";

	mvsim::World world;
	world.headless(true);
	world.load_from_XML(worldXml, ".");
	auto& veh = *world.getListOfVehicles().begin()->second;

	double sumTorque = 0;
	int nTorque = 0;
	bool climbing = false;
	veh.getLoggerPtr(mvsim::VehicleBase::LOGGER_IDX_WHEELS)
		->registerOnRowCallback(
			[&](const auto& cols)
			{
				if (!climbing)
				{
					return;
				}
				if (auto it = cols.find("pid_torque_l"); it != cols.end())
				{
					sumTorque += std::abs(it->second);
					nTorque++;
				}
			});

	Result r;
	// Accelerate, then climb at constant speed:
	veh.getControllerInterface()->setTwistCommand({0.5, 0, 0});
	world.run_simulation(2.0);
	climbing = true;
	world.run_simulation(2.0);
	climbing = false;
	r.pitch = veh.getPose().pitch;
	r.roll = veh.getPose().roll;
	r.meanTorque = nTorque > 0 ? sumTorque / nTorque : 0;

	// Stop, wait for the vehicle to settle, and measure any creep:
	veh.getControllerInterface()->setTwistCommand({0, 0, 0});
	world.run_simulation(2.0);
	const auto p0 = veh.getPose();
	world.run_simulation(3.0);
	const auto p1 = veh.getPose();
	r.creep = std::hypot(p1.x - p0.x, p1.y - p0.y);
	return r;
}
}  // namespace

int main()
{
	const Result flat = run(0.0);
	const Result slope = run(0.1);	// ~5.7 deg

	std::printf(
		"flat:  pitch=%.2f deg torque=%.3f Nm creep=%.5f m\n", flat.pitch * 180 / M_PI,
		flat.meanTorque, flat.creep);
	std::printf(
		"slope: pitch=%.2f deg roll=%.2f deg torque=%.3f Nm creep=%.5f m\n",
		slope.pitch * 180 / M_PI, slope.roll * 180 / M_PI, slope.meanTorque, slope.creep);

	// The terrain is tilted under the vehicle:
	EXPECT_GT(std::abs(slope.pitch), 0.05);

	// Climbing at constant speed needs (roughly) the weight component along the
	// slope, m*g*sin(pitch)*R, shared among the 4 motors:
	const double mass = 60.0 + 4 * 2.0;
	const double expected = mass * 9.8 * std::sin(std::abs(slope.pitch)) * 0.15 / 4;
	EXPECT_GT(slope.meanTorque - flat.meanTorque, 0.7 * expected);
	EXPECT_LT(slope.meanTorque - flat.meanTorque, 1.3 * expected);

	// No creep once stopped, neither on flat ground nor on the slope:
	EXPECT_LT(flat.creep, 0.002);
	EXPECT_LT(slope.creep, 0.002);

	if (g_failures)
	{
		std::cerr << "test_slope_load: " << g_failures << " failure(s)\n";
		return 1;
	}
	std::cout << "test_slope_load: all passed\n";
	return 0;
}
