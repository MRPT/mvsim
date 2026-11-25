/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// XML numeric parameters must fill the whole target variable, whatever the
// "%f"/"%lf" format used for it, and <odometry> multipliers must take effect.

#include <mvsim/TParameterDefinitions.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <iostream>
#include <string>

#include "test_utils.h"

int g_failures = 0;

namespace
{
void test_mismatched_formats()
{
	std::cout << "[TEST] %f/%lf formats for double and float targets\n";

	double d = 7.0;
	mvsim::TParamEntry("%f", &d).parse("1.1", "d");
	EXPECT_NEAR(d, 1.1, 1e-12);

	// A trailing guard detects writes past the end of the float:
	struct
	{
		float value = 0;
		float guard = 42.0f;
	} f;
	mvsim::TParamEntry("%lf", &f.value).parse("2.5", "f");
	EXPECT_NEAR(f.value, 2.5, 1e-6);
	EXPECT_NEAR(f.guard, 42.0, 0);

	double ang = 0;
	mvsim::TParamEntry("%f_deg", &ang).parse("90", "ang");
	EXPECT_NEAR(ang, M_PI / 2, 1e-12);
}

void test_odometry_multipliers()
{
	std::cout << "[TEST] <odometry> multipliers\n";

	const double dt = 1e-3;
	mvsim::World world;
	world.headless(true);
	world.setMinLoggingLevel(mrpt::system::LVL_WARN);
	world.load_from_XML(
		"<mvsim_world version=\"1.0\"><simul_timestep>" + std::to_string(dt) +
		R"(</simul_timestep>
<vehicle name="veh">
	<dynamics class="differential">
		<l_wheel pos="0.0  0.5" mass="4.0" width="0.20" diameter="0.40" />
		<r_wheel pos="0.0 -0.5" mass="4.0" width="0.20" diameter="0.40" />
		<chassis mass="15.0" zmin="0.05" zmax="0.6" />
		<controller class="twist_ideal" />
	</dynamics>
	<odometry x_multiplier="1.1" y_multiplier="1.0" yaw_multiplier="0.9" />
	<init_pose>0 0 0</init_pose>
</vehicle>
</mvsim_world>)");
	auto veh = world.getListOfVehicles().find("veh")->second;

	const auto run = [&](double seconds, const mrpt::math::TTwist2D& cmd)
	{
		veh->getControllerInterface()->setTwistCommand(cmd);
		for (int i = 0; i < static_cast<int>(std::round(seconds / dt)); i++)
		{
			world.run_simulation(dt);
		}
	};

	// Straight motion: odometry scales the traveled distance.
	run(2.0, {1.0, 0, 0});
	const double gtX = veh->getCPose3D().x();
	EXPECT_GT(gtX, 1.0);
	EXPECT_NEAR(veh->getOdometry().x() / gtX, 1.1, 1e-3);

	// Rotation in place: odometry scales the heading change.
	veh->resetOdometry();
	const double yaw0 = veh->getCPose3D().yaw();
	run(1.0, {0, 0, 0.5});
	const double dYaw = veh->getCPose3D().yaw() - yaw0;
	EXPECT_GT(dYaw, 0.3);
	EXPECT_NEAR(veh->getOdometry().phi() / dYaw, 0.9, 1e-3);
}
}  // namespace

int main()
{
	test_mismatched_formats();
	test_odometry_multipliers();

	if (g_failures)
	{
		std::cerr << "test_xml_param_types: " << g_failures << " failure(s)\n";
		return 1;
	}
	std::cout << "test_xml_param_types: all passed\n";
	return 0;
}
