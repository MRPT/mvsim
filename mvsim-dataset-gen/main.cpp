/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// mvsim-dataset-gen: offline, ray-traced (no OpenGL) simulated dataset
// generator. Given a world XML (restricted to analytic geometry) and a
// prescribed ground-truth trajectory, it writes an MRPT .rawlog with
// ideal (or optionally noisy) simulated 3D LiDAR sweeps.
// See ~/plans/mvsim-lidar-simulator.md for the full design.

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/Sensors/Lidar3D.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/raytracer/Lidar3DModel.h>

#include <CLI/CLI.hpp>
#include <iostream>
#include <random>

#include "LidarSimulator.h"
#include "SceneBuilder.h"
#include "TrajectorySource.h"

using namespace mvsim_dataset_gen;

namespace
{
mvsim::rt::Lidar3DModel::Params paramsFromSensor(const mvsim::Lidar3D& lidar)
{
	mvsim::rt::Lidar3DModel::Params p;
	p.vertNumRays = lidar.vertNumRays();
	p.vertFovDegrees = lidar.verticalFovDegrees();
	if (!lidar.verticalRayAnglesStr().empty())
	{
		std::vector<std::string> toks;
		mrpt::system::tokenize(lidar.verticalRayAnglesStr(), " \t\r\n", toks);
		for (const auto& s : toks)
		{
			p.verticalRayAnglesDeg.push_back(std::stod(s));
		}
	}
	p.horzNumRays = lidar.horzNumRays();
	p.sensorPeriod = lidar.sensor_period();
	p.minRange = lidar.minRange();
	p.maxRange = lidar.maxRange();
	return p;
}
}  // namespace

int main(int argc, char** argv)
{
	CLI::App cli{"mvsim-dataset-gen: ray-traced offline LiDAR/IMU dataset generator"};

	std::string worldXmlPath;
	std::string trajectoryPath;
	std::string outputPath;
	std::string vehicleName;
	std::string shutterStr = "global";
	double duration = -1.0;
	bool noiseless = false;
	double noiseScale = 1.0;
	unsigned seed = 0;
	bool allowUnsupported = false;

	cli.add_option("world_xml", worldXmlPath, "World XML file (analytic geometry only)")
		->required()
		->check(CLI::ExistingFile);
	cli.add_option("--trajectory", trajectoryPath, "Ground-truth trajectory, .tum format (SE3)")
		->required()
		->check(CLI::ExistingFile);
	cli.add_option("-o,--output", outputPath, "Output .rawlog file")->required();
	cli.add_option("--vehicle", vehicleName, "Name of the vehicle carrying the sensors");
	cli.add_option("--duration", duration, "Simulation duration [s] (default: trajectory span)");
	cli.add_option("--shutter", shutterStr, "LiDAR shutter mode: global (default) | rolling")
		->check(CLI::IsMember({"global", "rolling"}));
	cli.add_flag("--noiseless", noiseless, "Force every sensor noise sigma to zero");
	cli.add_option("--noise-scale", noiseScale, "Scale every XML-configured noise sigma");
	cli.add_option("--seed", seed, "RNG seed (default: 0, fully reproducible)");
	cli.add_flag(
		"--allow-unsupported", allowUnsupported,
		"Skip unsupported world geometry with a warning instead of aborting");

	CLI11_PARSE(cli, argc, argv);

	try
	{
		mvsim::World world;
		world.headless(true);
		world.load_from_XML_file(worldXmlPath);

		// Pick the ego vehicle:
		if (world.getListOfVehicles().empty())
		{
			std::cerr << "Error: world has no <vehicle>.\n";
			return 1;
		}
		mvsim::VehicleBase::Ptr egoVehicle;
		if (!vehicleName.empty())
		{
			auto it = world.getListOfVehicles().find(vehicleName);
			if (it == world.getListOfVehicles().end())
			{
				std::cerr << "Error: no vehicle named '" << vehicleName << "'.\n";
				return 1;
			}
			egoVehicle = it->second;
		}
		else if (world.getListOfVehicles().size() == 1)
		{
			egoVehicle = world.getListOfVehicles().begin()->second;
		}
		else
		{
			std::cerr << "Error: world has multiple vehicles; specify one with --vehicle.\n";
			return 1;
		}
		vehicleName = egoVehicle->getName();

		// Build the ray-traceable scene (everything except the ego vehicle):
		SceneBuilder::Options sbOpts;
		sbOpts.allowUnsupported = allowUnsupported;
		const auto scene = SceneBuilder::build(world, vehicleName, sbOpts);
		std::cout << "[mvsim-dataset-gen] Scene built: " << scene.primitiveCount()
				  << " ray-traceable primitives.\n";

		// Collect the LiDAR sensors aboard the ego vehicle:
		std::vector<std::shared_ptr<mvsim::Lidar3D>> lidars;
		for (const auto& s : egoVehicle->getSensors())
		{
			if (auto l = std::dynamic_pointer_cast<mvsim::Lidar3D>(s); l)
			{
				lidars.push_back(l);
			}
		}
		if (lidars.empty())
		{
			std::cerr << "Error: vehicle '" << vehicleName << "' has no lidar3d sensor.\n";
			return 1;
		}

		// Load the prescribed ground-truth trajectory:
		TrajectorySource traj;
		traj.loadTum(trajectoryPath);
		const double t0 = traj.startTime();
		const double simDuration = duration > 0 ? duration : (traj.endTime() - traj.startTime());
		const double t1 = t0 + simDuration;

		LidarSimOptions simOpts;
		simOpts.shutter = shutterStr == "rolling" ? ShutterMode::Rolling : ShutterMode::Global;

		std::mt19937 rng(seed);

		mrpt::io::CFileGZOutputStream rawlogStream(outputPath);
		auto arch = mrpt::serialization::archiveFrom(rawlogStream);

		mrpt::poses::CPose3DInterpolator gtVehicle;
		std::map<std::string, mrpt::poses::CPose3DInterpolator> gtSensors;

		size_t nSweeps = 0;
		for (const auto& lidar : lidars)
		{
			const auto params = paramsFromSensor(*lidar);
			mvsim::rt::Lidar3DModel model(params);

			simOpts.rangeStdNoise = noiseless ? 0.0 : lidar->rangeStdNoise() * noiseScale;
			simOpts.generateIntensity = lidar->generateIntensityFromRGB();

			const mrpt::poses::CPose3D sensorPoseOnVeh = lidar->sensorPoseOnVehicle();
			const std::string label = lidar->getName();

			for (double t = t0; t <= t1 + 1e-9; t += params.sensorPeriod)
			{
				auto obs =
					simulateLidarSweep(scene, model, traj, sensorPoseOnVeh, t, label, simOpts, rng);
				arch << *obs;
				nSweeps++;

				gtVehicle.insert(mrpt::Clock::fromDouble(t), traj.poseAt(t).asTPose());
				gtSensors[label].insert(mrpt::Clock::fromDouble(t), obs->sensorPose.asTPose());
			}
		}

		std::cout << "[mvsim-dataset-gen] Wrote " << nSweeps << " lidar sweep(s) from "
				  << lidars.size() << " sensor(s) to '" << outputPath << "'.\n";

		const std::string gtBase = mrpt::system::fileNameChangeExtension(outputPath, "gt.tum");
		gtVehicle.saveToTextFile_TUM(gtBase);
		std::cout << "[mvsim-dataset-gen] Wrote vehicle ground truth to '" << gtBase << "'.\n";
		for (const auto& [label, path] : gtSensors)
		{
			const std::string p =
				mrpt::system::fileNameChangeExtension(outputPath, label + ".gt.tum");
			path.saveToTextFile_TUM(p);
			std::cout << "[mvsim-dataset-gen] Wrote sensor '" << label << "' ground truth to '" << p
					  << "'.\n";
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}

	return 0;
}
