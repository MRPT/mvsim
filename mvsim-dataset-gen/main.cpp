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
// ideal (or optionally noisy) simulated 3D LiDAR, IMU and wheel-odometry
// data. See ~/plans/mvsim-lidar-simulator.md for the full design.

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mvsim/Sensors/IMU.h>
#include <mvsim/Sensors/Lidar3D.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/raytracer/Lidar3DModel.h>

#include <CLI/CLI.hpp>
#include <functional>
#include <iostream>
#include <queue>
#include <random>

#include "ImuSimulator.h"
#include "LidarSimulator.h"
#include "OdometrySimulator.h"
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
	double odomRateHz = 10.0;
	double odomTransNoiseRel = 0.01;  //!< Std.dev. as a fraction of distance traveled per step.
	double odomRotNoiseRel = 0.01;	//!< Std.dev. as a fraction of |motion| (m + rad) per step.
	std::string trajectoryFormat = "auto";	// auto | tum | waypoints2d
	double footprintLx = 0.6;
	double footprintLy = 0.4;

	cli.add_option("world_xml", worldXmlPath, "World XML file (analytic geometry only)")
		->required()
		->check(CLI::ExistingFile);
	cli.add_option(
		   "--trajectory", trajectoryPath,
		   "Ground-truth trajectory: .tum (SE3), or 2D waypoints (XML <waypoint> block or plain "
		   "'t x y' text) with terrain-following")
		->required()
		->check(CLI::ExistingFile);
	cli.add_option(
		   "--trajectory-format", trajectoryFormat,
		   "Trajectory format: auto (default, by extension: .tum -> tum, else -> waypoints2d) | "
		   "tum | "
		   "waypoints2d")
		->check(CLI::IsMember({"auto", "tum", "waypoints2d"}));
	cli.add_option(
		"--footprint-lx", footprintLx,
		"Vehicle footprint length [m], for terrain-following probes (waypoints2d only, default "
		"0.6)");
	cli.add_option(
		"--footprint-ly", footprintLy,
		"Vehicle footprint width [m], for terrain-following probes (waypoints2d only, default "
		"0.4)");
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
	cli.add_option("--odom-rate", odomRateHz, "Wheel-odometry sample rate [Hz] (default: 10)");
	cli.add_option(
		"--odom-trans-noise-rel", odomTransNoiseRel,
		"Odometry translation noise std.dev., as a fraction of distance traveled per step");
	cli.add_option(
		"--odom-rot-noise-rel", odomRotNoiseRel,
		"Odometry rotation noise std.dev., as a fraction of |motion| (m+rad) per step");

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

		// Collect the LiDAR and IMU sensors aboard the ego vehicle:
		std::vector<std::shared_ptr<mvsim::Lidar3D>> lidars;
		std::vector<std::shared_ptr<mvsim::IMU>> imus;
		for (const auto& s : egoVehicle->getSensors())
		{
			if (auto l = std::dynamic_pointer_cast<mvsim::Lidar3D>(s); l)
			{
				lidars.push_back(l);
			}
			else if (auto i = std::dynamic_pointer_cast<mvsim::IMU>(s); i)
			{
				imus.push_back(i);
			}
		}
		if (lidars.empty())
		{
			std::cerr << "Error: vehicle '" << vehicleName << "' has no lidar3d sensor.\n";
			return 1;
		}

		// Load the prescribed ground-truth trajectory:
		TrajectorySource traj;
		std::string fmt = trajectoryFormat;
		if (fmt == "auto")
		{
			const std::string ext = mrpt::system::extractFileExtension(trajectoryPath, true);
			fmt = mrpt::system::lowerCase(ext) == "tum" ? "tum" : "waypoints2d";
		}
		if (fmt == "tum")
		{
			traj.loadTum(trajectoryPath);
		}
		else
		{
			traj.load2DWithTerrain(trajectoryPath, world, footprintLx, footprintLy);
		}
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

		// A per-sensor generator: produces one observation at `nextTime`,
		// then advances by `period`. Sensors run at different rates, so all
		// of them are merged into the rawlog in strict global time order via
		// a min-heap below, rather than being written one sensor at a time.
		struct SensorGen
		{
			double nextTime;
			double period;
			std::function<mrpt::serialization::CSerializable::Ptr(double t)> generate;
		};
		std::vector<SensorGen> gens;

		size_t nSweeps = 0;
		for (const auto& lidar : lidars)
		{
			auto params =
				std::make_shared<mvsim::rt::Lidar3DModel::Params>(paramsFromSensor(*lidar));
			auto model = std::make_shared<mvsim::rt::Lidar3DModel>(*params);

			auto opts = std::make_shared<LidarSimOptions>(simOpts);
			opts->rangeStdNoise = noiseless ? 0.0 : lidar->rangeStdNoise() * noiseScale;
			opts->generateIntensity = lidar->generateIntensityFromRGB();

			const mrpt::poses::CPose3D sensorPoseOnVeh = lidar->sensorPoseOnVehicle();
			const std::string label = lidar->getName();

			gens.push_back(
				{t0, params->sensorPeriod,
				 [&, model, opts, sensorPoseOnVeh, label](double t)
				 {
					 auto obs = simulateLidarSweep(
						 scene, *model, traj, sensorPoseOnVeh, t, label, *opts, rng);
					 nSweeps++;
					 gtVehicle.insert(mrpt::Clock::fromDouble(t), traj.poseAt(t).asTPose());
					 gtSensors[label].insert(mrpt::Clock::fromDouble(t), obs->sensorPose.asTPose());
					 return obs;
				 }});
		}

		size_t nImuSamples = 0;
		for (const auto& imu : imus)
		{
			auto noiseModel = std::make_shared<mvsim::ImuNoiseModel>(imu->noiseModel());
			if (noiseless)
			{
				noiseModel->gyroscope = {};
				noiseModel->accelerometer = {};
			}
			else
			{
				noiseModel->gyroscope.white_noise_std *= noiseScale;
				noiseModel->gyroscope.random_walk_std *= noiseScale;
				noiseModel->accelerometer.white_noise_std *= noiseScale;
				noiseModel->accelerometer.random_walk_std *= noiseScale;
			}
			noiseModel->seed(seed);

			const double period = imu->sensor_period();
			const double diffStep = period / 10.0;
			const mrpt::poses::CPose3D sensorPoseOnVeh = imu->sensorPoseOnVehicle();
			const std::string label = imu->getName();
			const double orientationNoise =
				noiseless ? 0.0 : imu->orientationStdNoise() * noiseScale;
			const bool measureOri = imu->measureOrientation();
			auto prevT = std::make_shared<double>(t0);

			gens.push_back(
				{t0, period,
				 [&, noiseModel, diffStep, sensorPoseOnVeh, label, measureOri, orientationNoise,
				  prevT](double t)
				 {
					 auto obs = simulateImuSample(
						 world, traj, sensorPoseOnVeh, t, diffStep, t - *prevT, label, measureOri,
						 orientationNoise, *noiseModel);
					 nImuSamples++;
					 *prevT = t;
					 return obs;
				 }});
		}

		size_t nOdom = 0;
		{
			auto runningOdom = std::make_shared<mrpt::poses::CPose2D>();
			auto prevPose = std::make_shared<mrpt::math::TPose2D>(
				traj.poseAt(t0).x(), traj.poseAt(t0).y(), traj.poseAt(t0).yaw());
			const double odomPeriod = 1.0 / odomRateHz;
			const double transNoise = noiseless ? 0.0 : odomTransNoiseRel;
			const double rotNoise = noiseless ? 0.0 : odomRotNoiseRel;

			gens.push_back(
				{t0 + odomPeriod, odomPeriod,
				 [&, runningOdom, prevPose, transNoise, rotNoise](double t)
				 {
					 const auto p3 = traj.poseAt(t);
					 const mrpt::math::TPose2D curPose{p3.x(), p3.y(), p3.yaw()};
					 auto obs = simulateOdometryStep(
						 *prevPose, curPose, t, transNoise, rotNoise, *runningOdom, rng);
					 *prevPose = curPose;
					 nOdom++;
					 return obs;
				 }});
		}

		// Merge all sensor streams into the rawlog in strict global time
		// order (min-heap over each generator's next firing time):
		using Entry = std::pair<double, size_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<>> pq;
		for (size_t i = 0; i < gens.size(); i++)
		{
			if (gens[i].nextTime <= t1 + 1e-9)
			{
				pq.push({gens[i].nextTime, i});
			}
		}
		while (!pq.empty())
		{
			const auto [t, idx] = pq.top();
			pq.pop();
			auto& g = gens[idx];
			arch << *g.generate(t);
			g.nextTime += g.period;
			if (g.nextTime <= t1 + 1e-9)
			{
				pq.push({g.nextTime, idx});
			}
		}

		std::cout << "[mvsim-dataset-gen] Wrote " << nSweeps << " lidar sweep(s) from "
				  << lidars.size() << " sensor(s), " << nImuSamples << " IMU sample(s) from "
				  << imus.size() << " sensor(s), and " << nOdom << " odometry sample(s) to '"
				  << outputPath << "'.\n";

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
