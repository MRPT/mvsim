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
// ideal (or optionally noisy) simulated 3D LiDAR, IMU, GNSS and
// wheel-odometry data, plus the ground truth of the trajectory and of every
// sensor. See docs/mvsim-dataset-gen.rst.

#include <mrpt/core/exceptions.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/progress.h>
#include <mrpt/system/string_utils.h>
#include <mvsim/Sensors/CameraSensor.h>
#include <mvsim/Sensors/DepthCameraSensor.h>
#include <mvsim/Sensors/GNSS.h>
#include <mvsim/Sensors/IMU.h>
#include <mvsim/Sensors/Lidar3D.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/raytracer/Lidar3DModel.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <random>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "CameraSimulator.h"
#include "GroundTruthYaml.h"
#include "ImuSimulator.h"
#include "LidarSimulator.h"
#include "OdometrySimulator.h"
#include "SceneBuilder.h"
#include "TimeUtils.h"
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

/** Parses the `--moving-block <block name>=<trajectory.tum>` arguments into
 * one moving object per block, and adds the block names to `movingNames`
 * and the blocks with their trajectories to `movingBlocks` (for cameras).
 * Blocks sharing a trajectory file share one trajectory, and are stored
 * together. Throws on any invalid argument. */
std::vector<MovingObject> loadMovingBlocks(
	const mvsim::World& world, const std::vector<std::string>& args,
	std::set<std::string>& movingNames, std::vector<CameraScene::MovingBlock>& movingBlocks)
{
	// Trajectory file => names of the blocks following it:
	std::map<std::string, std::vector<std::string>> blocksByFile;
	for (const auto& arg : args)
	{
		const size_t eq = arg.find('=');
		if (eq == std::string::npos || eq == 0 || eq + 1 == arg.size())
		{
			THROW_EXCEPTION(
				"Malformed --moving-block '" + arg + "': expected <block name>=<trajectory.tum>");
		}
		const std::string name = arg.substr(0, eq);
		if (world.getListOfBlocks().count(name) == 0)
		{
			THROW_EXCEPTION("--moving-block: the world has no block named '" + name + "'");
		}
		if (!movingNames.insert(name).second)
		{
			THROW_EXCEPTION("--moving-block: block '" + name + "' given more than once");
		}
		blocksByFile[arg.substr(eq + 1)].push_back(name);
	}

	std::vector<MovingObject> objects;
	for (const auto& [file, names] : blocksByFile)
	{
		auto traj = std::make_shared<TrajectorySource>();
		traj->loadTum(file);
		for (const auto& name : names)
		{
			const auto range = world.getListOfBlocks().equal_range(name);
			for (auto it = range.first; it != range.second; ++it)
			{
				// Same rule as for the static scene:
				if (it->second->isIntangible())
				{
					THROW_EXCEPTION(
						"--moving-block: block '" + name +
						"' is intangible, and intangible blocks are not ray-traced");
				}
				auto blockScene = SceneBuilder::buildBlock(*it->second);
				if (blockScene.primitiveCount() == 0)
				{
					THROW_EXCEPTION(
						"--moving-block: block '" + name + "' has no ray-traceable geometry");
				}
				objects.emplace_back(std::move(blockScene), traj);
				movingBlocks.emplace_back(it->second, traj);
			}
		}
	}
	return objects;
}

/** Parses the `--clock-offset <sensor label>=<seconds>` arguments. Throws on
 * any invalid argument. */
std::map<std::string, double> parseClockOffsets(const std::vector<std::string>& args)
{
	std::map<std::string, double> out;
	for (const auto& arg : args)
	{
		const size_t eq = arg.find('=');
		bool ok = eq != std::string::npos && eq != 0 && eq + 1 < arg.size();
		double value = 0;
		if (ok)
		{
			const std::string v = arg.substr(eq + 1);
			char* end = nullptr;
			value = std::strtod(v.c_str(), &end);
			// Representable as a clock tick count (100 ns):
			ok = end != v.c_str() && *end == '\0' && std::isfinite(value) &&
				 std::abs(value) * 1e7 < 9e18;
		}
		if (!ok)
		{
			THROW_EXCEPTION(
				"Malformed --clock-offset '" + arg + "': expected <sensor label>=<seconds>");
		}
		if (!out.emplace(arg.substr(0, eq), value).second)
		{
			THROW_EXCEPTION(
				"--clock-offset: sensor '" + arg.substr(0, eq) + "' given more than once");
		}
	}
	return out;
}

/** A seed of its own for the noise of each camera, from the run seed and
 * the camera name (FNV-1a, the same on every platform). */
uint32_t cameraNoiseSeed(unsigned seed, const std::string& label)
{
	uint32_t h = 2166136261u;
	for (const char c : label)
	{
		h = (h ^ static_cast<uint8_t>(c)) * 16777619u;
	}
	return h ^ (seed * 0x9E3779B9u);
}

/** Calls a function when it goes out of scope, also during stack unwinding. */
class AtScopeExit
{
   public:
	explicit AtScopeExit(std::function<void()> f) : f_(std::move(f)) {}
	~AtScopeExit() { f_(); }
	AtScopeExit(const AtScopeExit&) = delete;
	AtScopeExit& operator=(const AtScopeExit&) = delete;

   private:
	std::function<void()> f_;
};

/** File name of a path, without its directory. */
std::string fileNameOf(const std::string& path)
{
	const size_t slash = path.find_last_of("/\\");
	return slash == std::string::npos ? path : path.substr(slash + 1);
}
}  // namespace

int main(int argc, char** argv)
{
	CLI::App cli{"mvsim-dataset-gen: ray-traced offline LiDAR/IMU/GNSS dataset generator"};

	std::string worldXmlPath;
	std::string trajectoryPath;
	std::string outputPath;
	std::string vehicleName;
	std::string shutterStr = "rolling";
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
	bool labels = false;
	int mixedReturnLabel = -1;
	std::vector<std::string> movingBlockArgs;
	std::vector<std::string> clockOffsetArgs;
	bool noCameras = false;
	unsigned imageThreads = std::max(1u, std::min(8u, std::thread::hardware_concurrency()));

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
	cli.add_option(
		   "--shutter", shutterStr,
		   "LiDAR shutter mode: rolling (default: each azimuth column fires at its own time and "
		   "pose, per-point 't' in [0, sensor period)) | global (one pose per sweep, t=0)")
		->check(CLI::IsMember({"global", "rolling"}));
	cli.add_flag("--noiseless", noiseless, "Force every sensor noise sigma to zero");
	cli.add_flag(
		"--no-cameras", noCameras,
		"Do not render the camera and depth camera sensors (rendering needs OpenGL, through EGL)");
	cli.add_option(
		   "--image-threads", imageThreads,
		   "Threads that encode and write camera image files (default: up to 8)")
		->check(CLI::PositiveNumber);
	cli.add_option("--noise-scale", noiseScale, "Scale every XML-configured noise sigma");
	cli.add_option("--seed", seed, "RNG seed (default: 0, fully reproducible)");
	cli.add_flag(
		"--allow-unsupported", allowUnsupported,
		"Skip unsupported world geometry with a warning instead of aborting");
	cli.add_option(
		   "--moving-block", movingBlockArgs,
		   "A block that moves during the run, as <block name>=<trajectory.tum>: the block is "
		   "left out of the static scene, and its frame follows the given TUM trajectory (same "
		   "clock as --trajectory) instead of staying at its <init_pose>. Repeat the option for "
		   "more blocks; several blocks may share one trajectory file")
		->allow_extra_args(false);
	auto* labelsFlag = cli.add_flag(
		"--labels", labels,
		"Store per-point uint16 'label' and 'instance' LiDAR fields: the <semantic_class> and "
		"<instance_id> of the world entity each ray hit (0 if not given)");
	cli.add_option(
		   "--mixed-return-label", mixedReturnLabel,
		   "With --labels: semantic class of mixed returns (ranges between two surfaces), with "
		   "instance 0 (default: the labels of the surface their ray hit)")
		->check(CLI::Range(0, 65535))
		->needs(labelsFlag);
	cli.add_option(
		   "--clock-offset", clockOffsetArgs,
		   "Clock offset of a sensor stream, as <sensor label>=<seconds>: its observations are "
		   "stamped at their sampling time plus this offset (the wheel odometry label is "
		   "'odom'). Repeat the option for more sensors")
		->allow_extra_args(false);
	cli.add_option("--odom-rate", odomRateHz, "Wheel-odometry sample rate [Hz] (default: 10)")
		->check(CLI::PositiveNumber);
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

		// Build the ray-traceable scene (everything except the ego vehicle
		// and the moving blocks):
		SceneBuilder::Options sbOpts;
		sbOpts.allowUnsupported = allowUnsupported;
		std::vector<CameraScene::MovingBlock> movingBlocksForCameras;
		const std::vector<MovingObject> movingObjects =
			loadMovingBlocks(world, movingBlockArgs, sbOpts.excludedBlocks, movingBlocksForCameras);
		const auto scene = SceneBuilder::build(world, vehicleName, sbOpts);
		std::cout << "[mvsim-dataset-gen] Scene built: " << scene.primitiveCount()
				  << " ray-traceable primitives.\n";
		if (!movingObjects.empty())
		{
			std::cout << "[mvsim-dataset-gen] Moving blocks: " << movingObjects.size() << ".\n";
		}
		if (labels)
		{
			size_t nLabeled = 0;
			size_t nPrimitives = 0;
			const auto countLabeled = [&](const mvsim::rt::RayScene& s)
			{
				nPrimitives += s.primitiveCount();
				for (size_t i = 0; i < s.primitiveCount(); i++)
				{
					nLabeled += s.primitive(i).semanticClass != 0 ? 1 : 0;
				}
			};
			countLabeled(scene);
			for (const auto& obj : movingObjects)
			{
				countLabeled(obj.scene);
			}
			std::cout << "[mvsim-dataset-gen] Semantic labels: " << nLabeled << " of "
					  << nPrimitives << " primitives have a semantic class.\n";
		}

		// Collect the LiDAR, IMU and GNSS sensors aboard the ego vehicle:
		std::vector<std::shared_ptr<mvsim::Lidar3D>> lidars;
		std::vector<std::shared_ptr<mvsim::IMU>> imus;
		std::vector<std::shared_ptr<mvsim::GNSS>> gnssSensors;
		std::vector<std::shared_ptr<mvsim::CameraSensor>> cameras;
		std::vector<std::shared_ptr<mvsim::DepthCameraSensor>> depthCameras;
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
			else if (auto g = std::dynamic_pointer_cast<mvsim::GNSS>(s); g)
			{
				gnssSensors.push_back(g);
			}
			else if (auto c = std::dynamic_pointer_cast<mvsim::CameraSensor>(s); c && !noCameras)
			{
				cameras.push_back(c);
			}
			else if (auto d = std::dynamic_pointer_cast<mvsim::DepthCameraSensor>(s);
					 d && !noCameras)
			{
				if (d->senseRgb() || d->senseDepth())
				{
					depthCameras.push_back(d);
				}
				else
				{
					std::cerr << "[mvsim-dataset-gen] Warning: '" << d->getName()
							  << "' senses neither RGB nor depth: skipped.\n";
				}
			}
		}
		if (lidars.empty())
		{
			std::cerr << "Error: vehicle '" << vehicleName << "' has no lidar3d sensor.\n";
			return 1;
		}
		const auto& georef = world.georeferenceOptions();
		const bool hasGeoref = georef.is_set;
		if (!gnssSensors.empty() && !hasGeoref)
		{
			std::cerr << "Error: vehicle '" << vehicleName
					  << "' has a gnss sensor, but the world has no <georeference>.\n";
			return 1;
		}
		const std::map<std::string, double> clockOffsets = parseClockOffsets(clockOffsetArgs);
		const auto clockOffsetOf = [&](const std::string& label)
		{
			const auto it = clockOffsets.find(label);
			return it == clockOffsets.end() ? 0.0 : it->second;
		};

		// Validate the stream labels before any output is written. Each one
		// identifies one stream (sensors are already unique, but one may be
		// named as the wheel odometry stream, "odom"):
		{
			std::set<std::string> streamLabels = {"odom"};
			std::vector<std::string> sensorLabels;
			for (const auto& l : lidars)
			{
				sensorLabels.push_back(l->getName());
			}
			for (const auto& i : imus)
			{
				sensorLabels.push_back(i->getName());
			}
			for (const auto& g : gnssSensors)
			{
				sensorLabels.push_back(g->getName());
			}
			for (const auto& c : cameras)
			{
				sensorLabels.push_back(c->getName());
			}
			for (const auto& d : depthCameras)
			{
				sensorLabels.push_back(d->getName());
			}
			for (const auto& label : sensorLabels)
			{
				if (!streamLabels.insert(label).second)
				{
					THROW_EXCEPTION(
						"Two sensor streams are labeled '" + label +
						"' (the wheel odometry stream is 'odom'): rename the sensor");
				}
			}
			// Every --clock-offset must name a stream:
			for (const auto& [label, offset] : clockOffsets)
			{
				if (streamLabels.count(label) == 0)
				{
					THROW_EXCEPTION("--clock-offset: no sensor stream named '" + label + "'");
				}
			}
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

		// Every stamp, sampling time plus clock offset, must fit in the clock
		// tick count:
		for (const auto& [label, offset] : clockOffsets)
		{
			constexpr int64_t kMax = std::numeric_limits<int64_t>::max();
			constexpr int64_t kMin = std::numeric_limits<int64_t>::min();
			const int64_t off = std::llround(offset * 1e7);
			const int64_t first = timeFromSeconds(t0).time_since_epoch().count();
			const int64_t last = timeFromSeconds(t1).time_since_epoch().count();
			if ((off > 0 && last > kMax - off) || (off < 0 && first < kMin - off))
			{
				THROW_EXCEPTION("--clock-offset: the offset of '" + label + "' is out of range");
			}
		}

		LidarSimOptions simOpts;
		simOpts.shutter = shutterStr == "rolling" ? ShutterMode::Rolling : ShutterMode::Global;
		std::cout << "[mvsim-dataset-gen] LiDAR shutter mode: " << shutterStr << "\n";

		std::mt19937 rng(seed);

		mrpt::io::CFileGZOutputStream rawlogStream(outputPath);
		auto arch = mrpt::serialization::archiveFrom(rawlogStream);

		mrpt::poses::CPose3DInterpolator gtVehicle;
		std::map<std::string, mrpt::poses::CPose3DInterpolator> gtSensors;

		// A per-sensor generator: produces one observation sampled at
		// `nextTime` (or none, e.g. during a GNSS outage), then advances by
		// `period`. Sensors run at different rates, so all of them are merged
		// into the rawlog in strict global order of their stamps (sampling
		// time plus `clockOffset`) via a min-heap below, rather than being
		// written one sensor at a time.
		struct SensorGen
		{
			std::string label;
			double nextTime;
			double period;
			std::function<mrpt::obs::CObservation::Ptr(double t)> generate;
			double clockOffset = 0;
		};
		std::vector<SensorGen> gens;

		DatasetGroundTruth groundTruth;

		size_t nSweeps = 0;
		for (const auto& lidar : lidars)
		{
			auto params =
				std::make_shared<mvsim::rt::Lidar3DModel::Params>(paramsFromSensor(*lidar));
			auto model = std::make_shared<mvsim::rt::Lidar3DModel>(*params);

			auto opts = std::make_shared<LidarSimOptions>(simOpts);
			opts->rangeStdNoise = noiseless ? 0.0 : lidar->rangeStdNoise() * noiseScale;
			LidarGroundTruth lidarGt;
			if (!noiseless)
			{
				const auto& em = lidar->errorModel();
				opts->rangeStdNoisePerMeter = em.rangeStdNoisePerMeter * noiseScale;
				opts->beamDivergenceDeg = em.beamDivergenceDeg;
				opts->maxIncidenceDeg = em.maxIncidenceDeg;
				opts->mixedPixelProbability = em.mixedPixelProbability;
				opts->echoSeparation = em.echoSeparation;
				opts->detectionRange = em.detectionRange;
				drawCalibrationErrors(
					*opts, model->vertRays(), em.beamElevationStdDeg * noiseScale,
					em.beamAzimuthStdDeg * noiseScale, em.beamRangeBiasStd * noiseScale,
					em.calibrationSeed);
				lidarGt.beamElevationStdDeg = em.beamElevationStdDeg * noiseScale;
				lidarGt.beamAzimuthStdDeg = em.beamAzimuthStdDeg * noiseScale;
				lidarGt.beamRangeBiasStd = em.beamRangeBiasStd * noiseScale;
				lidarGt.calibrationSeed = em.calibrationSeed;
			}
			opts->generateIntensity = lidar->generateIntensityFromRGB();
			opts->generateLabels = labels;
			if (mixedReturnLabel >= 0)
			{
				opts->mixedReturnLabel = static_cast<uint16_t>(mixedReturnLabel);
			}
			if (lidar->generateRGB())
			{
				std::cerr << "[mvsim-dataset-gen] Warning: lidar '" << lidar->getName()
						  << "' has generate_rgb=true, but the ray tracer has no colors: "
							 "points will have no RGB fields.\n";
			}

			const mrpt::poses::CPose3D sensorPoseOnVeh = lidar->sensorPoseOnVehicle();
			const std::string label = lidar->getName();

			lidarGt.label = label;
			lidarGt.pose = sensorPoseOnVeh;
			lidarGt.clockOffset = clockOffsetOf(label);
			lidarGt.model = model;
			lidarGt.opts = opts;
			groundTruth.lidars.push_back(lidarGt);

			gens.push_back(
				{label, t0, params->sensorPeriod,
				 [&, model, opts, sensorPoseOnVeh, label](double t)
				 {
					 auto obs = simulateLidarSweep(
						 scene, movingObjects, *model, traj, sensorPoseOnVeh, t, label, *opts, rng);
					 nSweeps++;
					 gtVehicle.insert(timeFromSeconds(t), traj.poseAt(t).asTPose());
					 gtSensors[label].insert(
						 timeFromSeconds(t), (traj.poseAt(t) + sensorPoseOnVeh).asTPose());
					 return obs;
				 },
				 lidarGt.clockOffset});
		}

		size_t nImuSamples = 0;
		for (size_t imuIdx = 0; imuIdx < imus.size(); imuIdx++)
		{
			const auto& imu = imus[imuIdx];
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
			noiseModel->reset();
			// Each IMU has its own noise stream; the first one is seeded as in
			// older versions, so their datasets reproduce:
			noiseModel->seed(seed + 0x9E3779B9u * static_cast<unsigned>(imuIdx));

			const double period = imu->sensor_period();
			const double diffStep = period / 10.0;
			const mrpt::poses::CPose3D sensorPoseOnVeh = imu->sensorPoseOnVehicle();
			const std::string label = imu->getName();
			const double orientationNoise =
				noiseless ? 0.0 : imu->orientationStdNoise() * noiseScale;
			const bool measureOri = imu->measureOrientation();
			auto prevT = std::make_shared<double>(t0);

			ImuGroundTruth imuGt;
			imuGt.label = label;
			imuGt.pose = sensorPoseOnVeh;
			imuGt.clockOffset = clockOffsetOf(label);
			imuGt.period = period;
			imuGt.gyroscope = noiseModel->gyroscope;
			imuGt.accelerometer = noiseModel->accelerometer;
			// The bias time series, if it varies:
			std::shared_ptr<std::ofstream> biasOut;
			if (noiseModel->gyroscope.random_walk_std > 0 ||
				noiseModel->accelerometer.random_walk_std > 0)
			{
				const std::string biasPath =
					mrpt::system::fileNameChangeExtension(outputPath, label + ".bias.txt");
				biasOut = std::make_shared<std::ofstream>(biasPath);
				if (!biasOut->is_open())
				{
					THROW_EXCEPTION("Cannot write '" + biasPath + "'");
				}
				*biasOut << "# t gyro_bias_x gyro_bias_y gyro_bias_z accel_bias_x accel_bias_y "
							"accel_bias_z\n"
							"# Bias of each sample [s, rad/s, m/s^2], at its sampling time\n"
						 << std::setprecision(10);
				imuGt.biasFile = fileNameOf(biasPath);
			}
			groundTruth.imus.push_back(imuGt);

			gens.push_back(
				{label, t0, period,
				 [&, noiseModel, diffStep, sensorPoseOnVeh, label, measureOri, orientationNoise,
				  prevT, biasOut](double t)
				 {
					 auto obs = simulateImuSample(
						 world, traj, sensorPoseOnVeh, t, diffStep, t - *prevT, label, measureOri,
						 orientationNoise, *noiseModel, rng);
					 nImuSamples++;
					 *prevT = t;
					 if (biasOut)
					 {
						 const auto& bg = noiseModel->gyroBias();
						 const auto& ba = noiseModel->accelBias();
						 *biasOut << std::fixed << std::setprecision(7) << t << std::defaultfloat
								  << std::setprecision(10) << " " << bg.x << " " << bg.y << " "
								  << bg.z << " " << ba.x << " " << ba.y << " " << ba.z << "\n";
					 }
					 return obs;
				 },
				 imuGt.clockOffset});
		}

		size_t nGnss = 0;
		for (size_t gnssIdx = 0; gnssIdx < gnssSensors.size(); gnssIdx++)
		{
			const auto& gnss = gnssSensors[gnssIdx];
			auto gnssRng = std::make_shared<mrpt::random::CRandomGenerator>();
			gnssRng->randomize(seed ^ (0x85EBCA6Bu * static_cast<unsigned>(gnssIdx + 1)));
			const double gnssNoiseScale = noiseless ? 0.0 : noiseScale;

			GnssGroundTruth gnssGt;
			gnssGt.label = gnss->getName();
			gnssGt.sensor = gnss.get();
			gnssGt.clockOffset = clockOffsetOf(gnssGt.label);
			gnssGt.noiseScale = gnssNoiseScale;
			groundTruth.gnss.push_back(gnssGt);

			gens.push_back(
				{gnssGt.label, t0, gnss->sensor_period(),
				 [&, gnss, gnssRng, gnssNoiseScale](double t) -> mrpt::obs::CObservation::Ptr
				 {
					 // The degradation events are timed from the trajectory start:
					 auto obs = gnss->simulateReading(
						 traj.poseAt(t), t - t0, timeFromSeconds(t), *gnssRng, gnssNoiseScale);
					 if (obs)
					 {
						 nGnss++;
					 }
					 return obs;
				 },
				 gnssGt.clockOffset});
		}

		size_t nOdom = 0;
		{
			auto runningOdom = std::make_shared<mrpt::poses::CPose2D>();
			auto prevPose = std::make_shared<mrpt::math::TPose2D>(
				traj.poseAt(t0).x(), traj.poseAt(t0).y(), traj.poseAt(t0).yaw());
			const double odomPeriod = 1.0 / odomRateHz;
			const double transNoise = noiseless ? 0.0 : odomTransNoiseRel;
			const double rotNoise = noiseless ? 0.0 : odomRotNoiseRel;

			OdometryGroundTruth odomGt;
			odomGt.label = "odom";	// see OdometrySimulator
			odomGt.clockOffset = clockOffsetOf(odomGt.label);
			odomGt.period = odomPeriod;
			odomGt.transNoiseRel = transNoise;
			odomGt.rotNoiseRel = rotNoise;
			groundTruth.odometry.push_back(odomGt);

			gens.push_back(
				{odomGt.label, t0 + odomPeriod, odomPeriod,
				 [&, runningOdom, prevPose, transNoise, rotNoise](double t)
				 {
					 const auto p3 = traj.poseAt(t);
					 const mrpt::math::TPose2D curPose{p3.x(), p3.y(), p3.yaw()};
					 auto obs = simulateOdometryStep(
						 *prevPose, curPose, t, transNoise, rotNoise, *runningOdom, rng);
					 *prevPose = curPose;
					 nOdom++;
					 return obs;
				 },
				 odomGt.clockOffset});
		}

		// Cameras: rendered with OpenGL (EGL) from a scene of the world,
		// images in files in <output name>_Images/. The first renderer owns the
		// OpenGL context, so renderers are released in reverse order, on every
		// exit path (the generators hold them too):
		size_t nImages = 0;
		std::unique_ptr<CameraScene> cameraScene;
		std::vector<std::shared_ptr<RgbCameraSim>> rgbSims;
		std::vector<std::shared_ptr<DepthCameraSim>> depthSims;
		std::shared_ptr<ImageWriter> imageWriter;
		const AtScopeExit releaseRenderers(
			[&]()
			{
				gens.clear();
				while (!depthSims.empty())
				{
					depthSims.pop_back();
				}
				while (!rgbSims.empty())
				{
					rgbSims.pop_back();
				}
				cameraScene.reset();
			});
		const std::string imagesDirName =
			mrpt::system::extractFileName(outputPath) + std::string("_Images");
		const std::string imagesDir =
			mrpt::system::pathJoin({mrpt::system::extractFileDirectory(outputPath), imagesDirName});
		if (!cameras.empty() || !depthCameras.empty())
		{
			if (!mrpt::system::directoryExists(imagesDir) &&
				!mrpt::system::createDirectory(imagesDir))
			{
				THROW_EXCEPTION("Cannot create the images directory '" + imagesDir + "'");
			}
			cameraScene = std::make_unique<CameraScene>(world, movingBlocksForCameras);
			imageWriter = std::make_shared<ImageWriter>(imageThreads);
			const double camNoiseScale = noiseless ? 0.0 : noiseScale;
			bool firstRenderer = true;
			for (const auto& cam : cameras)
			{
				const std::string label = cam->getName();
				CameraImageModel m;
				m.camera = cam->sensorParams().cameraParams;
				m.poseOnVehicle = cam->sensorParams().cameraPose;
				m.clipMin = cam->clipMin();
				m.clipMax = cam->clipMax();
				m.noiseStd = cam->imageOptions().imageNoiseStd * camNoiseScale;
				m.rollingShutterLineTime = cam->imageOptions().rollingShutterLineTime;
				m.rollingShutterBands = cam->imageOptions().rollingShutterBands;
				auto sim = std::make_shared<RgbCameraSim>(
					m, label, imagesDir, imageWriter, cameraNoiseSeed(seed, label), firstRenderer);
				firstRenderer = false;
				rgbSims.push_back(sim);

				// The model as rendered (e.g. the band count it uses):
				const auto& sm = sim->model();
				CameraGroundTruth camGt;
				camGt.label = label;
				camGt.clockOffset = clockOffsetOf(label);
				camGt.period = cam->sensor_period();
				CameraImageGroundTruth im;
				im.camera = sm.camera;
				im.pose = sm.poseOnVehicle;
				im.clipMin = sm.clipMin;
				im.clipMax = sm.clipMax;
				im.noiseStd = sm.noiseStd;
				im.rollingShutterLineTime = sm.rollingShutterLineTime;
				im.rollingShutterBands = sm.rollingShutterBands;
				im.files = imagesDirName + "/" + label + "_%06d.png";
				camGt.rgb = im;
				groundTruth.cameras.push_back(camGt);

				gens.push_back(
					{label, t0, cam->sensor_period(),
					 [&, sim](double t) -> mrpt::obs::CObservation::Ptr
					 {
						 nImages++;
						 return sim->simulate(*cameraScene, traj, t);
					 },
					 camGt.clockOffset});
			}
			for (const auto& cam : depthCameras)
			{
				const std::string label = cam->getName();
				auto sim = std::make_shared<DepthCameraSim>(
					*cam, imagesDir, imageWriter, cameraNoiseSeed(seed, label), camNoiseScale,
					firstRenderer);
				firstRenderer = false;
				depthSims.push_back(sim);

				CameraGroundTruth camGt;
				camGt.label = label;
				camGt.clockOffset = clockOffsetOf(label);
				camGt.period = cam->sensor_period();
				if (sim->hasRgb())
				{
					const auto& m = sim->rgbModel();
					CameraImageGroundTruth im;
					im.camera = m.camera;
					im.pose = m.poseOnVehicle;
					im.clipMin = m.clipMin;
					im.clipMax = m.clipMax;
					im.noiseStd = m.noiseStd;
					im.rollingShutterLineTime = m.rollingShutterLineTime;
					im.rollingShutterBands = m.rollingShutterBands;
					im.files = imagesDirName + "/" + label + "_%06d.png";
					camGt.rgb = im;
				}
				if (sim->hasDepth())
				{
					const auto& m = sim->depthModel();
					CameraImageGroundTruth im;
					im.camera = m.camera;
					im.pose = m.poseOnVehicle;
					im.clipMin = m.clipMin;
					im.clipMax = m.clipMax;
					im.noiseStd = m.noiseStd;
					im.isDepth = true;
					im.depthUnit = m.unit;
					im.depthErrors = m.errors;
					im.files = imagesDirName + "/" + label + "_%06d_depth.bin";
					camGt.depth = im;
				}
				groundTruth.cameras.push_back(camGt);

				gens.push_back(
					{label, t0, cam->sensor_period(),
					 [&, sim](double t) -> mrpt::obs::CObservation::Ptr
					 {
						 nImages++;
						 return sim->simulate(*cameraScene, traj, t);
					 },
					 camGt.clockOffset});
			}
			std::cout << "[mvsim-dataset-gen] Cameras: " << rgbSims.size() + depthSims.size()
					  << ", images in '" << imagesDir << "'.\n";
		}

		// Observations are generated in sampling time order (min-heap over
		// each generator's next sampling time), so that clock offsets never
		// change the order in which the streams draw from the shared RNG, and
		// written in stamp order through a reorder buffer: a buffered
		// observation is written once no later sample can have an earlier
		// stamp (exact clock ticks, then sampling time, stream index and
		// generation order). Without offsets, both orders are the same.
		const auto offsetTicks = [](const SensorGen& g)
		{ return mrpt::Clock::duration(std::llround(g.clockOffset * 1e7)); };
		mrpt::Clock::duration minOffset = mrpt::Clock::duration::max();
		for (const auto& g : gens)
		{
			minOffset = std::min(minOffset, offsetTicks(g));
		}
		using Entry = std::pair<double, size_t>;
		std::priority_queue<Entry, std::vector<Entry>, std::greater<>> pq;
		for (size_t i = 0; i < gens.size(); i++)
		{
			if (gens[i].nextTime <= t1 + 1e-9)
			{
				pq.push({gens[i].nextTime, i});
			}
		}
		using Pending = std::tuple<int64_t, double, size_t, size_t>;
		std::priority_queue<Pending, std::vector<Pending>, std::greater<>> pending;
		std::map<size_t, mrpt::obs::CObservation::Ptr> pendingObs;	// by generation order
		size_t nGenerated = 0;
		mrpt::system::CTicTac progressTimer;
		double lastProgressPrintTime = 0.0;
		while (!pq.empty())
		{
			const auto [t, idx] = pq.top();
			pq.pop();
			auto& g = gens[idx];
			auto obs = g.generate(t);
			if (obs)
			{
				if (g.clockOffset != 0)
				{
					// In exact clock ticks: epoch seconds in a double are coarser.
					obs->timestamp += offsetTicks(g);
				}
				pending.push({obs->timestamp.time_since_epoch().count(), t, idx, nGenerated});
				pendingObs[nGenerated++] = obs;
			}
			g.nextTime += g.period;
			if (g.nextTime <= t1 + 1e-9)
			{
				pq.push({g.nextTime, idx});
			}
			// Later samples are stamped at or after this bound:
			const int64_t bound =
				pq.empty()
					? std::numeric_limits<int64_t>::max()
					: (timeFromSeconds(pq.top().first) + minOffset).time_since_epoch().count();
			while (!pending.empty() && std::get<0>(pending.top()) < bound)
			{
				const auto it = pendingObs.find(std::get<3>(pending.top()));
				arch << *it->second;
				pendingObs.erase(it);
				pending.pop();
			}

			const double elapsed = progressTimer.Tac();
			if (elapsed - lastProgressPrintTime >= 0.2 || pq.empty())
			{
				lastProgressPrintTime = elapsed;
				const double ratio =
					pq.empty() ? 1.0
							   : (simDuration > 0 ? std::min(1.0, (t - t0) / simDuration) : 1.0);
				const double eta = ratio > 1e-6 ? elapsed * (1.0 - ratio) / ratio : 0.0;
				std::cout << "\r[mvsim-dataset-gen] " << mrpt::system::progress(ratio, 30) << " "
						  << static_cast<int>(100 * ratio) << "% ETA "
						  << mrpt::system::formatTimeInterval(eta) << "   " << std::flush;
			}
		}
		std::cout << "\n";
		if (imageWriter)
		{
			imageWriter->finish();
		}

		std::cout << "[mvsim-dataset-gen] Wrote " << nSweeps << " lidar sweep(s) from "
				  << lidars.size() << " sensor(s), " << nImuSamples << " IMU sample(s) from "
				  << imus.size() << " sensor(s), " << nGnss << " GNSS reading(s) from "
				  << gnssSensors.size() << " sensor(s), " << nImages << " camera frame(s) from "
				  << rgbSims.size() + depthSims.size() << " sensor(s), and " << nOdom
				  << " odometry sample(s) to '" << outputPath << "'.\n";

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

		groundTruth.worldFile = worldXmlPath;
		groundTruth.rawlogFile = fileNameOf(outputPath);
		groundTruth.vehicleGtFile = fileNameOf(gtBase);
		groundTruth.vehicle = vehicleName;
		groundTruth.baseFrame = egoVehicle->getBaseFrameId();
		groundTruth.shutter = shutterStr;
		groundTruth.tStart = t0;
		groundTruth.tEnd = t1;
		groundTruth.seed = seed;
		groundTruth.noiseless = noiseless;
		groundTruth.noiseScale = noiseScale;
		groundTruth.georef = hasGeoref ? &georef : nullptr;
		const std::string gtYaml = mrpt::system::fileNameChangeExtension(outputPath, "gt.yaml");
		writeGroundTruthYaml(gtYaml, groundTruth);
		std::cout << "[mvsim-dataset-gen] Wrote sensor ground truth to '" << gtYaml << "'.\n";

		const auto printTiming = [](const std::string& label, const CameraTiming& tm)
		{
			if (tm.frames == 0)
			{
				return;
			}
			const double n = static_cast<double>(tm.frames);
			std::cout << mrpt::format(
				"[mvsim-dataset-gen] Camera '%s': %zu frames, %.1f ms rendering and %.1f ms "
				"writing per frame.\n",
				label.c_str(), tm.frames, 1e3 * tm.render / n, 1e3 * tm.write / n);
		};
		for (size_t i = 0; i < rgbSims.size(); i++)
		{
			printTiming(cameras.at(i)->getName(), rgbSims[i]->timing());
		}
		for (size_t i = 0; i < depthSims.size(); i++)
		{
			printTiming(depthCameras.at(i)->getName(), depthSims[i]->timing());
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}

	return 0;
}
