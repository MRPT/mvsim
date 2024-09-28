/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/tfest.h>	 // least-squares methods
#include <mrpt/tfest/TMatchingPair.h>
#include <mrpt/version.h>
#include <mvsim/World.h>

using namespace mvsim;
using namespace std;

/** Runs the simulation for a given time interval (in seconds) */
void World::run_simulation(double dt)
{
	ASSERT_(initialized_);

	const double t0 = mrpt::Clock::nowDouble();

	// Define start of simulation time:
	if (!simul_start_wallclock_time_.has_value()) simul_start_wallclock_time_ = t0 - dt;

	timlogger_.registerUserMeasure("run_simulation.dt", dt);

	const double simulTimestep = get_simul_timestep();

	// sanity checks:
	ASSERT_(dt > 0);
	ASSERT_(simulTimestep > 0);

	// Run in time steps:
	const double end_time = get_simul_time() + dt;
	// tolerance for rounding errors summing time steps
	const double timetol = 1e-4;
	while (get_simul_time() < (end_time - timetol))
	{
		// Timestep: always "simul_step" for the sake of repeatibility,
		// except if requested to run a shorter step:
		const double remainingTime = end_time - get_simul_time();
		if (remainingTime <= 0) break;

		internal_one_timestep(remainingTime >= simulTimestep ? simulTimestep : remainingTime);

		// IMPORTANT: This must be inside the loop to allow breaking if we are
		// closing the app and simulatedTime is not ticking anymore.
		if (simulator_must_close()) break;
	}

	const double t1 = mrpt::Clock::toDouble(mrpt::Clock::now());

	timlogger_.registerUserMeasure("run_simulation.cpu_dt", t1 - t0);
}

/** Runs one individual time step */
void World::internal_one_timestep(double dt)
{
	if (simulator_must_close()) return;

	std::lock_guard<std::mutex> lck(simulationStepRunningMtx_);

	timer_iteration_.Tic();

	TSimulContext context;
	context.world = this;
	context.b2_world = box2d_world_.get();
	context.simul_time = get_simul_time();
	context.dt = dt;

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());

	// 1) Pre-step
	{
		mrpt::system::CTimeLoggerEntry tle(timlogger_, "timestep.1.prestep");
		for (auto& e : simulableObjects_)
			if (e.second) e.second->simul_pre_timestep(context);
	}
	// 2) vehicles terrain elevation
	{
		mrpt::system::CTimeLoggerEntry tle(timlogger_, "timestep.2.terrain_elevation");
		internal_simul_pre_step_terrain_elevation();
	}

	// 3) Run dynamics
	{
		mrpt::system::CTimeLoggerEntry tle(timlogger_, "timestep.3.dynamics_integrator");

		box2d_world_->Step(dt, b2dVelIters_, b2dPosIters_);

		// Move time forward:
		auto lckSimTim = mrpt::lockHelper(simul_time_mtx_);
		simulTime_ += dt;
	}

	// 4) Save dynamical state and post-step processing:
	// This also makes a copy of all objects dynamical state, so service calls
	// can be answered straight away without waiting for the main simulation
	// mutex:
	{
		mrpt::system::CTimeLoggerEntry tle(timlogger_, "timestep.4.save_dynstate");

		const auto lckPhys = mrpt::lockHelper(physical_objects_mtx());
		const auto lckCopy = mrpt::lockHelper(copy_of_objects_dynstate_mtx_);

		for (auto& e : simulableObjects_)
		{
			if (!e.second) continue;
			// process:
			e.second->simul_post_timestep(context);

			// save our own copy of the kinematic state:
			copy_of_objects_dynstate_pose_[e.first] = e.second->getPose();
			copy_of_objects_dynstate_twist_[e.first] = e.second->getTwist();

			if (e.second->hadCollision()) copy_of_objects_had_collision_.insert(e.first);
		}
	}
	{
		const auto lckCollis = mrpt::lockHelper(reset_collision_flags_mtx_);
		for (const auto& sId : reset_collision_flags_)
		{
			if (auto itV = simulableObjects_.find(sId); itV != simulableObjects_.end())
				itV->second->resetCollisionFlag();
		}
		reset_collision_flags_.clear();
	}
	lckListObjs.unlock();  // for simulableObjects_

	// 5) Wait for 3D sensors (OpenGL raytrace) to get executed on its thread:
	mrpt::system::CTimeLoggerEntry tle4(timlogger_, "timestep.5.wait_3D_sensors");
	if (pending_running_sensors_on_3D_scene())
	{
		// Use a huge timeout here to avoid timing out in build farms / cloud
		// containers:
		for (int i = 0; i < 20000 && pending_running_sensors_on_3D_scene(); i++)
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

		if (pending_running_sensors_on_3D_scene())
		{
			MRPT_LOG_WARN(
				"Timeout waiting for async sensors to be simulated in opengl "
				"thread.");
			timlogger_.registerUserMeasure("timestep.timeout_3D_sensors", 1.0);
		}
	}
	tle4.stop();

	// 6) If we have .rawlog generation enabled, process odometry, etc.
	mrpt::system::CTimeLoggerEntry tle5(timlogger_, "timestep.6.post_rawlog");

	internalPostSimulStepForRawlog();
	internalPostSimulStepForTrajectory();

	tle5.stop();

	const double ts = timer_iteration_.Tac();
	timlogger_.registerUserMeasure("timestep", ts);
	if (ts > dt) timlogger_.registerUserMeasure("timestep.too_slow_alert", ts);
}

double World::get_simul_timestep() const
{
	ASSERT_GE_(simulTimestep_, .0);
	static bool firstTimeCheck = true;

	auto lambdaMinimumSensorPeriod = [this]() -> std::optional<double>
	{
		std::optional<double> ret;
		for (const auto& veh : vehicles_)
		{
			if (!veh.second) continue;
			for (const auto& s : veh.second->getSensors())
			{
				const double T = s->sensor_period();
				if (ret)
					mrpt::keep_min(*ret, T);
				else
					ret = T;
			}
		}
		return ret;
	};

	if (simulTimestep_ == 0)
	{
		// `0` means auto-determine as the minimum of 5 ms and the shortest
		// sensor sample period.
		simulTimestep_ = 5e-3;

		auto sensorMinPeriod = lambdaMinimumSensorPeriod();
		if (sensorMinPeriod) mrpt::keep_min(simulTimestep_, *sensorMinPeriod);

		MRPT_LOG_INFO_FMT(
			"Physics simulation timestep automatically determined as: %.02f ms",
			1e3 * simulTimestep_);

		firstTimeCheck = false;	 // no need to check.
	}
	else if (firstTimeCheck)
	{
		firstTimeCheck = false;
		auto sensorMinPeriod = lambdaMinimumSensorPeriod();
		if (sensorMinPeriod && simulTimestep_ > *sensorMinPeriod)
		{
			MRPT_LOG_WARN_FMT(
				"Physics simulation timestep (%.02f ms) should be >= than the "
				"minimum sensor period (%.02f ms) to avoid missing sensor "
				"readings!!",
				1e3 * simulTimestep_, 1e3 * *sensorMinPeriod);
		}
	}

	return simulTimestep_;
}

void World::internalPostSimulStepForRawlog()
{
	if (save_to_rawlog_.empty()) return;

	ASSERT_GT_(rawlog_odometry_rate_, 0.0);

	const double now = get_simul_time();
	const double T_odom = 1.0 / rawlog_odometry_rate_;

	if (rawlog_last_odom_time_.has_value() && (now - *rawlog_last_odom_time_) < T_odom)
	{
		return;	 // not yet
	}

	rawlog_last_odom_time_ = now;

	// Create one observation per robot:
	for (const auto& veh : vehicles_)
	{
		auto obs = mrpt::obs::CObservationOdometry::Create();
		obs->timestamp = get_simul_timestamp();
		obs->sensorLabel = "odom";
		obs->odometry = veh.second->getCPose2D();

		obs->hasVelocities = true;
		obs->velocityLocal = veh.second->getVelocityLocal();

		// TODO: Simul noisy odometry and odom velocity

		internalOnObservation(*veh.second, obs);
	}
}

void World::internalPostSimulStepForTrajectory()
{
	using namespace std::string_literals;

	if (save_ground_truth_trajectory_.empty()) return;

	ASSERT_GT_(ground_truth_rate_, 0.0);

	const double now = get_simul_time();
	const double T_odom = 1.0 / ground_truth_rate_;

	if (gt_last_time_.has_value() && (now - *gt_last_time_) < T_odom)
	{
		return;	 // not yet
	}

	gt_last_time_ = now;

	// Create one entry per robot.
	// First, create the output files if this is the first time:
	auto lck = mrpt::lockHelper(gt_io_mtx_);
	if (gt_io_per_veh_.empty())
	{
		for (const auto& [vehName, veh] : vehicles_)
		{
			const std::string fileName = mrpt::system::fileNameChangeExtension(
				save_ground_truth_trajectory_, vehName + ".txt"s);

			MRPT_LOG_INFO_STREAM("Creating ground truth file: " << fileName);

			gt_io_per_veh_[vehName] = std::fstream(fileName, std::ios::out);
		}
	}

	const double t = mrpt::Clock::toDouble(get_simul_timestamp());

	for (const auto& [vehName, veh] : vehicles_)
	{
		const auto p = mrpt::poses::CPose3DQuat(veh->getCPose3D());

		// each row contains these elements separated by spaces:
		// timestamp x y z q_x q_y q_z q_w

		gt_io_per_veh_.at(vehName) << mrpt::format(
			"%f %f %f %f %f %f %f %f\n", t, p.x(), p.y(), p.z(), p.quat().x(), p.quat().y(),
			p.quat().z(), p.quat().w());
	}
}

void World::internal_simul_pre_step_terrain_elevation()
{
	// For each vehicle:
	// 1) Compute its 3D pose according to the mesh tilt angle.
	// 2) Apply gravity force
	const double gravity = get_gravity();

	mrpt::tfest::TMatchingPairList corrs;
	mrpt::poses::CPose3D optimalTf;

	const World::VehicleList& lstVehs = getListOfVehicles();
	for (auto& [name, veh] : lstVehs)
	{
		getTimeLogger().enter("elevationmap.handle_vehicle");

		const size_t nWheels = veh->getNumWheels();

		// 1) Compute its 3D pose according to the mesh tilt angle.
		// Idea: run a least-squares method to find the best
		// SE(3) transformation that map the wheels contact point,
		// as seen in local & global coordinates.
		// (For large tilt angles, may have to run it iteratively...)
		// -------------------------------------------------------------
		// the final downwards direction (unit vector (0,0,-1)) as seen in
		// vehicle local frame.
		mrpt::math::TPoint3D dir_down;
		for (int iter = 0; iter < 2; iter++)
		{
			const mrpt::math::TPose3D& cur_pose = veh->getPose();
			// This object is faster for repeated point projections
			const mrpt::poses::CPose3D cur_cpose(cur_pose);

			mrpt::math::TPose3D new_pose = cur_pose;
			corrs.clear();

			bool out_of_area = false;
			for (size_t iW = 0; !out_of_area && iW < nWheels; iW++)
			{
				const Wheel& wheel = veh->getWheelInfo(iW);

				// Local frame
				mrpt::tfest::TMatchingPair corr;

				corr.localIdx = iW;
				corr.local = mrpt::math::TPoint3D(wheel.x, wheel.y, 0);

				// Global frame
				const mrpt::math::TPoint3D gPt = cur_cpose.composePoint({wheel.x, wheel.y, 0.0});

				const mrpt::math::TPoint3D gPtWheelsAxis =
					gPt + mrpt::math::TPoint3D(.0, .0, .5 * wheel.diameter);

				// Get "the ground" under my wheel axis:
				const auto z = this->getHighestElevationUnder(gPtWheelsAxis);
				if (!z.has_value())
				{
					out_of_area = true;
					continue;  // vehicle is out of bounds!
				}

				corr.globalIdx = iW;
				corr.global = mrpt::math::TPoint3D(gPt.x, gPt.y, *z);

				corrs.push_back(corr);
			}
			if (out_of_area) continue;

			// Register:
			double transf_scale;
			mrpt::poses::CPose3DQuat tmpl;

			mrpt::tfest::se3_l2(corrs, tmpl, transf_scale, true /*force scale unity*/);

			optimalTf = mrpt::poses::CPose3D(tmpl);

			new_pose.z = optimalTf.z();
			new_pose.yaw = optimalTf.yaw();
			new_pose.pitch = optimalTf.pitch();
			new_pose.roll = optimalTf.roll();

			veh->setPose(new_pose);

		}  // end iters

#if 0
		// debug contact points:
		if (debugShowContactPoints_)
		{
			gl_debugWheelsContactPoints_->clear();
			for (const auto& c : corrs_) gl_debugWheelsContactPoints_->insertPoint(c.global);
		}
#endif

		// compute "down" direction:
		{
			mrpt::poses::CPose3D rot_only;
			rot_only.setRotationMatrix(optimalTf.getRotationMatrix());
			rot_only.inverseComposePoint(.0, .0, -1.0, dir_down.x, dir_down.y, dir_down.z);
		}

		// 2) Apply gravity force
		// -------------------------------------------------------------
		{
			// To chassis:
			const double chassis_weight = veh->getChassisMass() * gravity;
			const mrpt::math::TPoint2D chassis_com = veh->getChassisCenterOfMass();
			veh->apply_force(
				{dir_down.x * chassis_weight, dir_down.y * chassis_weight}, chassis_com);

			// To wheels:
			for (size_t iW = 0; iW < nWheels; iW++)
			{
				const Wheel& wheel = veh->getWheelInfo(iW);
				const double wheel_weight = wheel.mass * gravity;
				veh->apply_force(
					{dir_down.x * wheel_weight, dir_down.y * wheel_weight}, {wheel.x, wheel.y});
			}
		}

		getTimeLogger().leave("elevationmap.handle_vehicle");
	}
}
