/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSinCosLookUpTableFor2DScans.h>
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
	// (1/2) Compute its 3D pose according to the mesh tilt angle.
	//       and apply gravity force.
	const double gravity = get_gravity();

	mrpt::tfest::TMatchingPairList corrs;
	mrpt::poses::CPose3D optimalTf;

	std::map<const VehicleBase*, std::vector<float>> wheel_heights_per_vehicle;

	const World::VehicleList& lstVehs = getListOfVehicles();
	for (auto& [name, veh] : lstVehs)
	{
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

			bool all_equal = true;
			std::optional<float> equal_zs;

			// Store wheels height for the posterior stage of collision detection:
			auto& wheelHeights = wheel_heights_per_vehicle[veh.get()];
			wheelHeights.resize(nWheels);

			for (size_t iW = 0; iW < nWheels; iW++)
			{
				const Wheel& wheel = veh->getWheelInfo(iW);

				// Local frame
				mrpt::tfest::TMatchingPair corr;

				corr.localIdx = iW;
				corr.local = mrpt::math::TPoint3D(wheel.x, wheel.y, 0);

				// Global frame
				const mrpt::math::TPoint3D gPt = cur_cpose.composePoint({wheel.x, wheel.y, 0.0});

				const mrpt::math::TPoint3D gPtWheelsAxis =
					gPt + mrpt::math::TPoint3D(.0, .0, 0.5 * wheel.diameter);

				// Get "the ground" under my wheel axis:
				const float z = this->getHighestElevationUnder(gPtWheelsAxis);

				wheelHeights[iW] = z;

				if (!equal_zs) equal_zs = z;
				if (std::abs(*equal_zs - z) > 1e-4) all_equal = false;

				corr.globalIdx = iW;
				corr.global = mrpt::math::TPoint3D(gPt.x, gPt.y, z);

				corrs.push_back(corr);
			}  // end for each Wheel

			if (all_equal && equal_zs.has_value())
			{
				// Optimization: just use the constant elevation without optimizing:
				new_pose.z = *equal_zs;
				new_pose.pitch = 0;
				new_pose.roll = 0;
			}
			else if (corrs.size() >= 3)
			{
				// Register:
				double transf_scale;
				mrpt::poses::CPose3DQuat tmpl;

				mrpt::tfest::se3_l2(corrs, tmpl, transf_scale, true /*force scale unity*/);

				optimalTf = mrpt::poses::CPose3D(tmpl);

				new_pose.z = optimalTf.z();
				new_pose.yaw = optimalTf.yaw();
				new_pose.pitch = optimalTf.pitch();
				new_pose.roll = optimalTf.roll();
			}

			veh->setPose(new_pose);

		}  // end iters

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
	}  // end for each vehicle

	// (2/2) Collisions due to steep slopes

	// Make a list of objects subject to collide with the occupancy grid:
	// - Vehicles
	// - Blocks
	const World::BlockList& lstBlocks = getListOfBlocks();
	const size_t nObjs = lstVehs.size() + lstBlocks.size();
	obstacles_for_each_obj_.resize(nObjs);
	size_t objIdx = 0;
	for (auto& [name, veh] : lstVehs)
	{
		auto& e = obstacles_for_each_obj_.at(objIdx);
		if (!e.has_value()) e.emplace();

		TInfoPerCollidableobj& ipv = e.value();
		ipv.pose = veh->getCPose3D();
		ipv.representativeHeight = 1.05 * veh->chassisZMax();
		ipv.contour = veh->getChassisShape();
		ipv.maxWorkableStepHeight = 0.55 * veh->getWheelInfo(0).diameter;

		const auto& tw = veh->getTwist();
		ipv.speed = mrpt::math::TVector2D(tw.vx, tw.vy).norm();

		if (auto it = wheel_heights_per_vehicle.find(veh.get());
			it != wheel_heights_per_vehicle.end())
		{
			ipv.wheel_heights = &it->second;
		}

		objIdx++;
	}

#if 0  // Do not process blocks for now...
	for (const auto& [name, block] : lstBlocks)
	{
		auto& e = obstacles_for_each_obj_.at(objIdx);
		objIdx++;

		// only for moving blocks:
		if (block->isStatic()) continue;
		const auto tw = block->getTwist();
		if (std::abs(tw.vx) < 1e-4 && std::abs(tw.vy) < 1e-4 && std::abs(tw.omega) < 1e-4) continue;

		if (!e.has_value()) e.emplace();

		TInfoPerCollidableobj& ipv = e.value();
		ipv.pose = block->getCPose3D();
		ipv.representativeHeight = 1.05 * block->block_z_max();
		ipv.contour = block->blockShape();
	}
#endif

	// For each object, create a ground body with fixtures at each potential collision point
	// around the vehicle, so it can collide with the environment:
	for (auto& e : obstacles_for_each_obj_)
	{
		if (!e.has_value()) continue;

		TInfoPerCollidableobj& ipv = e.value();

		ASSERT_(ipv.wheel_heights);
		// Get mean wheels elevation:
		float avrg_wheels_z = .0f;
		for (const auto z : *ipv.wheel_heights) avrg_wheels_z += z;
		avrg_wheels_z /= 1.0f * ipv.wheel_heights->size();

		ipv.contour_heights.assign(ipv.contour.size(), 0);
		// 1) get obstacles around the vehicle:
		for (size_t k = 0; k < ipv.contour.size(); k++)
		{
			const auto ptLocal = mrpt::math::TPoint3D(
				ipv.contour.at(k).x, ipv.contour.at(k).y, ipv.representativeHeight);
			const auto ptGlobal = ipv.pose.composePoint(ptLocal);
			const float z = getHighestElevationUnder(ptGlobal);
			ipv.contour_heights.at(k) = z;
		}
		// 2) Create a Box2D "ground body" with square "fixtures" so the
		// vehicle can collide with the occ. grid:

		// Create Box2D objects upon first usage:
		if (!ipv.collide_body)
		{
			b2BodyDef bdef;
			ipv.collide_body = getBox2DWorld()->CreateBody(&bdef);
			ASSERT_(ipv.collide_body);
		}
		// Force move the body to the vehicle origins (to use obstacles in
		// local coords):
		ipv.collide_body->SetTransform(b2Vec2(ipv.pose.x(), ipv.pose.y()), .0f);

		// Create fixtures at their place (or disable it if no obstacle has
		// been sensed):
		ipv.collide_fixtures.resize(ipv.contour.size());
		for (size_t k = 0; k < ipv.contour.size(); k++)
		{
			const double obsW = 0.01 + ipv.speed * 0.07;

			if (!ipv.collide_fixtures[k].fixture)
			{
				// Physical properties of each "obstacle":
				// make the size proportional to speed to avoid "going thru" walls:
				b2PolygonShape sqrPoly;
				sqrPoly.SetAsBox(obsW, obsW);
				sqrPoly.m_radius = 1e-3;  // The "skin" depth of the body
				b2FixtureDef fixtureDef;
				fixtureDef.shape = &sqrPoly;
				fixtureDef.restitution = 0.1f;	// restitution_;
				fixtureDef.density = 0;	 // Fixed (inf. mass)
				fixtureDef.friction = 0.5f;	 // lateral_friction_;  // 0.5f;

				ipv.collide_fixtures[k].fixture = ipv.collide_body->CreateFixture(&fixtureDef);
			}

			const bool is_collision =
				(ipv.contour_heights[k] - avrg_wheels_z) > ipv.maxWorkableStepHeight;

			if (!is_collision)
			{
				// Box2D's way of saying: don't collide with this!
				ipv.collide_fixtures[k].fixture->SetSensor(true);
				ipv.collide_fixtures[k].fixture->GetUserData().pointer =
					INVISIBLE_FIXTURE_USER_DATA;
			}
			else
			{
				ipv.collide_fixtures[k].fixture->SetSensor(false);
				ipv.collide_fixtures[k].fixture->GetUserData().pointer = 0;

				b2PolygonShape* poly =
					dynamic_cast<b2PolygonShape*>(ipv.collide_fixtures[k].fixture->GetShape());
				ASSERT_(poly != nullptr);

				const auto ptLocal = mrpt::math::TPoint3D(
					ipv.contour.at(k).x, ipv.contour.at(k).y, ipv.representativeHeight);
				auto ptGlobal = ipv.pose.composePoint(ptLocal);
				ptGlobal.x = mrpt::round(ptGlobal.x / obsW) * obsW;
				ptGlobal.y = mrpt::round(ptGlobal.y / obsW) * obsW;

				const float lx = ptGlobal.x - ipv.pose.x();
				const float ly = ptGlobal.y - ipv.pose.y();

				poly->SetAsBox(obsW, obsW, b2Vec2(lx, ly), .0f /*angle*/);
			}
		}
	}  // end for object
}

const World::LUTCache& World::getLUTCacheOfObjects() const
{
	if (!lut2d_objects_is_up_to_date_) internal_update_lut_cache();

	return lut2d_objects_;
}

World::lut_2d_coordinates_t World::xy_to_lut_coords(const mrpt::math::TPoint2Df& p)
{
	constexpr float LUT_GRID_SIZE = 4.0;
	World::lut_2d_coordinates_t c;
	c.x = static_cast<int32_t>(p.x / LUT_GRID_SIZE);
	c.y = static_cast<int32_t>(p.y / LUT_GRID_SIZE);
	return c;
}

void World::internal_update_lut_cache() const
{
	lut2d_objects_is_up_to_date_ = true;

	lut2d_objects_.clear();
	for (const auto& [name, obj] : blocks_)
	{
		std::set<lut_2d_coordinates_t, LutIndexHash> affected_coords;
		const auto p = obj->getCPose3D();
		for (const auto& vertex : obj->blockShape())
		{
			const auto pt = p.composePoint({vertex.x, vertex.y, .0});
			const auto c = xy_to_lut_coords(mrpt::math::TPoint2Df(pt.x, pt.y));
			affected_coords.insert(c);
		}
		for (const auto& c : affected_coords) lut2d_objects_[c].push_back(obj);
	}
}
