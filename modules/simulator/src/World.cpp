/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TTwist2D.h>
#include <mrpt/system/filesystem.h>	 // filePathSeparatorsToNative()
#include <mvsim/World.h>
#include <mvsim/mvsim-msgs/GenericAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvGetPoseAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwist.pb.h>
#include <mvsim/mvsim-msgs/SrvSetControllerTwistAnswer.pb.h>
#include <mvsim/mvsim-msgs/SrvSetPose.pb.h>
#include <mvsim/mvsim-msgs/SrvSetPoseAnswer.pb.h>

#include <algorithm>  // count()
#include <map>
#include <stdexcept>

using namespace mvsim;
using namespace std;

// Default ctor: inits empty world.
World::World() : mrpt::system::COutputLogger("mvsim::World")
{
	this->clear_all();
}

// Dtor.
World::~World()
{
	if (m_gui_thread.joinable())
	{
		MRPT_LOG_DEBUG("Waiting for GUI thread to quit...");
		m_gui_thread_must_close = true;
		m_gui_thread.join();
		MRPT_LOG_DEBUG("GUI thread shut down successful.");
	}

	this->clear_all();
	m_box2d_world.reset();
}

// Resets the entire simulation environment to an empty world.
void World::clear_all()
{
	auto lck = mrpt::lockHelper(m_world_cs);

	// Reset params:
	m_simul_time = 0.0;

	// (B2D) World contents:
	// ---------------------------------------------
	m_box2d_world = std::make_unique<b2World>(b2Vec2_zero);

	// Define the ground body.
	b2BodyDef groundBodyDef;
	m_b2_ground_body = m_box2d_world->CreateBody(&groundBodyDef);

	// Clear lists of objs:
	// ---------------------------------------------
	m_vehicles.clear();
	m_world_elements.clear();
	m_blocks.clear();
}

/** Runs the simulation for a given time interval (in seconds) */
void World::run_simulation(double dt)
{
	const double t0 = mrpt::Clock::toDouble(mrpt::Clock::now());

	// Define start of simulation time:
	if (!m_simul_start_wallclock_time.has_value())
		m_simul_start_wallclock_time = t0;

	m_timlogger.registerUserMeasure("run_simulation.dt", dt);

	// sanity checks:
	ASSERT_(dt > 0);
	ASSERT_(m_simul_timestep > 0);

	// Run in time steps:
	const double end_time = m_simul_time + dt;
	// tolerance for rounding errors summing time steps
	const double timetol = 1e-6;
	while (m_simul_time < (end_time - timetol))
	{
		// Timestep: always "simul_step" for the sake of repeatibility,
		// except if requested to run a shorter step:
		const double remainingTime = end_time - m_simul_time;
		if (remainingTime < 0) break;

		internal_one_timestep(
			remainingTime > m_simul_timestep ? m_simul_timestep
											 : remainingTime);
	}

	const double t1 = mrpt::Clock::toDouble(mrpt::Clock::now());

	m_timlogger.registerUserMeasure("run_simulation.cpu_dt", t1 - t0);
}

/** Runs one individual time step */
void World::internal_one_timestep(double dt)
{
	std::lock_guard<std::mutex> lck(m_simulationStepRunningMtx);

	m_timer_iteration.Tic();

	TSimulContext context;
	context.world = this;
	context.b2_world = m_box2d_world.get();
	context.simul_time = m_simul_time;
	context.dt = dt;

	// 1) Pre-step
	{
		mrpt::system::CTimeLoggerEntry tle(m_timlogger, "timestep.0.prestep");

		for (auto& e : m_simulableObjects)
			if (e.second) e.second->simul_pre_timestep(context);
	}

	// 2) Run dynamics
	{
		mrpt::system::CTimeLoggerEntry tle(
			m_timlogger, "timestep.1.dynamics_integrator");

		m_box2d_world->Step(dt, m_b2d_vel_iters, m_b2d_pos_iters);
		m_simul_time += dt;	 // Avance time
	}

	// 3) Save dynamical state and post-step processing:
	{
		mrpt::system::CTimeLoggerEntry tle(
			m_timlogger, "timestep.3.save_dynstate");

		const auto lckPhys = mrpt::lockHelper(physical_objects_mtx());

		for (auto& e : m_simulableObjects)
			if (e.second) e.second->simul_post_timestep(context);
	}

	// 4) Wait for 3D sensors (OpenGL raytrace) to get executed on its thread:
	mrpt::system::CTimeLoggerEntry tle4(
		m_timlogger, "timestep.4.wait_3D_sensors");
	if (pending_running_sensors_on_3D_scene())
	{
		for (int i = 0; i < 1000 && pending_running_sensors_on_3D_scene(); i++)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
		if (pending_running_sensors_on_3D_scene())
		{
#if 1
			MRPT_LOG_WARN(
				"Timeout waiting for async sensors to be simulated in opengl "
				"thread.");
#endif
			m_timlogger.registerUserMeasure("timestep.timeout_3D_sensors", 1.0);
		}
	}
	tle4.stop();

	const double ts = m_timer_iteration.Tac();
	m_timlogger.registerUserMeasure("timestep", ts);
	if (ts > dt) m_timlogger.registerUserMeasure("timestep.too_slow_alert", ts);
}

std::string World::xmlPathToActualPath(const std::string& modelURI) const
{
	std::string localFileName;
	if (modelURI.substr(0, 7) == "http://" ||
		modelURI.substr(0, 8) == "https://")
	{
		MRPT_TODO("Retrieve models from online sources");
		THROW_EXCEPTION("To do: online models");
		// localFileName = xx;
	}
	else if (modelURI.substr(0, 7) == "file://")
	{
		localFileName = modelURI.substr(7);
	}
	else
		localFileName = modelURI;

	return resolvePath(localFileName);
}

/** Replace macros, prefix the base_path if input filename is relative, etc.
 */
std::string World::resolvePath(const std::string& s_in) const
{
	std::string ret;
	const std::string s = mrpt::system::trim(s_in);

	// Relative path? It's not if:
	// "X:\*", "/*"
	// -------------------
	bool is_relative = true;
	if (s.size() > 2 && s[1] == ':' && (s[2] == '/' || s[2] == '\\'))
		is_relative = false;
	if (s.size() > 0 && (s[0] == '/' || s[0] == '\\')) is_relative = false;
	if (is_relative)
	{
		ret = m_base_path;
		if (!ret.empty() && ret[ret.size() - 1] != '/' &&
			ret[ret.size() - 1] != '\\')
			ret += string("/");
		ret += s;
	}
	else
		ret = s;

	return mrpt::system::filePathSeparatorsToNative(ret);
}

void World::runVisitorOnVehicles(const vehicle_visitor_t& v)
{
	for (auto& veh : m_vehicles)
		if (veh.second) v(*veh.second);
}

void World::runVisitorOnWorldElements(const world_element_visitor_t& v)
{
	for (auto& we : m_world_elements)
		if (we) v(*we);
}

void World::runVisitorOnBlocks(const block_visitor_t& v)
{
	for (auto& b : m_blocks)
		if (b.second) v(*b.second);
}

void World::connectToServer()
{
	//
	m_client.setVerbosityLevel(this->getMinLoggingLevel());
	m_client.serverHostAddress(m_server_address);
	m_client.connect();

	// Let objects register topics / services:
	for (auto& o : m_simulableObjects)
	{
		ASSERT_(o.second);
		o.second->registerOnServer(m_client);
	}

	// global services:
	m_client
		.advertiseService<mvsim_msgs::SrvSetPose, mvsim_msgs::SrvSetPoseAnswer>(
			"set_pose",
			std::function<mvsim_msgs::SrvSetPoseAnswer(
				const mvsim_msgs::SrvSetPose&)>(
				[this](const mvsim_msgs::SrvSetPose& req) {
					std::lock_guard<std::mutex> lck(m_simulationStepRunningMtx);

					mvsim_msgs::SrvSetPoseAnswer ans;
					ans.set_objectisincollision(false);

					const auto sId = req.objectid();

					if (auto itV = m_simulableObjects.find(sId);
						itV != m_simulableObjects.end())
					{
						if (req.has_relativeincrement() &&
							req.relativeincrement())
						{
							auto p =
								mrpt::poses::CPose3D(itV->second->getPose());
							p = p + mrpt::poses::CPose3D(
										req.pose().x(), req.pose().y(),
										req.pose().z(), req.pose().yaw(),
										req.pose().pitch(), req.pose().roll());
							itV->second->setPose(p.asTPose());

							auto* absPose = ans.mutable_objectglobalpose();
							absPose->set_x(p.x());
							absPose->set_y(p.y());
							absPose->set_z(p.z());
							absPose->set_yaw(p.yaw());
							absPose->set_pitch(p.pitch());
							absPose->set_roll(p.roll());
						}
						else
						{
							itV->second->setPose(
								{req.pose().x(), req.pose().y(), req.pose().z(),
								 req.pose().yaw(), req.pose().pitch(),
								 req.pose().roll()});
						}
						ans.set_success(true);
						ans.set_objectisincollision(
							itV->second->hadCollision());
						itV->second->resetCollisionFlag();
					}
					else
					{
						ans.set_success(false);
					}
					return ans;
				}));

	m_client
		.advertiseService<mvsim_msgs::SrvGetPose, mvsim_msgs::SrvGetPoseAnswer>(
			"get_pose",
			std::function<mvsim_msgs::SrvGetPoseAnswer(
				const mvsim_msgs::SrvGetPose&)>(
				[this](const mvsim_msgs::SrvGetPose& req) {
					std::lock_guard<std::mutex> lck(m_simulationStepRunningMtx);

					mvsim_msgs::SrvGetPoseAnswer ans;
					const auto sId = req.objectid();
					ans.set_objectisincollision(false);

					if (auto itV = m_simulableObjects.find(sId);
						itV != m_simulableObjects.end())
					{
						ans.set_success(true);
						const mrpt::math::TPose3D p = itV->second->getPose();
						auto* po = ans.mutable_pose();
						po->set_x(p.x);
						po->set_y(p.y);
						po->set_z(p.z);
						po->set_yaw(p.yaw);
						po->set_pitch(p.pitch);
						po->set_roll(p.roll);

						const auto t = itV->second->getTwist();
						auto* tw = ans.mutable_twist();
						tw->set_vx(t.vx);
						tw->set_vy(t.vy);
						tw->set_vz(0);
						tw->set_wx(0);
						tw->set_wy(0);
						tw->set_wz(t.omega);

						ans.set_objectisincollision(
							itV->second->hadCollision());
						itV->second->resetCollisionFlag();
					}
					else
					{
						ans.set_success(false);
					}
					return ans;
				}));

	m_client.advertiseService<
		mvsim_msgs::SrvSetControllerTwist,
		mvsim_msgs::SrvSetControllerTwistAnswer>(
		"set_controller_twist",
		std::function<mvsim_msgs::SrvSetControllerTwistAnswer(
			const mvsim_msgs::SrvSetControllerTwist&)>(
			[this](const mvsim_msgs::SrvSetControllerTwist& req) {
				std::lock_guard<std::mutex> lck(m_simulationStepRunningMtx);

				mvsim_msgs::SrvSetControllerTwistAnswer ans;
				ans.set_success(false);

				const auto sId = req.objectid();

				auto itV = m_simulableObjects.find(sId);
				if (itV == m_simulableObjects.end())
				{
					ans.set_errormessage("objectId not found");
					return ans;
				}

				auto veh = std::dynamic_pointer_cast<VehicleBase>(itV->second);
				if (!veh)
				{
					ans.set_errormessage("objectId is not of VehicleBase type");
					return ans;
				}

				mvsim::ControllerBaseInterface* controller =
					veh->getControllerInterface();
				if (!controller)
				{
					ans.set_errormessage(
						"objectId vehicle seems not to have any controller");
					return ans;
				}

				const mrpt::math::TTwist2D t(
					req.twistsetpoint().vx(), req.twistsetpoint().vy(),
					req.twistsetpoint().wz());

				const bool ctrlAcceptTwist = controller->setTwistCommand(t);
				if (!ctrlAcceptTwist)
				{
					ans.set_errormessage(
						"objectId vehicle controller did not accept the twist "
						"command");
					return ans;
				}

				ans.set_success(true);
				return ans;
			}));
}

void World::insertBlock(const Block::Ptr& block)
{
	// Assign each block an "index" number
	block->setBlockIndex(m_blocks.size());

	// make sure the name is not duplicated:
	m_blocks.insert(BlockList::value_type(block->getName(), block));
	m_simulableObjects.insert(
		m_simulableObjects.end(),
		std::make_pair(
			block->getName(), std::dynamic_pointer_cast<Simulable>(block)));
}
