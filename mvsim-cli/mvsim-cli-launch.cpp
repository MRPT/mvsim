/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/core/round.h>
#include <mrpt/system/os.h>	 // consoleColorAndStyle()
#include <mvsim/World.h>

#include <csignal>	// sigaction
#include <rapidxml_utils.hpp>
#include <thread>

#if defined(WIN32)
#include <windows.h>  // SetConsoleCtrlHandler
MRPT_TODO("win32: add SetConsoleCtrlHandler");
#endif

#include "mvsim-cli.h"

struct TThreadParams
{
	mvsim::World* world = nullptr;
	std::mutex closingMtx;

	TThreadParams() = default;

	bool isClosing()
	{
		closingMtx.lock();
		bool ret = closing_;
		closingMtx.unlock();
		return ret;
	}
	void closing(bool v)
	{
		closingMtx.lock();
		closing_ = v;
		closingMtx.unlock();
	}

   private:
	bool closing_ = false;
};

static void mvsim_server_thread_update_GUI(TThreadParams& thread_params);
static void mvsim_server_thread_headless(TThreadParams& thread_params);

mvsim::World::GUIKeyEvent gui_key_events;
std::mutex gui_key_events_mtx;
std::string msg2gui;

struct LaunchData
{
	mvsim::World world;
	TThreadParams thread_params;
	std::thread thGUI;
	size_t teleopIdxVeh = 0;  // Index of the vehicle to teleop
};

std::optional<LaunchData> app;

void mvsim_launch_shutdown()
{
	app->thread_params.closing(true);

	if (app->thGUI.joinable()) app->thGUI.join();

	// save full profiling, if enabled:
	if (app->world.getTimeLogger().isEnabledKeepWholeHistory())
	{
		const std::string sFil = "mvsim_profiler.m";
		std::cout << "\n***SAVING PROFILER DATA TO***: " << sFil << std::endl;
		app->world.getTimeLogger().saveToMFile(sFil);
	}

	app->world.free_opengl_resources();
	app.reset();  // destroy all
}

void mvsim_signal_handler(int s)
{
	std::cerr << "Caught signal " << s << ". Shutting down..." << std::endl;
	mvsim_launch_shutdown();
	exit(0);
}

void mvsim_install_signal_handler()
{
	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = &mvsim_signal_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, nullptr);
}

// returns log strings
std::string mvsim_launch_handle_teleop(
	const mvsim::World::GUIKeyEvent keyevent, const std::optional<mvsim::TJoyStickEvent>& js)
{
	using namespace mvsim;

	std::string txt2gui_tmp;

	const World::VehicleList& vehs = app->world.getListOfVehicles();
	txt2gui_tmp += mrpt::format(
		"Selected vehicle: %u/%u", static_cast<unsigned>(app->teleopIdxVeh + 1),
		static_cast<unsigned>(vehs.size()));

	if (app->teleopIdxVeh >= vehs.size()) return txt2gui_tmp;

	// Get iterator to selected vehicle:
	World::VehicleList::const_iterator it_veh = vehs.begin();
	std::advance(it_veh, app->teleopIdxVeh);

	auto& veh = *it_veh->second;

	// is it logging?
	if (veh.isLogging())
		txt2gui_tmp += " (LOGGING)\n";
	else
		txt2gui_tmp += "\n";

	// Get speed: ground truth
	{
		const mrpt::math::TTwist2D& vel = veh.getVelocityLocal();
		txt2gui_tmp += mrpt::format(
			"gt. vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg/s\n", vel.vx, vel.vy,
			mrpt::RAD2DEG(vel.omega));
	}
	// Get speed: ground truth
	{
		const mrpt::math::TTwist2D& vel = veh.getVelocityLocalOdoEstimate();
		txt2gui_tmp += mrpt::format(
			"odo vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg/s\n", vel.vx, vel.vy,
			mrpt::RAD2DEG(vel.omega));
	}

	// Generic teleoperation interface for any controller that
	// supports it:
	{
		ControllerBaseInterface* controller = veh.getControllerInterface();
		ControllerBaseInterface::TeleopInput teleop_in;
		ControllerBaseInterface::TeleopOutput teleop_out;
		teleop_in.keycode = keyevent.keycode;
		teleop_in.js = js;
		controller->teleop_interface(teleop_in, teleop_out);
		txt2gui_tmp += teleop_out.append_gui_lines;
	}

	return txt2gui_tmp;
}

int launchSimulation()
{
	using namespace mvsim;

	// check args:
	bool badArgs = false;
	const auto& unlabeledArgs = cli->argCmd.getValue();
	if (unlabeledArgs.size() != 2) badArgs = true;

	if (cli->argHelp.isSet() || badArgs)
	{
		fprintf(
			stdout,
			R"XXX(Usage: mvsim launch <WORLD_MODEL.xml> [options]

Available options:
 --headless              Launch without GUI (e.g. suitable for dockerized envs.)
 --full-profiler         Enable full profiling (generates file with all timings)
 --realtime-factor <1.0> Run slower (<1) or faster (>1) than real time if !=1.0
 -v, --verbosity         Set verbosity level: DEBUG, INFO (default), WARN, ERROR
)XXX");
		return 0;
	}

	// Handle CTRL+C:
	mvsim_install_signal_handler();

	const auto verbosityLevel = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
		cli->argVerbosity.getValue());

	if (verbosityLevel <= mrpt::system::LVL_INFO)
	{
		mrpt::system::consoleColorAndStyle(mrpt::system::ConsoleForegroundColor::BRIGHT_YELLOW);
		std::cout  //
			<< "\n"
			<< "====================================================\n"
			<< " MVSIM simulator running. Press CTRL+C to end.      \n"
			<< "====================================================\n"
			<< "\n";
		mrpt::system::consoleColorAndStyle(mrpt::system::ConsoleForegroundColor::DEFAULT);
	}

	const auto sXMLfilename = unlabeledArgs.at(1);

	app.emplace();

	app->world.setMinLoggingLevel(verbosityLevel);

	// CLI flags:
	if (cli->argFullProfiler.isSet()) app->world.getTimeLogger().enableKeepWholeHistory();

	if (cli->argHeadless.isSet()) app->world.headless(true);

	// Load from XML:
	try
	{
		rapidxml::file<> fil_xml(sXMLfilename.c_str());
		app->world.load_from_XML(fil_xml.data(), sXMLfilename.c_str());
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		mvsim_launch_shutdown();
		return 1;
	}

	// Start network server:
	commonLaunchServer();

	// Attach world as a mvsim communications node:
	app->world.connectToServer();

	// Launch GUI thread, unless we are in headless mode:
	app->thread_params.world = &app->world;

	if (!cli->argHeadless.isSet())
	{
		// regular GUI:
		app->thGUI = std::thread(&mvsim_server_thread_update_GUI, std::ref(app->thread_params));
	}
	else
	{
		// headless thread for off-screen rendering sensors:
		app->thGUI = std::thread(&mvsim_server_thread_headless, std::ref(app->thread_params));
	}

	// Run simulation:
	const double tAbsInit = mrpt::Clock::nowDouble();
	const double rtFactor = cli->argRealTimeFactor.getValue();
	bool doExit = false;

	while (!doExit)
	{
		// was the quit button hit in the GUI?
		if (app->world.simulator_must_close()) break;

		// Simulation
		// ============================================================
		// Compute how much time has passed to simulate in real-time:
		double tNew = mrpt::Clock::nowDouble();
		double incrTime = rtFactor * (tNew - tAbsInit) - app->world.get_simul_time();
		int incrTimeSteps =
			static_cast<int>(std::floor(incrTime / app->world.get_simul_timestep()));

		// Simulate:
		if (incrTimeSteps > 0)
		{
			app->world.run_simulation(incrTimeSteps * app->world.get_simul_timestep());
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		// GUI msgs, teleop, etc.
		// ====================================================
		gui_key_events_mtx.lock();
		World::GUIKeyEvent keyevent = gui_key_events;
		gui_key_events_mtx.unlock();

		// Global keys:
		switch (keyevent.keycode)
		{
			case GLFW_KEY_ESCAPE:
				doExit = true;
				break;
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
				app->teleopIdxVeh = keyevent.keycode - '1';
				break;
		};

		const auto js = app->world.getJoystickState();

		const auto txt2gui_tmp = mvsim_launch_handle_teleop(keyevent, js);

		// Clear the keystroke buffer
		gui_key_events_mtx.lock();
		if (keyevent.keycode != 0) gui_key_events = World::GUIKeyEvent();
		gui_key_events_mtx.unlock();

		msg2gui = txt2gui_tmp;	// send txt msgs to show in the GUI

		if (app->thread_params.isClosing()) doExit = true;

	}  // end while()

	mvsim_launch_shutdown();

	return 0;
}

void mvsim_server_thread_update_GUI(TThreadParams& thread_params)
{
	try
	{
		ASSERT_(thread_params.world);
		while (!thread_params.isClosing())
		{
			mvsim::World::TUpdateGUIParams guiparams;
			guiparams.msg_lines = msg2gui;

			thread_params.world->update_GUI(&guiparams);

			// Send key-strokes to the main thread:
			if (guiparams.keyevent.keycode != 0)
			{
				gui_key_events_mtx.lock();
				gui_key_events = guiparams.keyevent;
				gui_key_events_mtx.unlock();
			}

			std::this_thread::sleep_for(std::chrono::milliseconds(25));
		}
	}
	catch (const std::exception& e)
	{
		std::cerr << "[mvsim_server_thread_update_GUI] Exception: " << e.what() << std::endl;
	}
}

void mvsim_server_thread_headless(TThreadParams& thread_params)
{
	try
	{
		ASSERT_(thread_params.world);
		while (!thread_params.isClosing() && !thread_params.world->simulator_must_close())
		{
			thread_params.world->internalGraphicsLoopTasksForSimulation();

			std::this_thread::sleep_for(std::chrono::microseconds(
				mrpt::round(thread_params.world->get_simul_timestep() * 1000000)));
		}

		// in case we are here due to simulator_must_close()
		thread_params.closing(true);
	}
	catch (const std::exception& e)
	{
		std::cerr << "[mvsim_server_thread_update_GUI] Exception: " << e.what() << std::endl;
	}
}
