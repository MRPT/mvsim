/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>	 // kbhit()
#include <mvsim/Comms/Server.h>
#include <mvsim/Comms/ports.h>
#include <mvsim/World.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <rapidxml_utils.hpp>
#include <thread>

TCLAP::CmdLine cmd("mvsim", ' ', "version", false /* no --help */);

TCLAP::UnlabeledMultiArg<std::string> argCmd(
	"command", "Command to run. Run 'mvsim help' to list commands.", false, "",
	cmd);

TCLAP::ValueArg<std::string> argVerbosity(
	"v", "verbose", "Verbosity level", false, "INFO", "INFO", cmd);

TCLAP::SwitchArg argHelp(
	"h", "help", "Shows more detailed help for command", cmd);

TCLAP::ValueArg<int> argPort(
	"p", "port", "TCP port to listen at", false, mvsim::MVSIM_PORTNO_MAIN_REP,
	"TCP port", cmd);

using namespace mvsim;

// ======= Command handlers =======
using cmd_t = std::function<int(void)>;

static int printListCommands();	 // "help"
static int launchStandAloneServer();  // "server"
static int launchSimulation();	// "launch":

const std::map<std::string, cmd_t> cliCommands = {
	{"help", cmd_t(&printListCommands)},
	{"server", cmd_t(&launchStandAloneServer)},
	{"launch", cmd_t(&launchSimulation)},
};

int main(int argc, char** argv)
{
	try
	{
		if (!cmd.parse(argc, argv))
		{
			printListCommands();
			return 1;
		}

		// Take first unlabeled argument:
		std::string command;
		if (const auto& lst = argCmd.getValue(); !lst.empty())
			command = lst.at(0);

		// Look up command in table:
		auto itCmd = cliCommands.find(command);

		if (!argCmd.isSet() || itCmd == cliCommands.end())
		{
			mrpt::system::setConsoleColor(mrpt::system::CONCOL_RED);
			std::cerr << "Error: missing or unknown command.\n";
			mrpt::system::setConsoleColor(mrpt::system::CONCOL_NORMAL);
			printListCommands();
			return 1;
		}

		// Execute command:
		return (itCmd->second)();
	}
	catch (const std::exception& e)
	{
		std::cerr << "ERROR: " << mrpt::exception_to_str(e);
		return 1;
	}
	return 0;
}

int printListCommands()
{
	fprintf(
		stderr,
		R"XXX(mvsim: A lightweight multivehicle simulation environment.

Available commands:
    mvsim launch <WORLD.xml>  Start a comm. server and simulates a world.
    mvsim server              Start a standalone communication server.
    mvsim node                List connected nodes.
    mvsim topic               Show information on topics. 

Or use `mvsim <COMMAND> --help` for further options
)XXX");
	return 0;
}

static std::shared_ptr<mvsim::Server> server;

static void commonLaunchServer()
{
	ASSERT_(!server);

	// Start network server:
	server = std::make_shared<mvsim::Server>();

	if (argPort.isSet()) server->listenningPort(argPort.getValue());

	server->setMinLoggingLevel(
		mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
			argVerbosity.getValue()));

	server->start();
}

int launchStandAloneServer()
{
	if (argHelp.isSet())
	{
		fprintf(
			stdout,
			R"XXX(Usage: mvsim server

Available options:
  -p %5u, --port %5u   Listen on given TCP port.
  -v, --verbosity      Set verbosity level: DEBUG, INFO (default), WARN, ERROR
)XXX",
			mvsim::MVSIM_PORTNO_MAIN_REP, mvsim::MVSIM_PORTNO_MAIN_REP);
		return 0;
	}

	commonLaunchServer();
	return 0;
}

struct TThreadParams
{
	World* world;
	volatile bool closing;
	TThreadParams() : world(NULL), closing(false) {}
};
static void mvsim_server_thread_update_GUI(TThreadParams& thread_params);
World::TGUIKeyEvent gui_key_events;
std::string msg2gui;

int launchSimulation()
{
	// check args:
	bool badArgs = false;
	const auto& unlabeledArgs = argCmd.getValue();
	if (unlabeledArgs.size() != 2) badArgs = true;

	if (argHelp.isSet() || badArgs)
	{
		fprintf(
			stdout,
			R"XXX(Usage: mvsim launch <WORLD_MODEL.xml>

Available options:
  -v, --verbosity      Set verbosity level: DEBUG, INFO (default), WARN, ERROR
)XXX");
		return 0;
	}

	const auto sXMLfilename = unlabeledArgs.at(1);

	// Start network server:
	commonLaunchServer();

	mvsim::World world;

	world.setMinLoggingLevel(
		mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
			argVerbosity.getValue()));

	// Load from XML:
	rapidxml::file<> fil_xml(sXMLfilename.c_str());
	world.load_from_XML(fil_xml.data(), sXMLfilename.c_str());

	// Attach world as a mvsim communications node:
	world.connectToServer();

	// Launch GUI thread:
	TThreadParams thread_params;
	thread_params.world = &world;
	std::thread thGUI =
		std::thread(&mvsim_server_thread_update_GUI, std::ref(thread_params));

	// Run simulation:
	mrpt::system::CTicTac tictac;
	double t_old = tictac.Tac();
	double REALTIME_FACTOR = 1.0;
	bool do_exit = false;
	size_t teleop_idx_veh = 0;	// Index of the vehicle to teleop

	while (!do_exit && !mrpt::system::os::kbhit())
	{
		// Simulation
		// ============================================================
		// Compute how much time has passed to simulate in real-time:
		double t_new = tictac.Tac();
		double incr_time = REALTIME_FACTOR * (t_new - t_old);

		// Just in case the computer is *really fast*...
		if (incr_time >= world.get_simul_timestep())
		{
			// Simulate:
			world.run_simulation(incr_time);

			// t_old_simul = world.get_simul_time();
			t_old = t_new;
		}

		// I could use 10ms here but chono literals are since gcc 4.9.3
		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		// GUI msgs, teleop, etc.
		// ====================================================

		std::string txt2gui_tmp;
		World::TGUIKeyEvent keyevent = gui_key_events;

		// Global keys:
		switch (keyevent.keycode)
		{
			case GLFW_KEY_ESCAPE:
				do_exit = true;
				break;
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
				teleop_idx_veh = keyevent.keycode - '1';
				break;
		};

		{  // Test: Differential drive: Control raw forces
			const World::TListVehicles& vehs = world.getListOfVehicles();
			txt2gui_tmp += mrpt::format(
				"Selected vehicle: %u/%u\n",
				static_cast<unsigned>(teleop_idx_veh + 1),
				static_cast<unsigned>(vehs.size()));
			if (vehs.size() > teleop_idx_veh)
			{
				// Get iterator to selected vehicle:
				World::TListVehicles::const_iterator it_veh = vehs.begin();
				std::advance(it_veh, teleop_idx_veh);

				// Get speed: ground truth
				{
					const vec3& vel = it_veh->second->getVelocityLocal();
					txt2gui_tmp += mrpt::format(
						"gt. vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg/s\n",
						vel.vals[0], vel.vals[1], mrpt::RAD2DEG(vel.vals[2]));
				}
				// Get speed: ground truth
				{
					const vec3& vel =
						it_veh->second->getVelocityLocalOdoEstimate();
					txt2gui_tmp += mrpt::format(
						"odo vel: lx=%7.03f, ly=%7.03f, w= %7.03fdeg/s\n",
						vel.vals[0], vel.vals[1], mrpt::RAD2DEG(vel.vals[2]));
				}

				// Generic teleoperation interface for any controller that
				// supports it:
				{
					ControllerBaseInterface* controller =
						it_veh->second->getControllerInterface();
					ControllerBaseInterface::TeleopInput teleop_in;
					ControllerBaseInterface::TeleopOutput teleop_out;
					teleop_in.keycode = keyevent.keycode;
					controller->teleop_interface(teleop_in, teleop_out);
					txt2gui_tmp += teleop_out.append_gui_lines;
				}
			}
		}

		// Clear the keystroke buffer
		if (keyevent.keycode != 0) gui_key_events = World::TGUIKeyEvent();

		msg2gui = txt2gui_tmp;	// send txt msgs to show in the GUI

	}  // end while()

	thread_params.closing = true;
	thGUI.join();  // TODO: It could break smth

	return 0;
}

void mvsim_server_thread_update_GUI(TThreadParams& thread_params)
{
	while (!thread_params.closing)
	{
		World::TUpdateGUIParams guiparams;
		guiparams.msg_lines = msg2gui;

		thread_params.world->update_GUI(&guiparams);

		// Send key-strokes to the main thread:
		if (guiparams.keyevent.keycode != 0)
			gui_key_events = guiparams.keyevent;

		std::this_thread::sleep_for(std::chrono::milliseconds(25));
	}
}
