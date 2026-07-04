/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <CLI/CLI.hpp>
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <mvsim/Comms/ports.h>
#endif

#include <functional>
#include <memory>
#include <string>
#include <vector>

struct cli_flags
{
	CLI::App cmd{"mvsim", "mvsim"};

	std::vector<std::string> argCmd;
	std::string argVerbosity = "INFO";
	bool argFullProfiler = false;
	bool argHeadless = false;
	bool argDetails = false;
	bool argVersion = false;
	bool argHelp = false;
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	int argPort = mvsim::MVSIM_PORTNO_MAIN_REP;
#endif
	double argRealTimeFactor = 1.0;

	cli_flags()
	{
		cmd.set_help_flag();  // disable built-in --help/-h
		cmd.allow_extras(true);

		cmd.add_option("commands", argCmd, "Command to run. Run 'mvsim help' to list commands.");

		cmd.add_option("-v,--verbose", argVerbosity, "Verbosity level");

		cmd.add_flag("--full-profiler", argFullProfiler,
			"Enable saving *all* timing data, dumping it to a file at the end of the program.");

		cmd.add_flag("--headless", argHeadless, "Runs the simulator without any GUI window.");

		cmd.add_flag("--details", argDetails, "Shows details in the specified subcommand");

		cmd.add_flag("--version", argVersion, "Shows program version and exits");

		cmd.add_flag("-h,--help", argHelp, "Shows more detailed help for command");

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
		cmd.add_option("-p,--port", argPort, "TCP port to listen at");
#endif

		cmd.add_option("--realtime-factor", argRealTimeFactor,
			"Realtime modification factor: <1 slower than real-time, >1 faster than real-time");
	}
};

extern std::unique_ptr<cli_flags> cli;

using cmd_t = std::function<int(void)>;

int printListCommands();  // "help"
void printVersion();  // "--version"
int launchStandAloneServer();  // "server"
int launchSimulation();	 // "launch"
int commandNode();	// "node"
int commandTopic();	 // "topic"

void setConsoleErrorColor();
void setConsoleNormalColor();

void commonLaunchServer();
