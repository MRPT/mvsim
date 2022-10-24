/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mvsim/Comms/ports.h>

#include <functional>
#include <memory>

// We need all TCLAP objects to be initialized in order for all translation
// units, that is why we use this holder structure:
struct cli_flags
{
	TCLAP::CmdLine cmd{"mvsim", ' ', "version", false /* no --help */};

	TCLAP::UnlabeledMultiArg<std::string> argCmd{
		"command", "Command to run. Run 'mvsim help' to list commands.", false,
		"", cmd};

	TCLAP::ValueArg<std::string> argVerbosity{
		"v",   "verbose", "Verbosity level",
		false, "INFO",	  "ERROR|WARN|INFO|DEBUG",
		cmd};

	TCLAP::SwitchArg argFullProfiler{
		"", "full-profiler",
		"Enable saving *all* timing data, dumping it to a file at the end of "
		"the "
		"program.",
		cmd};

	TCLAP::SwitchArg argDetails{
		"", "details", "Shows details in the specified subcommand", cmd};

	TCLAP::SwitchArg argVersion{
		"", "version", "Shows program version and exits", cmd};

	TCLAP::SwitchArg argHelp{
		"h", "help", "Shows more detailed help for command", cmd};

	TCLAP::ValueArg<int> argPort{
		"p",
		"port",
		"TCP port to listen at",
		false,
		mvsim::MVSIM_PORTNO_MAIN_REP,
		"TCP port",
		cmd};
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
