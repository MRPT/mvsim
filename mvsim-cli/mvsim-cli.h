/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/3rdparty/tclap/CmdLine.h>
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <mvsim/Comms/ports.h>
#endif

#include <functional>
#include <memory>

// We need all TCLAP objects to be initialized in order for all translation
// units, that is why we use this holder structure:
struct cli_flags
{
	TCLAP::CmdLine cmd{"mvsim", ' ', "version", false /* no --help */};

	TCLAP::UnlabeledMultiArg<std::string> argCmd{
		"command", "Command to run. Run 'mvsim help' to list commands.", false, "", cmd};

	TCLAP::ValueArg<std::string> argVerbosity{
		"v", "verbose", "Verbosity level", false, "INFO", "ERROR|WARN|INFO|DEBUG", cmd};

	TCLAP::SwitchArg argFullProfiler{
		"", "full-profiler",
		"Enable saving *all* timing data, dumping it to a file at the end of "
		"the "
		"program.",
		cmd};

	TCLAP::SwitchArg argHeadless{"", "headless", "Runs the simulator without any GUI window.", cmd};

	TCLAP::SwitchArg argDetails{"", "details", "Shows details in the specified subcommand", cmd};

	TCLAP::SwitchArg argVersion{"", "version", "Shows program version and exits", cmd};

	TCLAP::SwitchArg argHelp{"h", "help", "Shows more detailed help for command", cmd};

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	TCLAP::ValueArg<int> argPort{
		"p", "port", "TCP port to listen at", false, mvsim::MVSIM_PORTNO_MAIN_REP, "TCP port", cmd};
#endif

	TCLAP::ValueArg<double> argRealTimeFactor{
		"",
		"realtime-factor",
		"Realtime modification factor: <1 slower than real-time, >1 faster "
		"than real-time",
		false,
		1.0,
		"1.0",
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
