/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>	 // kbhit()

#include <chrono>
#include <functional>
#include <iostream>
#include <map>

#include "mvsim-cli.h"

TCLAP::CmdLine cmd("mvsim", ' ', "version", false /* no --help */);

TCLAP::UnlabeledMultiArg<std::string> argCmd(
	"command", "Command to run. Run 'mvsim help' to list commands.", false, "",
	cmd);

TCLAP::ValueArg<std::string> argVerbosity(
	"v", "verbose", "Verbosity level", false, "INFO", "INFO", cmd);

TCLAP::SwitchArg argHelp(
	"h", "help", "Shows more detailed help for command", cmd);

// ======= Command handlers =======
using cmd_t = std::function<int(void)>;

const std::map<std::string, cmd_t> cliCommands = {
	{"help", cmd_t(&printListCommands)},
	{"server", cmd_t(&launchStandAloneServer)},
	{"launch", cmd_t(&launchSimulation)},
	{"node", cmd_t(&commandNode)},
	{"topic", cmd_t(&commandTopic)},
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
    mvsim node                List connected nodes, etc.
    mvsim topic               Inspect, publish, etc. topics.

Or use `mvsim <COMMAND> --help` for further options
)XXX");
	return 0;
}
