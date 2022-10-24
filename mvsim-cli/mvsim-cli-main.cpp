/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/3rdparty/tclap/CmdLine.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/os.h>	 // kbhit()
#include <mrpt/version.h>
#include <mvsim/mvsim_version.h>

#include <chrono>
#include <iostream>
#include <map>

#include "mvsim-cli.h"

// ======= Command handlers =======
std::unique_ptr<cli_flags> cli;

const std::map<std::string, cmd_t> cliCommands = {
	{"help", cmd_t(&printListCommands)},
	{"server", cmd_t(&launchStandAloneServer)},
	{"launch", cmd_t(&launchSimulation)},
	{"node", cmd_t(&commandNode)},
	{"topic", cmd_t(&commandTopic)},
};

void setConsoleErrorColor()
{
#if MRPT_VERSION >= 0x233
	mrpt::system::consoleColorAndStyle(
		mrpt::system::ConsoleForegroundColor::RED);
#else
	mrpt::system::setConsoleColor(mrpt::system::CONCOL_RED);
#endif
}

void setConsoleNormalColor()
{
#if MRPT_VERSION >= 0x233
	mrpt::system::consoleColorAndStyle(
		mrpt::system::ConsoleForegroundColor::DEFAULT);
#else
	mrpt::system::setConsoleColor(mrpt::system::CONCOL_NORMAL);
#endif
}

int main(int argc, char** argv)
{
	try
	{
		cli = std::make_unique<cli_flags>();

		if (!cli->cmd.parse(argc, argv))
		{
			printListCommands();
			return 1;
		}

		if (cli->argVersion.isSet())
		{
			printVersion();
			return 0;
		}

		// Take first unlabeled argument:
		std::string command;
		if (const auto& lst = cli->argCmd.getValue(); !lst.empty())
			command = lst.at(0);

		// Look up command in table:
		auto itCmd = cliCommands.find(command);

		if (!cli->argCmd.isSet() || itCmd == cliCommands.end())
		{
			if (!cli->argHelp.isSet())
			{
				setConsoleErrorColor();
				std::cerr << "Error: missing or unknown command.\n";
				setConsoleNormalColor();
			}
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
		R"XXX(mvsim v%s: A lightweight multivehicle simulation environment.

Available commands:
    mvsim launch <WORLD.xml>  Start a comm. server and simulates a world.
    mvsim server              Start a standalone communication server.
    mvsim node                List connected nodes, etc.
    mvsim topic               Inspect, publish, etc. topics.
    mvsim --version           Shows program version.
    mvsim --help              Shows this information.

Or use `mvsim <COMMAND> --help` for further options
)XXX",
		MVSIM_VERSION);
	return 0;
}

void printVersion() { std::cout << "mvsim v" << MVSIM_VERSION << std::endl; }
