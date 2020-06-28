/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/Comms/Client.h>

#include "mvsim-cli.h"

static int printCommandsTopic(bool showErrorMsg);
static int topicList();

static const std::map<std::string, cmd_t> cliTopicCommands = {
	{"list", cmd_t(&topicList)},
};

int commandNode()
{
	const auto& lstCmds = argCmd.getValue();
	if (argHelp.isSet()) return printCommandsTopic(false);
	if (lstCmds.size() != 2) return printCommandsTopic(true);

	// Take second unlabeled argument:
	const std::string subcommand = lstCmds.at(1);
	auto itSubcmd = cliTopicCommands.find(subcommand);

	if (itSubcmd == cliTopicCommands.end()) return printCommandsTopic(true);

	// Execute command:
	return (itSubcmd->second)();
}

int topicList()
{
	mvsim::Client client;

	client.setMinLoggingLevel(
		mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
			argVerbosity.getValue()));

	std::cout << "# Connecting to server...\n";
	client.connect();
	std::cout << "# Connected.\n";
	std::cout << "# Querying list of nodes to server...\n";
	const auto lstNodes = client.requestListOfNodes();

	std::cout << "# Done. Found " << lstNodes.size() << " nodes:\n";

	for (const auto& n : lstNodes)
	{
		std::cout << "- name: \"" << n.name << "\"\n";
	}

	return 0;
}

int printCommandsTopic(bool showErrorMsg)
{
	if (showErrorMsg)
	{
		mrpt::system::setConsoleColor(mrpt::system::CONCOL_RED);
		std::cerr << "Error: missing or unknown subcommand.\n";
		mrpt::system::setConsoleColor(mrpt::system::CONCOL_NORMAL);
	}

	fprintf(
		stderr,
		R"XXX(Usage:

    mvsim node --help     Show this help
    mvsim node list       List all nodes connected to the server.

)XXX");

	return showErrorMsg ? 1 : 0;
}
