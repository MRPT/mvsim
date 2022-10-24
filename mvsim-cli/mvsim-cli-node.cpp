/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/Comms/Client.h>

#include "mvsim-cli.h"

static int printCommandsNode(bool showErrorMsg);
static int nodeList();

const std::map<std::string, cmd_t> cliNodeCommands = {
	{"list", cmd_t(&nodeList)},
};

int commandNode()
{
	const auto& lstCmds = cli->argCmd.getValue();
	if (cli->argHelp.isSet()) return printCommandsNode(false);
	if (lstCmds.size() != 2) return printCommandsNode(true);

	// Take second unlabeled argument:
	const std::string subcommand = lstCmds.at(1);
	auto itSubcmd = cliNodeCommands.find(subcommand);

	if (itSubcmd == cliNodeCommands.end()) return printCommandsNode(true);

	// Execute command:
	return (itSubcmd->second)();
}

int nodeList()
{
	mvsim::Client client;

	client.setMinLoggingLevel(
		mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
			cli->argVerbosity.getValue()));

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

int printCommandsNode(bool showErrorMsg)
{
	if (showErrorMsg)
	{
		setConsoleErrorColor();
		std::cerr << "Error: missing or unknown subcommand.\n";
		setConsoleNormalColor();
	}

	fprintf(
		stderr,
		R"XXX(Usage:

    mvsim node --help     Show this help
    mvsim node list       List all nodes connected to the server.

)XXX");

	return showErrorMsg ? 1 : 0;
}
