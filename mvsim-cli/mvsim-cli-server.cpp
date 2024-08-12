/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mvsim/Comms/Server.h>
#include <mvsim/Comms/ports.h>	// MVSIM_PORTNO_MAIN_REP

#include "mvsim-cli.h"

std::shared_ptr<mvsim::Server> server;

void commonLaunchServer()
{
	ASSERT_(!server);

	// Start network server:
	server = std::make_shared<mvsim::Server>();

	if (cli->argPort.isSet()) server->listenningPort(cli->argPort.getValue());

	server->setMinLoggingLevel(mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(
		cli->argVerbosity.getValue()));

	server->start();
}

int launchStandAloneServer()
{
	if (cli->argHelp.isSet())
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
