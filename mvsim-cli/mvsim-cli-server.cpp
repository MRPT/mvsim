/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <mvsim/Comms/Server.h>
#include <mvsim/Comms/ports.h>	// MVSIM_PORTNO_MAIN_REP
#endif

#include "mvsim-cli.h"

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
std::shared_ptr<mvsim::Server> server;
#endif

unsigned int commonLaunchServer()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	ASSERT_(!server);

	const bool portFromCli = cli->cmd["--port"]->count() > 0;
	const bool portIsForced = portFromCli || mvsim::serverPortFromEnvironment().has_value();
	const unsigned int firstPort = portFromCli ? cli->argPort : mvsim::defaultServerPort();
	const unsigned int numCandidates =
		portIsForced ? 1 : mvsim::MVSIM_PORTNO_MAIN_REP_NUM_CANDIDATES;

	// Start network server, looking for a free port if allowed:
	for (unsigned int i = 0; i < numCandidates; i++)
	{
		auto s = std::make_shared<mvsim::Server>();
		s->listenningPort(firstPort + i);
		s->setMinLoggingLevel(
			mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(cli->argVerbosity));

		try
		{
			s->start();
		}
		catch (const std::exception&)
		{
			if (i + 1 == numCandidates)
			{
				throw;
			}
			continue;
		}

		server = s;
		if (i > 0)
		{
			std::cerr << "WARNING: TCP port " << firstPort
					  << " is in use by another process, listening at port " << firstPort + i
					  << " instead.\nSet the environment variable MVSIM_SERVER_PORT="
					  << firstPort + i
					  << " (or use --port) for other MVSim tools and clients to find this "
						 "server.\n";
		}
		return firstPort + i;
	}
#endif
	return 0;
}

int launchStandAloneServer()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	if (cli->argHelp)
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
#endif

	try
	{
		commonLaunchServer();
	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		return 1;
	}
	return 0;
}
