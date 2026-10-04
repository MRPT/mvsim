/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <cstdlib>
#include <optional>

namespace mvsim
{
/** \ingroup mvsim_comms_module
 * @{ */

constexpr unsigned int MVSIM_PORTNO_MAIN_REP = 23700;

/** Number of consecutive ports, starting at MVSIM_PORTNO_MAIN_REP, that
 * "mvsim launch" tries when the default port is already in use. */
constexpr unsigned int MVSIM_PORTNO_MAIN_REP_NUM_CANDIDATES = 10;

/** Returns the TCP port set by the environment variable MVSIM_SERVER_PORT,
 * if it holds a valid port number. It is honored by both servers and clients,
 * so all processes can be pointed to a non-default port. */
inline std::optional<unsigned int> serverPortFromEnvironment()
{
	const char* env = std::getenv("MVSIM_SERVER_PORT");
	if (!env || !*env)
	{
		return std::nullopt;
	}
	char* end = nullptr;
	const unsigned long v = std::strtoul(env, &end, 10);
	if (*end != '\0' || v < 1 || v > 65535)
	{
		return std::nullopt;
	}
	return static_cast<unsigned int>(v);
}

/** The port used by default: MVSIM_SERVER_PORT, or MVSIM_PORTNO_MAIN_REP. */
inline unsigned int defaultServerPort()
{
	return serverPortFromEnvironment().value_or(MVSIM_PORTNO_MAIN_REP);
}

}  // namespace mvsim
