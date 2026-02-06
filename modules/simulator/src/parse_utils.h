/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <map>
#include <string>

namespace mvsim
{
/**
 * Parse a string (typically as read from an XML file) and replaces the
 * following expressions:
 * - `${VAR}`: Variable names.
 * - `$env{VAR}`: Environment variables.
 * - `$(cmd)`: The output from an external program.
 *
 */
std::string parse(
	const std::string& input, const std::map<std::string, std::string>& variableNamesValues = {});

/** removes trailing and leading whitespaces and leading new lines */
std::string trim(const std::string& s);

}  // namespace mvsim
