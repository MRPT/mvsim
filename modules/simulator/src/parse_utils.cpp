/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "parse_utils.h"

#include <mrpt/core/exceptions.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>

using namespace mvsim;

static std::string parseEnvVars(const std::string& text)
{
	MRPT_TRY_START

	const auto start = text.find("$env{");
	if (start == std::string::npos) return text;

	const std::string pre = text.substr(0, start);
	const std::string post = text.substr(start + 5);

	const auto post_end = post.find('}');
	if (post_end == std::string::npos)
	{
		THROW_EXCEPTION_FMT(
			"Column=%u: Cannot find matching `}` for `$env{` in: `%s`",
			static_cast<unsigned int>(start), text.c_str());
	}

	const auto varname = post.substr(0, post_end);
	std::string varvalue;
	const char* v = ::getenv(varname.c_str());
	if (v != nullptr)
		varvalue = std::string(v);
	else
	{
		THROW_EXCEPTION_FMT(
			"parseEnvVars(): Undefined environment variable found: $env{%s}",
			varname.c_str());
	}

	return parseEnvVars(pre + varvalue + post.substr(post_end + 1));
	MRPT_TRY_END
}

static std::string parseVars(
	const std::string& text,
	const std::map<std::string, std::string>& variableNamesValues)
{
	MRPT_TRY_START

	const auto start = text.find("${");
	if (start == std::string::npos) return text;

	const std::string pre = text.substr(0, start);
	const std::string post = text.substr(start + 2);

	const auto post_end = post.find('}');
	if (post_end == std::string::npos)
	{
		THROW_EXCEPTION_FMT(
			"Column=%u: Cannot find matching `}` for `${` in: `%s`",
			static_cast<unsigned int>(start), text.c_str());
	}

	const auto varname = post.substr(0, post_end);
	std::string varvalue;
	if (const auto it = variableNamesValues.find(varname);
		it != variableNamesValues.end())
	{
		varvalue = it->second;
	}
	else
	{
		THROW_EXCEPTION_FMT(
			"parseEnvVars(): Undefined variable found: $env{%s}",
			varname.c_str());
	}

	return parseEnvVars(pre + varvalue + post.substr(post_end + 1));
	MRPT_TRY_END
}

static std::string parseCmdRuns(const std::string& text)
{
	MRPT_TRY_START

	const auto start = text.find("$(");
	if (start == std::string::npos) return text;

	const std::string pre = text.substr(0, start);
	const std::string post = text.substr(start + 2);

	const auto post_end = post.find(')');
	if (post_end == std::string::npos)
	{
		THROW_EXCEPTION_FMT(
			"Column=%u: Cannot find matching `)` for `$(` in: `%s`",
			static_cast<unsigned int>(start), text.c_str());
	}

	const auto cmd = post.substr(0, post_end);

	// Launch command and get console output:
	std::string cmdOut;

	int ret = mrpt::system::executeCommand(cmd, &cmdOut);
	if (ret != 0)
	{
		THROW_EXCEPTION_FMT(
			"Error (retval=%i) executing external command: `%s`", ret,
			cmd.c_str());
	}
	// Clear whitespaces:
	cmdOut = mrpt::system::trim(cmdOut);
	cmdOut.erase(std::remove(cmdOut.begin(), cmdOut.end(), '\r'), cmdOut.end());
	cmdOut.erase(std::remove(cmdOut.begin(), cmdOut.end(), '\n'), cmdOut.end());

	return parseCmdRuns(pre + cmdOut + post.substr(post_end + 1));
	MRPT_TRY_END
}

MRPT_TODO("Add '$f{xxx}' exprtk parser, define random() user function")

std::string mvsim::parse(
	const std::string& input,
	const std::map<std::string, std::string>& variableNamesValues)
{
	std::string s = input;

	std::string prevValue = s;
	for (int iter = 0; iter < 10; iter++)
	{
		s = parseVars(s, variableNamesValues);
		s = parseEnvVars(s);
		s = parseCmdRuns(s);
		// We may need to iterate since, in general, each expression generator
		// might generate another kind of expression:
		if (s == prevValue) break;
		prevValue = s;
	}

	return s;
}
