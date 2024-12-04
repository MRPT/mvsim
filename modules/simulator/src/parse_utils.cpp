/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "parse_utils.h"

#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/version.h>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <sstream>

thread_local const bool MVSIM_VERBOSE_PARSE = mrpt::get_env<bool>("MVSIM_VERBOSE_PARSE", false);

using namespace mvsim;

namespace
{
std::string parseEnvVars(const std::string& text)
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
			"parseEnvVars(): Undefined environment variable found: $env{%s}", varname.c_str());
	}

	return parseEnvVars(pre + varvalue + post.substr(post_end + 1));
	MRPT_TRY_END
}

std::string parseVars(
	const std::string& text, const std::map<std::string, std::string>& variableNamesValues)
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
	if (const auto it = variableNamesValues.find(varname); it != variableNamesValues.end())
	{
		varvalue = it->second;
	}
	else
	{
		std::string allKnown;
		for (const auto& kv : variableNamesValues)
		{
			allKnown += kv.first;
			allKnown += ",";
		}

		THROW_EXCEPTION_FMT(
			"parseVars(): Undefined variable found: ${%s}. Known ones are: %s", varname.c_str(),
			allKnown.c_str());
	}

	return parseVars(pre + varvalue + post.substr(post_end + 1), variableNamesValues);
	MRPT_TRY_END
}

std::string parseCmdRuns(const std::string& text)
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
		THROW_EXCEPTION_FMT("Error (retval=%i) executing external command: `%s`", ret, cmd.c_str());
	}
	// Clear whitespaces:
	cmdOut = mrpt::system::trim(cmdOut);
	cmdOut.erase(std::remove(cmdOut.begin(), cmdOut.end(), '\r'), cmdOut.end());
	cmdOut.erase(std::remove(cmdOut.begin(), cmdOut.end(), '\n'), cmdOut.end());

	return parseCmdRuns(pre + cmdOut + post.substr(post_end + 1));
	MRPT_TRY_END
}

double my_rand()
{
	auto& rng = mrpt::random::getRandomGenerator();
	return rng.drawUniform(0.0, 1.0);
}
double my_unifrnd(double xMin, double xMax)
{
	auto& rng = mrpt::random::getRandomGenerator();
	return rng.drawUniform(xMin, xMax);
}
double randn()
{
	auto& rng = mrpt::random::getRandomGenerator();
	return rng.drawGaussian1D_normalized();
}
double randomize(double seed)
{
	auto& rng = mrpt::random::getRandomGenerator();
	rng.randomize(seed);
	return 0;
}

// Examples: "$f{180/5}",   "$f{ ${MAX_SPEED} * sin(deg2rad(45)) }"
std::string parseMathExpr(
	const std::string& text, const std::map<std::string, std::string>& variableNamesValues)
{
	MRPT_TRY_START

	const auto start = text.find("$f{");
	if (start == std::string::npos) return text;

	const std::string pre = text.substr(0, start);
	const std::string post = text.substr(start + 3);

	const auto post_end = post.find('}');
	if (post_end == std::string::npos)
	{
		THROW_EXCEPTION_FMT(
			"Column=%u: Cannot find matching `}` for `${` in: `%s`",
			static_cast<unsigned int>(start), text.c_str());
	}

	const auto sExpr = post.substr(0, post_end);

	mrpt::expr::CRuntimeCompiledExpression expr;

	expr.register_function("rand", &my_rand);
	expr.register_function("unifrnd", &my_unifrnd);
	expr.register_function("randn", &randn);
	expr.register_function("randomize", &randomize);

	std::map<std::string, double> numericVars;
	for (const auto& kv : variableNamesValues)
	{
		std::stringstream ss(kv.second);

		double val = 0;
		if (!(ss >> val)) continue;

		numericVars[kv.first] = val;
	}

	// Compile expression (will throw on syntax error):
	expr.compile(sExpr, numericVars);
	const double val = expr.eval();

	return parseCmdRuns(pre + mrpt::format("%g", val) + post.substr(post_end + 1));

	MRPT_TRY_END
}
}  // namespace

std::string mvsim::parse(
	const std::string& input, const std::map<std::string, std::string>& variableNamesValues)
{
	if (MVSIM_VERBOSE_PARSE)
	{
		std::cout << "[mvsim::parse] Input : '" << input << "' "
				  << "with these variables: ";
		for (const auto& kv : variableNamesValues) std::cout << kv.first << ", ";
		std::cout << "\n";
	}

	std::string s = input;

	std::string prevValue = s;
	for (int iter = 0; iter < 10; iter++)
	{
		s = parseVars(s, variableNamesValues);
		s = parseEnvVars(s);
		s = parseCmdRuns(s);
		s = parseMathExpr(s, variableNamesValues);
		// We may need to iterate since, in general, each expression generator
		// might generate another kind of expression:
		if (s == prevValue) break;
		prevValue = s;
	}

	if (MVSIM_VERBOSE_PARSE)
	{
		std::cout << "[mvsim::parse] Output: '" << s << "'\n";
	}

	return s;
}

std::string mvsim::trim(const std::string& s)
{
	auto out = mrpt::system::trim(s);
	while (!out.empty() && (out[0] == '\r' || out[0] == '\n')) out.erase(0, 1);
	return out;
}
