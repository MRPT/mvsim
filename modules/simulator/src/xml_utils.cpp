/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "xml_utils.h"

#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/img/TColor.h>
#include <mrpt/io/vector_loadsave.h>  // file_get_contents()
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mvsim/basic_types.h>

#include <cstdio>

#include "parse_utils.h"

using namespace rapidxml;
using namespace mvsim;

/** Tries to parse the given input string according to the expected format, then
 * store the result in "*val"
 * \exception std::runtime_error On format errors.
 */
void TParamEntry::parse(
	const std::string& inStr, const std::string& varName,
	const std::map<std::string, std::string>& variableNamesValues,
	const char* functionNameContext) const
{
	const std::string str = mvsim::parse(inStr, variableNamesValues);

	// Special cases:
	// "%s" ==> std::strings
	if (std::string(frmt) == std::string("%s"))
	{
		std::string& val2 = *reinterpret_cast<std::string*>(val);
		val2 = mrpt::system::trim(str);
	}
	// "%lf_deg" ==> mrpt::DEG2RAD()
	else if (std::string(frmt) == std::string("%lf_deg"))
	{
		if (1 != ::sscanf(str.c_str(), frmt, val))
			throw std::runtime_error(mrpt::format(
				"%s Error parsing attribute '%s'='%s' (Expected "
				"format:'%s')",
				functionNameContext, varName.c_str(), str.c_str(), frmt));
		double& ang = *reinterpret_cast<double*>(val);
		ang = mrpt::DEG2RAD(ang);
	}
	// "%bool" ==> bool*
	else if (std::string(frmt) == std::string("%bool"))
	{
		bool& bool_val = *reinterpret_cast<bool*>(val);

		const std::string sStr =
			mrpt::system::lowerCase(mrpt::system::trim(std::string(str)));
		if (sStr == "1" || sStr == "true")
			bool_val = true;
		else if (sStr == "0" || sStr == "false")
			bool_val = false;
		else
			throw std::runtime_error(mrpt::format(
				"%s Error parsing 'bool' attribute '%s'='%s' (Expected "
				"'true' or 'false')",
				functionNameContext, varName.c_str(), str.c_str()));
	}
	// "%color" ==> mrpt::img::TColor
	else if (std::string(frmt) == std::string("%color"))
	{
		// HTML-like format:
		if (!(str.size() > 1 && str[0] == '#'))
			throw std::runtime_error(mrpt::format(
				"%s Error parsing '%s'='%s' (Expected "
				"format:'#RRGGBB[AA]')",
				functionNameContext, varName.c_str(), str.c_str()));

		unsigned int r, g, b, a = 0xff;
		int ret = ::sscanf(str.c_str() + 1, "%2x%2x%2x%2x", &r, &g, &b, &a);
		if (ret != 3 && ret != 4)
			throw std::runtime_error(mrpt::format(
				"%s Error parsing '%s'='%s' (Expected "
				"format:'#RRGGBB[AA]')",
				functionNameContext, varName.c_str(), str.c_str()));
		mrpt::img::TColor& col = *reinterpret_cast<mrpt::img::TColor*>(val);
		col = mrpt::img::TColor(r, g, b, a);
	}
	// "%pose2d"
	// "%pose2d_ptr3d"
	else if (!strncmp(frmt, "%pose2d", strlen("%pose2d")))
	{
		double x, y, yaw;
		int ret = ::sscanf(str.c_str(), "%lf %lf %lf", &x, &y, &yaw);
		if (ret != 3)
			throw std::runtime_error(mrpt::format(
				"%s Error parsing '%s'='%s' (Expected format:'X Y "
				"YAW_DEG')",
				functionNameContext, varName.c_str(), str.c_str()));

		// User provides angles in deg:
		yaw = mrpt::DEG2RAD(yaw);

		const mrpt::poses::CPose2D p(x, y, yaw);

		// Sub-cases:
		if (!strcmp(frmt, "%pose2d"))
		{
			mrpt::poses::CPose2D& pp =
				*reinterpret_cast<mrpt::poses::CPose2D*>(val);
			pp = p;
		}
		else if (!strcmp(frmt, "%pose2d_ptr3d"))
		{
			mrpt::poses::CPose3D& pp =
				*reinterpret_cast<mrpt::poses::CPose3D*>(val);
			pp = mrpt::poses::CPose3D(p);
		}
		else
			throw std::runtime_error(mrpt::format(
				"%s Error: Unknown format specifier '%s'", functionNameContext,
				frmt));
	}
	// "%pose3d"
	else if (!strncmp(frmt, "%pose3d", strlen("%pose3d")))
	{
		double x, y, z, yawDeg, pitchDeg, rollDeg;
		int ret = ::sscanf(
			str.c_str(), "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &yawDeg,
			&pitchDeg, &rollDeg);
		if (ret != 6)
			throw std::runtime_error(mrpt::format(
				"%s Error parsing '%s'='%s' (Expected format:'X Y Z"
				"YAW_DEG PITCH_DEG ROLL_DEG')",
				functionNameContext, varName.c_str(), str.c_str()));

		// User provides angles in deg:
		const auto yaw = mrpt::DEG2RAD(yawDeg);
		const auto pitch = mrpt::DEG2RAD(pitchDeg);
		const auto roll = mrpt::DEG2RAD(rollDeg);

		const mrpt::poses::CPose3D p(x, y, yaw);

		mrpt::poses::CPose3D& pp =
			*reinterpret_cast<mrpt::poses::CPose3D*>(val);
		pp = mrpt::poses::CPose3D(x, y, z, yaw, pitch, roll);
	}
	else
	{
		// Generic parse:
		if (1 != ::sscanf(str.c_str(), frmt, val))
			throw std::runtime_error(mrpt::format(
				"%s Error parsing attribute '%s'='%s' (Expected "
				"format:'%s')",
				functionNameContext, varName.c_str(), str.c_str(), frmt));
	}
}

void mvsim::parse_xmlnode_attribs(
	const rapidxml::xml_node<char>& xml_node,
	const TParameterDefinitions& params,
	const std::map<std::string, std::string>& variableNamesValues,
	const char* functionNameContext)
{
	for (const auto& param : params)
	{
		if (auto attr = xml_node.first_attribute(param.first.c_str());
			attr && attr->value())
		{
			param.second.parse(
				attr->value(), attr->name(), variableNamesValues,
				functionNameContext);
		}
	}
}

bool mvsim::parse_xmlnode_as_param(
	const rapidxml::xml_node<char>& xml_node,
	const TParameterDefinitions& params,
	const std::map<std::string, std::string>& variableNamesValues,
	const char* functionNameContext)
{
	TParameterDefinitions::const_iterator it_param =
		params.find(xml_node.name());

	if (it_param != params.end())
	{
		// parse parameter:
		it_param->second.parse(
			xml_node.value(), xml_node.name(), variableNamesValues,
			functionNameContext);
		return true;
	}
	return false;
}

/** Call \a parse_xmlnode_as_param() for all children nodes of the given node.
 */
void mvsim::parse_xmlnode_children_as_param(
	const rapidxml::xml_node<char>& root, const TParameterDefinitions& params,
	const std::map<std::string, std::string>& variableNamesValues,
	const char* functionNameContext)
{
	rapidxml::xml_node<>* node = root.first_node();
	while (node)
	{
		parse_xmlnode_as_param(
			*node, params, variableNamesValues, functionNameContext);
		node = node->next_sibling(nullptr);	 // Move on to next node
	}
}

mrpt::math::TPose2D mvsim::parseXYPHI(
	const std::string& s, bool allow_missing_angle,
	double default_angle_radians)
{
	mrpt::math::TPose2D v;
	v.phi = mrpt::RAD2DEG(default_angle_radians);  // Default ang.

	int na = ::sscanf(s.c_str(), "%lf %lf %lf", &v.x, &v.y, &v.phi);

	// User provides numbers as degrees:
	v.phi = mrpt::DEG2RAD(v.phi);

	if ((na != 3 && !allow_missing_angle) ||
		(na != 2 && na != 3 && allow_missing_angle))
		throw std::runtime_error(
			mrpt::format("Malformed pose string: '%s'", s.c_str()));

	return v;
}

/** Parses a <shape><pt>X Y</pt>...</shape> XML node into a
 * mrpt::math::TPolygon2D
 * \exception std::exception On syntax errors, etc.
 */
void mvsim::parse_xmlnode_shape(
	const rapidxml::xml_node<char>& xml_node, mrpt::math::TPolygon2D& out_poly,
	const char* functionNameContext)
{
	out_poly.clear();

	for (rapidxml::xml_node<char>* pt_node = xml_node.first_node("pt"); pt_node;
		 pt_node = pt_node->next_sibling("pt"))
	{
		if (!pt_node->value())
			throw std::runtime_error(mrpt::format(
				"%s Error: <pt> node seems empty.", functionNameContext));

		mrpt::math::TPoint2D pt;
		const char* str_val = pt_node->value();
		if (2 != ::sscanf(str_val, "%lf %lf", &pt.x, &pt.y))
			throw std::runtime_error(mrpt::format(
				"%s Error parsing <pt> node: '%s' (Expected format:'<pt>X "
				"Y</pt>')",
				functionNameContext, str_val));

		out_poly.push_back(pt);
	}

	if (out_poly.size() < 3)
		throw std::runtime_error(mrpt::format(
			"%s Error: <shape> node requires 3 or more <pt>X Y</pt> "
			"entries.",
			functionNameContext));
}

std::tuple<std::shared_ptr<rapidxml::xml_document<>>, rapidxml::xml_node<>*>
	mvsim::readXmlTextAndGetRoot(
		const std::string& xmlData, const std::string& pathToFile)
{
	using namespace rapidxml;
	using namespace std::string_literals;

	// Parse:
	char* input_str = const_cast<char*>(xmlData.c_str());
	auto xml = std::make_shared<rapidxml::xml_document<>>();
	try
	{
		xml->parse<0>(input_str);
	}
	catch (rapidxml::parse_error& e)
	{
		unsigned int line =
			static_cast<long>(std::count(input_str, e.where<char>(), '\n') + 1);
		throw std::runtime_error(mrpt::format(
			"XML parse error (Line %u): %s", static_cast<unsigned>(line),
			e.what()));
	}

	// Sanity check:
	xml_node<>* root = xml->first_node();
	ASSERTMSG_(
		root, "XML parse error: No root node found (empty file '"s +
				  pathToFile + "'?)"s);
	return {xml, root};
}

std::tuple<XML_Doc_Data::Ptr, rapidxml::xml_node<>*> mvsim::readXmlAndGetRoot(
	const std::string& pathToFile,
	const std::map<std::string, std::string>& variables)
{
	using namespace rapidxml;
	using namespace std::string_literals;

	ASSERT_FILE_EXISTS_(pathToFile);

	auto xmlDoc = std::make_shared<XML_Doc_Data>();

	xmlDoc->documentData = mvsim::parse_variables(
		mrpt::io::file_get_contents(pathToFile), variables);
	auto [xml, root] = readXmlTextAndGetRoot(xmlDoc->documentData, pathToFile);

	xmlDoc->doc = std::move(xml);

	return {xmlDoc, root};
}

static std::string::size_type findClosing(
	size_t pos, const std::string& s, const char searchEndChar,
	const char otherStartChar)
{
	int openEnvs = 1;
	for (; pos < s.size(); pos++)
	{
		const char ch = s[pos];
		if (ch == otherStartChar)
			openEnvs++;
		else if (ch == searchEndChar)
		{
			openEnvs--;
			if (openEnvs == 0)
			{
				return pos;
			}
		}
	}

	// not found:
	return std::string::npos;
}

// "foo|bar" -> {"foo","bar"}
static std::tuple<std::string, std::string> splitVerticalBar(
	const std::string& s)
{
	const auto posBar = s.find("|");
	if (posBar == std::string::npos) return {s, {}};

	return {s.substr(0, posBar), s.substr(posBar + 1)};
}

static std::string parseVars(
	const std::string& text,
	const std::map<std::string, std::string>& variables)
{
	MRPT_TRY_START

	const auto start = text.find("${");
	if (start == std::string::npos) return text;

	const std::string pre = text.substr(0, start);
	const std::string post = text.substr(start + 2);

	const auto post_end = findClosing(0, post, '}', '{');
	if (post_end == std::string::npos)
	{
		THROW_EXCEPTION_FMT(
			"Column=%u: Cannot find matching `}` for `${` in: `%s`",
			static_cast<unsigned int>(start), text.c_str());
	}

	const auto varnameOrg = post.substr(0, post_end);

	const auto [varname, defaultValue] = splitVerticalBar(varnameOrg);

	std::string varvalue;
	if (auto itVal = variables.find(varname); itVal != variables.end())
	{
		varvalue = itVal->second;
	}
	else if (const char* v = ::getenv(varname.c_str()); v != nullptr)
	{
		varvalue = std::string(v);
	}
	else
	{
		if (!defaultValue.empty())
		{
			varvalue = defaultValue;
		}
		else
		{
			THROW_EXCEPTION_FMT(
				"MVSIM parseVars(): Undefined variable: ${%s}",
				varname.c_str());
		}
	}

	return parseVars(pre + varvalue + post.substr(post_end + 1), variables);
	MRPT_TRY_END
}

std::string mvsim::parse_variables(
	const std::string& in, const std::map<std::string, std::string>& variables)
{
	return parseVars(in, variables);
}
