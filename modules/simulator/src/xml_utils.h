/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

#include <mvsim/TParameterDefinitions.h>
#include <mvsim/basic_types.h>

#include <map>
#include <memory>
#include <rapidxml.hpp>
#include <set>
#include <string>
#include <tuple>

// Fwd decl:
namespace mrpt::math
{
class TPolygon2D;
}  // namespace mrpt::math

namespace mrpt::system
{
class COutputLogger;
}

namespace mvsim
{
struct XML_Doc_Data
{
	using Ptr = std::shared_ptr<XML_Doc_Data>;

	std::string documentData;
	std::shared_ptr<rapidxml::xml_document<>> doc;
};

std::tuple<std::shared_ptr<rapidxml::xml_document<>>, rapidxml::xml_node<>*> readXmlTextAndGetRoot(
	const std::string& xmlData, const std::string& pathToFile);

std::tuple<XML_Doc_Data::Ptr, rapidxml::xml_node<>*> readXmlAndGetRoot(
	const std::string& pathToFile, const std::map<std::string, std::string>& variables,
	const std::set<std::string>& varsRetain = {});

/**
 * Replaces: Variables are first searched in the "<include />" attributes,
 * or as environment variables if not found.
 *
 * - `${VAR}`: var contents. Throw on undefined var.
 * - `${VAR|DEFAULT}`: var contents, or DEFAULT if undefined.
 */
std::string parse_variables(
	const std::string& in, const std::map<std::string, std::string>& variables,
	const std::set<std::string>& varsRetain);

void parse_xmlnode_attribs(
	const rapidxml::xml_node<char>& xml_node, const TParameterDefinitions& params,
	const std::map<std::string, std::string>& variableNamesValues = {},
	const char* functionNameContext = "");

/** Check whether the given XML node name matches any of the param list
 * \return false if no match found
 * \sa parse_xmlnode_children_as_param
 */
bool parse_xmlnode_as_param(
	const rapidxml::xml_node<char>& xml_node, const TParameterDefinitions& params,
	const std::map<std::string, std::string>& variableNamesValues = {},
	const char* functionNameContext = "");

/** Call \a parse_xmlnode_as_param() for all children nodes of the given node.
 */
void parse_xmlnode_children_as_param(
	const rapidxml::xml_node<char>& xml_node, const TParameterDefinitions& params,
	const std::map<std::string, std::string>& variableNamesValues = {},
	const char* functionNameContext = "", mrpt::system::COutputLogger* logger = nullptr);

template <class NODE_LIST>
void parse_xmlnodelist_children_as_param(
	NODE_LIST& lst_nodes, const TParameterDefinitions& params, const char* functionNameContext = "")
{
	for (auto& node : lst_nodes)
		parse_xmlnode_children_as_param(*node, params, functionNameContext);
}

/** Convert an XML node into a string, solving in the way,
 all found "include"s. For this later task, we need a reference
to the World object (to solve for variables, current local dir, etc.)
*/
std::string xml_to_str_solving_includes(
	const World& parent, const rapidxml::xml_node<char>* xml_node,
	const std::set<std::string>& varsRetain = {});

// Bits:

/** Parses a string like "XXX YYY PHI" with X,Y in meters, PHI in degrees, and
 * returns a mrpt::math::TPose2D with [x,y,phi] with angle in radians. Raises an
 * exception upon malformed string.
 */
mrpt::math::TPose2D parseXYPHI(
	const std::string& s, bool allow_missing_angle = false, double default_angle_radians = 0.0,
	const std::map<std::string, std::string>& variableNamesValues = {});

/** Parses a <shape><pt>X Y</pt>...</shape> XML node into a
 * mrpt::math::TPolygon2D
 * \exception std::exception On syntax errors, etc.
 */
void parse_xmlnode_shape(
	const rapidxml::xml_node<char>& xml_node, mrpt::math::TPolygon2D& out_poly,
	const char* functionNameContext = "");
}  // namespace mvsim
