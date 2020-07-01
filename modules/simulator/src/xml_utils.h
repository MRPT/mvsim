/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

#include <mvsim/TParameterDefinitions.h>
#include <mvsim/basic_types.h>

#include <map>
#include <rapidxml.hpp>
#include <string>

// Fwd decl:
namespace mrpt::math
{
class TPolygon2D;
}  // namespace mrpt::math

namespace mvsim
{
void parse_xmlnode_attribs(
	const rapidxml::xml_node<char>& xml_node,
	const TParameterDefinitions& params,
	const char* function_name_context = "");

/** Check whether the given XML node name matches any of the param list
 * \return false if no match found
 * \sa parse_xmlnode_children_as_param
 */
bool parse_xmlnode_as_param(
	const rapidxml::xml_node<char>& xml_node,
	const TParameterDefinitions& params,
	const char* function_name_context = "");

/** Call \a parse_xmlnode_as_param() for all children nodes of the given node.
 */
void parse_xmlnode_children_as_param(
	const rapidxml::xml_node<char>& xml_node,
	const TParameterDefinitions& params,
	const char* function_name_context = "");

template <class NODE_LIST>
void parse_xmlnodelist_children_as_param(
	NODE_LIST& lst_nodes, const TParameterDefinitions& params,
	const char* function_name_context = "")
{
	for (auto& node : lst_nodes)
		parse_xmlnode_children_as_param(*node, params, function_name_context);
}

// Bits:

/** Parses a string like "XXX YYY PHI" with X,Y in meters, PHI in degrees, and
 * returns
 * a mrpt::math::TTwist2D with [x,y,phi] with angle in radians. Raises an
 * exception upon malformed string.
 */
mrpt::math::TPose2D parseXYPHI(
	const std::string& s, bool allow_missing_angle = false,
	double default_angle_radians = 0.0);

/** Parses a <shape><pt>X Y</pt>...</shape> XML node into a
 * mrpt::math::TPolygon2D
 * \exception std::exception On syntax errors, etc.
 */
void parse_xmlnode_shape(
	const rapidxml::xml_node<char>& xml_node, mrpt::math::TPolygon2D& out_poly,
	const char* function_name_context = "");
}  // namespace mvsim
