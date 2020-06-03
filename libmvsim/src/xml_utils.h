/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

#include <mvsim/basic_types.h>
#include <map>
#include <rapidxml.hpp>
#include <string>

// Fwd decl:
namespace mrpt
{
namespace math
{
class TPolygon2D;
}
}  // namespace mrpt

namespace mvsim
{
/** Normal case: sscanf()-like specifiers, and "void*" pointing to corresponding
 * variable type.
 * Special cases:
 *  - "%lf_deg" => "val" is a "double*". The read number will be converted
 * from degrees to radians.
 *  - "%s" => "val" is assumed to be a pointer to a std::string. Strings are
 * trimmed of whitespaces.
 *  - "%color" => Expected values: "#RRGGBB[AA]" ([00-FF] each). "val" is
 * assumed to be a pointer to a mrpt::img::TColor
 *  - "%pose2d" => Expects "X Y YAW_DEG". "Val" is a pointer to
 * mrpt::poses::CPose2D
 *  - "%pose2d_ptr3d" => Expects "X Y YAW_DEG". "Val" is a pointer to
 * mrpt::poses::CPose3D
 *  - "%bool" ==> bool*. Values: 'true'/'false' or '1'/'0'
 */
struct TParamEntry
{
	const char* frmt;
	void* val;

	TParamEntry() : frmt(nullptr), val(nullptr) {}
	TParamEntry(const char* frmt_, void* val_) : frmt(frmt_), val(val_) {}
	/** Tries to parse the given input string according to the expected format,
	 * then store the result in "*val"
	 * \exception std::runtime_error On format errors.
	 */
	void parse(
		const std::string& str, const std::string& varName,
		const char* function_name_context = "") const;
};

void parse_xmlnode_attribs(
	const rapidxml::xml_node<char>& xml_node,
	const std::map<std::string, TParamEntry>& params,
	const char* function_name_context = "");

/** Check whether the given XML node name matches any of the param list
 * \return false if no match found
 * \sa parse_xmlnode_children_as_param
 */
bool parse_xmlnode_as_param(
	const rapidxml::xml_node<char>& xml_node,
	const std::map<std::string, TParamEntry>& params,
	const char* function_name_context = "");

/** Call \a parse_xmlnode_as_param() for all children nodes of the given node.
 */
void parse_xmlnode_children_as_param(
	const rapidxml::xml_node<char>& xml_node,
	const std::map<std::string, TParamEntry>& params,
	const char* function_name_context = "");

template <class NODE_LIST>
void parse_xmlnodelist_children_as_param(
	NODE_LIST& lst_nodes, const std::map<std::string, TParamEntry>& params,
	const char* function_name_context = "")
{
	for (typename NODE_LIST::iterator it = lst_nodes.begin();
		 it != lst_nodes.end(); ++it)
		parse_xmlnode_children_as_param(**it, params, function_name_context);
}

// Bits:

/** Parses a string like "XXX YYY PHI" with X,Y in meters, PHI in degrees, and
 * returns
 * a vec3 with [x,y,phi] with angle in radians. Raises an exception upon
 * malformed string.
 */
vec3 parseXYPHI(
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
