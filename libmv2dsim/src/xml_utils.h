/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#pragma once

#include <rapidxml.hpp>

#include <map>
#include <string>

namespace mv2dsim
{
	/** Normal case: sscanf()-like specifiers, and "void*" pointing to corresponding variable type.
		* Special cases: 
		*  - "%s" => "val" is assumed to be a pointer to a std::string
		*  - "%color" => Expected values: "#RRGGBB" ([00-FF] each). "val" is assumed to be a pointer to a mrpt::utils::TColor
		*  - "%pose2d" => Expects "X Y YAW_DEG". "Val" is a pointer to mrpt::poses::CPose2D
		*  - "%pose2d_ptr3d" => Expects "X Y YAW_DEG". "Val" is a pointer to mrpt::poses::CPose3D
		*/
	struct TParamEntry
	{
		const char* frmt;  
		void *val;

		TParamEntry() : frmt(NULL),val(NULL) {}
		TParamEntry(const char* frmt_,void *val_) : frmt(frmt_),val(val_) {}

		/** Tries to parse the given input string according to the expected format, then store the result in "*val" 
		  * \exception std::runtime_error On format errors.
		  */
		void parse(const std::string & str,const std::string & varName,const char* function_name_context="") const;
	};


	void parse_xmlnode_attribs(
		const rapidxml::xml_node<char> &xml_node,
		const std::map<std::string,TParamEntry> &params,
		const char* function_name_context = "");

	/** Check whether the given XML node name matches any of the param list
	  * \return false if no match found 
	  * \sa parse_xmlnode_children_as_param
	  */
	bool parse_xmlnode_as_param(
		const rapidxml::xml_node<char> &xml_node,
		const std::map<std::string,TParamEntry> &params,
		const char* function_name_context="");

	/** Call \a parse_xmlnode_as_param() for all children nodes of the given node. */
	void parse_xmlnode_children_as_param(
		const rapidxml::xml_node<char> &xml_node,
		const std::map<std::string,TParamEntry> &params,
		const char* function_name_context="");

	// Bits:
	
	/** Parses a string like "XXX YYY PHI" with X,Y in meters, PHI in degrees, and returns 
	  * a vec3 with [x,y,phi] with angle in radians. Raises an exception upon malformed string.
	  */
	vec3 parseXYPHI(const std::string &s, bool allow_missing_angle = false, double default_angle_radians=0.0);


}
