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
     struct TXMLAttribs
     {
     	const char * name;
     	const char * frmt;
     	void * ptr;
     };

	struct TParamEntry
	{
		/** Normal case: sscanf()-like specifiers, and "void*" pointing to corresponding variable type.
		  * Special cases: 
		  *  - "%s" => "val" is assumed to be a pointer to a std::string
		  *  - "%color" => Expected values: "#RRGGBB" ([00-FF] each). "val" is assumed to be a pointer to a mrpt::utils::TColor
		  */
		const char* frmt;  
		void *val;

		TParamEntry() : frmt(NULL),val(NULL) {}
		TParamEntry(const char* frmt_,void *val_) : frmt(frmt_),val(val_) {}
	};


	void parse_xmlnode_attribs(
		const rapidxml::xml_node<char> &xml_node,
		const TXMLAttribs * attribs,
		const size_t nAttribs,
		const char* function_name_context = "");

	/** Check whether the given XML node name matches any of the param list
	  * \return false if no match found */
	bool parse_xmlnode_as_param(
		const rapidxml::xml_node<char> &xml_node,
		const std::map<std::string,TParamEntry> &params);


	// Bits:
	
	/** Parses a string like "XXX YYY PHI" with X,Y in meters, PHI in degrees, and returns 
	  * a vec3 with [x,y,phi] with angle in radians. Raises an exception upon malformed string.
	  */
	vec3 parseXYPHI(const std::string &s, bool allow_missing_angle = false, double default_angle=0.0);


}
