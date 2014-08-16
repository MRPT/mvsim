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

	/** Return false if none found */
	bool parse_xmlnode_children(
		const rapidxml::xml_node<char> &xml_node,
		const std::map<std::string,TParamEntry> &params);


}
