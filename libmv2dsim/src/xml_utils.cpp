/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/basic_types.h>
#include "xml_utils.h"

#include <cstdio>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/utils/TColor.h>

using namespace rapidxml;
using namespace mv2dsim;

void mv2dsim::parse_xmlnode_attribs(
	const rapidxml::xml_node<char> &xml_node,
	const TXMLAttribs * attribs,
	const size_t nAttribs,
	const char* function_name_context)
{
	for (size_t i=0;i<nAttribs;i++)
	{
		const rapidxml::xml_attribute<char> * attr = xml_node.first_attribute( attribs[i].name );
		if (attr && attr->value())
		{
			const std::string sAttr = attr->value();
			void *ptr = attribs[i].ptr;

			// Special cases:
			// "%s" ==> std::strings
			if (std::string(attribs[i].frmt)==std::string("%s"))
			{
				char auxStr[512];
				if (1!=::sscanf(sAttr.c_str(), attribs[i].frmt,auxStr))
					throw std::runtime_error(mrpt::format("%s Error parsing attribute '%s'='%s' (Expected format:'%s')",function_name_context,attr->name(),attr->value(),attribs[i].frmt ));
				std::string & str = *reinterpret_cast<std::string*>(attribs[i].ptr);
				str = auxStr;
			}
			else
			// "%color" ==> TColor
			if (std::string(attribs[i].frmt)==std::string("%color"))
			{
				// HTML-like format:
				if (!(sAttr.size()>1 && sAttr[0]=='#'))
					throw std::runtime_error(mrpt::format("%s Error parsing attribute '%s'='%s' (Expected format:'#RRGGBB')",function_name_context,attr->name(),attr->value() ));

				unsigned int r,g,b;
				if (3!=::sscanf(sAttr.c_str()+1, "%2x%2x%2x", &r, &g, &b))
					throw std::runtime_error(mrpt::format("%s Error parsing attribute '%s'='%s' (Expected format:'#RRGGBB')",function_name_context,attr->name(),attr->value() ));
				mrpt::utils::TColor & col= *reinterpret_cast<mrpt::utils::TColor*>(attribs[i].ptr);
				col.R = r;
				col.G = g;
				col.B = b;
			}
			else
			{
				// Generic parse:
				if (1!=::sscanf(sAttr.c_str(), attribs[i].frmt,ptr))
					throw std::runtime_error(mrpt::format("%s Error parsing attribute '%s'='%s' (Expected format:'%s')",function_name_context,attr->name(),attr->value(),attribs[i].frmt ));
			}

		}
	} // end for
}


bool mv2dsim::parse_xmlnode_as_param(
	const rapidxml::xml_node<char> &xml_node,
	const std::map<std::string,TParamEntry> &params)
{
	std::map<std::string,TParamEntry>::const_iterator it_param = params.find(xml_node.name());

	if (it_param != params.end() )
	{
		// parse parameter:
		if (1 != ::sscanf(xml_node.value(),it_param->second.frmt, it_param->second.val ) )
		{
			throw std::runtime_error(
				mrpt::format(
					"Error parsing entry '%s' with expected format '%s' and content '%s'",
					xml_node.name(), it_param->second.frmt, xml_node.value()
					) );
		}
		return true;
	}
	return false;
}

/** Call \a parse_xmlnode_as_param() for all children nodes of the given node. */
void mv2dsim::parse_xmlnode_children_as_param(
	const rapidxml::xml_node<char> &root,
	const std::map<std::string,TParamEntry> &params)
{
	rapidxml::xml_node<> *node = root.first_node();
	while (node)
	{
		parse_xmlnode_as_param(*node,params);
		node = node->next_sibling(NULL); // Move on to next node
	}
}

/** Parses a string like "XXX YYY PHI" with X,Y in meters, PHI in degrees, and returns 
	* a vec3 with [x,y,phi] with angle in radians. Raises an exception upon malformed string.
	*/
vec3 mv2dsim::parseXYPHI(const std::string &s, bool allow_missing_angle, double default_angle)
{
	vec3 v;  v.vals[2]=default_angle;// Default ang.

	int na = ::sscanf(s.c_str(),"%lf %lf %lf",&v.vals[0],&v.vals[1],&v.vals[2] );

	if ( (na!=3 && !allow_missing_angle) || (na!=2 && na!=3 && allow_missing_angle))
		throw std::runtime_error(mrpt::format("Malformed pose string: '%s'",s.c_str()));

	return v;
}
