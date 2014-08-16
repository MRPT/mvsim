/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include "xml_utils.h"

#include <cstdio>
#include <mrpt/utils/utils_defs.h>

using namespace rapidxml;
using namespace mv2dsim;

void mv2dsim::parse_xmlnode_attribs(
	const rapidxml::xml_node<char> &xml_node,
	const TXMLAttribs * attribs,
	const size_t nAttribs,
	const char* function_name_context)
{
	for (size_t i=0;i<sizeof(attribs)/sizeof(attribs[0]) ;i++)
	{
		const rapidxml::xml_attribute<char> * attr = xml_node.first_attribute( attribs[i].name );
		if (attr && attr->value())
		{
			const std::string sAttr = attr->value();
			if (1!=::sscanf(sAttr.c_str(), attribs[i].frmt,attribs[i].ptr))
				throw std::runtime_error(mrpt::format("%s Error parsing attribute '%s'='%s' (Expected format:'%s')",function_name_context,attr->name(),attr->value(),attribs[i].frmt ));
		}
	}
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
