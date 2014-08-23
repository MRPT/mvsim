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
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/system/string_utils.h>

using namespace rapidxml;
using namespace mv2dsim;

/** Tries to parse the given input string according to the expected format, then store the result in "*val" 
	* \exception std::runtime_error On format errors.
	*/
void TParamEntry::parse(const std::string & str,const std::string & varName,const char* function_name_context) const
{
	// Special cases:
	// "%s" ==> std::strings
	if (std::string(frmt)==std::string("%s"))
	{
		char auxStr[512];
		if (1!=::sscanf(str.c_str(), frmt,auxStr))
			throw std::runtime_error(mrpt::format("%s Error parsing '%s'='%s' (Expected format:'%s')",function_name_context,varName.c_str(),str.c_str(),frmt ));
		std::string & str = *reinterpret_cast<std::string*>(val);
		str = auxStr;
	}
	// "%lf_deg" ==> DEG2RAD()
	else if (std::string(frmt)==std::string("%lf_deg"))
	{
		if (1!=::sscanf(str.c_str(), frmt,val))
			throw std::runtime_error(mrpt::format("%s Error parsing attribute '%s'='%s' (Expected format:'%s')",function_name_context,varName.c_str(),str.c_str(),frmt ));
		double &ang = *reinterpret_cast<double*>(val);
		ang = mrpt::utils::DEG2RAD(ang);
	}
	// "%bool" ==> bool*
	else if (std::string(frmt)==std::string("%bool"))
	{
		bool &bool_val = *reinterpret_cast<bool*>(val);

		const std::string sStr = mrpt::system::lowerCase( mrpt::system::trim(std::string(str)) );
		if (sStr=="1" ||  sStr=="true") bool_val=true;
		else if (sStr=="0" ||  sStr=="false") bool_val=false;
		else throw std::runtime_error(mrpt::format("%s Error parsing 'bool' attribute '%s'='%s' (Expected 'true' or 'false')",function_name_context,varName.c_str(),str.c_str()));
	}
	// "%color" ==> TColor
	else if (std::string(frmt)==std::string("%color"))
	{
		// HTML-like format:
		if (!(str.size()>1 && str[0]=='#'))
			throw std::runtime_error(mrpt::format("%s Error parsing '%s'='%s' (Expected format:'#RRGGBB[AA]')",function_name_context,varName.c_str(),str.c_str() ));

		unsigned int r,g,b,a=0xff;
		int ret = ::sscanf(str.c_str()+1, "%2x%2x%2x%2x", &r, &g, &b,&a);
		if (ret!=3 && ret!=4)
			throw std::runtime_error(mrpt::format("%s Error parsing '%s'='%s' (Expected format:'#RRGGBB[AA]')",function_name_context,varName.c_str(),str.c_str() ));
		mrpt::utils::TColor & col= *reinterpret_cast<mrpt::utils::TColor*>(val);
		col=mrpt::utils::TColor(r,g,b,a);
	}
	// "%pose2d"
	// "%pose2d_ptr3d"
	else if ( !strncmp(frmt,"%pose2d",strlen("%pose2d")))
	{
		double x,y,yaw;
		int ret = ::sscanf(str.c_str(), "%lf %lf %lf", &x,&y,&yaw);
		if (ret!=3)
			throw std::runtime_error(mrpt::format("%s Error parsing '%s'='%s' (Expected format:'X Y YAW_DEG')",function_name_context,varName.c_str(),str.c_str() ));

		// User provides angles in deg:
		yaw = mrpt::utils::DEG2RAD(yaw);

		const mrpt::poses::CPose2D p(x,y,yaw);

		// Sub-cases:
		if (!strcmp(frmt,"%pose2d"))
		{
			mrpt::poses::CPose2D & pp= *reinterpret_cast<mrpt::poses::CPose2D*>(val);
			pp = p;
		}
		else if (!strcmp(frmt,"%pose2d_ptr3d"))
		{
			mrpt::poses::CPose3D & pp= *reinterpret_cast<mrpt::poses::CPose3D*>(val);
			pp = mrpt::poses::CPose3D(p);
		}
		else
			throw std::runtime_error( mrpt::format("%s Error: Unknown format specifier '%s'",function_name_context,frmt) );
	}
	else
	{
		// Generic parse:
		if (1!=::sscanf(str.c_str(), frmt,val))
			throw std::runtime_error(mrpt::format("%s Error parsing attribute '%s'='%s' (Expected format:'%s')",function_name_context,varName.c_str(),str.c_str(),frmt ));
	}
}


void mv2dsim::parse_xmlnode_attribs(
	const rapidxml::xml_node<char> &xml_node,
	const std::map<std::string,TParamEntry> &params,
	const char* function_name_context)
{
	for (std::map<std::string,TParamEntry>::const_iterator it=params.begin();it!=params.end();++it)
	{
		const rapidxml::xml_attribute<char> * attr = xml_node.first_attribute( it->first.c_str() );
		if (attr && attr->value())
			it->second.parse( attr->value(), attr->name(), function_name_context );
	}
}

bool mv2dsim::parse_xmlnode_as_param(
	const rapidxml::xml_node<char> &xml_node,
	const std::map<std::string,TParamEntry> &params,
	const char* function_name_context)
{
	std::map<std::string,TParamEntry>::const_iterator it_param = params.find(xml_node.name());

	if (it_param != params.end() )
	{
		// parse parameter:
		it_param->second.parse( xml_node.value(), xml_node.name(), function_name_context );
		return true;
	}
	return false;
}

/** Call \a parse_xmlnode_as_param() for all children nodes of the given node. */
void mv2dsim::parse_xmlnode_children_as_param(
	const rapidxml::xml_node<char> &root,
	const std::map<std::string,TParamEntry> &params,
	const char* function_name_context)
{
	rapidxml::xml_node<> *node = root.first_node();
	while (node)
	{
		parse_xmlnode_as_param(*node,params,function_name_context);
		node = node->next_sibling(NULL); // Move on to next node
	}
}

/** Parses a string like "XXX YYY PHI" with X,Y in meters, PHI in degrees, and returns 
	* a vec3 with [x,y,phi] with angle in radians. Raises an exception upon malformed string.
	*/
vec3 mv2dsim::parseXYPHI(const std::string &s, bool allow_missing_angle, double default_angle_radians)
{
	vec3 v;  
	v.vals[2]= mrpt::utils::RAD2DEG(default_angle_radians);// Default ang.

	int na = ::sscanf(s.c_str(),"%lf %lf %lf",&v.vals[0],&v.vals[1],&v.vals[2] );
	
	// User provides numbers as degrees:
	v.vals[2] = mrpt::utils::DEG2RAD( v.vals[2] );

	if ( (na!=3 && !allow_missing_angle) || (na!=2 && na!=3 && allow_missing_angle))
		throw std::runtime_error(mrpt::format("Malformed pose string: '%s'",s.c_str()));

	return v;
}


/** Parses a <shape><pt>X Y</pt>...</shape> XML node into a mrpt::math::TPolygon2D
	* \exception std::exception On syntax errors, etc.
	*/
void mv2dsim::parse_xmlnode_shape(
	const rapidxml::xml_node<char> &xml_node,
	mrpt::math::TPolygon2D &out_poly,
	const char* function_name_context)
{
	out_poly.clear();

	for (rapidxml::xml_node<char> *pt_node=xml_node.first_node("pt"); pt_node ; pt_node = pt_node->next_sibling("pt") )
	{
		if (!pt_node->value())
			throw std::runtime_error(mrpt::format("%s Error: <pt> node seems empty.",function_name_context));

		mrpt::math::TPoint2D pt;
		const char* str_val = pt_node->value();
		if (2!=::sscanf(str_val, "%lf %lf",&pt.x,&pt.y))
			throw std::runtime_error(mrpt::format("%s Error parsing <pt> node: '%s' (Expected format:'<pt>X Y</pt>')",function_name_context,str_val));
		
		out_poly.push_back(pt);
	}

	if (out_poly.size()<3)
		throw std::runtime_error(mrpt::format("%s Error: <shape> node requires 3 or more <pt>X Y</pt> entries.",function_name_context));
}
