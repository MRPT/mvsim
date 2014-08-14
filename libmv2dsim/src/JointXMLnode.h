/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#pragma once

#include <rapidxml.hpp>

namespace mv2dsim
{
	/** Proxy class to a set of XML nodes to appear as if they where one. */
	template <typename Ch=char>
	class JointXMLnode
	{
	private:
		typedef std::vector<const rapidxml::xml_node<Ch>*> TListNodes;
		TListNodes m_nodes;

	public:
		void add(const rapidxml::xml_node<Ch>* node) { m_nodes.push_back(node); }
	
		const rapidxml::xml_node<Ch> * first_node(const char* name) 
		{
			const rapidxml::xml_node<Ch> *ret=NULL;
			for (TListNodes::const_iterator it=m_nodes.begin();it!=m_nodes.end();++it) {
				ret = (*it)->first_node(name);
				if (ret!=NULL) return ret;
			}
			return ret;
		}

	};
}
