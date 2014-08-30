/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#pragma once

#include <rapidxml.hpp>
#include <stdexcept>

namespace mvsim
{
	/** Proxy class to a set of XML nodes to appear as if they where one. */
	template <typename Ch=char>
	class JointXMLnode
	{
	public:
		typedef std::vector<const rapidxml::xml_node<Ch>*> TListNodes;

	private:
		TListNodes m_nodes;

	public:
		void add(const rapidxml::xml_node<Ch>* node) { m_nodes.push_back(node); }

		const rapidxml::xml_node<Ch> * first_node(const char* name)
		{
			const rapidxml::xml_node<Ch> *ret=NULL;
			for (typename TListNodes::const_iterator it=m_nodes.begin();it!=m_nodes.end();++it) {
				ret = (*it)->first_node(name);
				if (ret!=NULL) return ret;
			}
			return ret;
		}

		TListNodes & getListOfNodes() { return m_nodes; }

		// Iterators-like interface ----------------------
		class iterator
		{
		public:
			template <typename Ch_>
			friend class JointXMLnode;

			// ++it
			iterator & operator ++()
			{
				if (!current) throw std::runtime_error("++ called on end() iterator!?");
				current = current->next_sibling();
				JointXMLnode<Ch>::TListNodes &lst = parent.getListOfNodes();
				while (!current && lst_idx<lst.size())
				{
					lst_idx++;
					if (lst_idx<lst.size())
						current=lst[lst_idx]->first_node();
					else current=NULL;
				}
				return *this;
			}

			rapidxml::xml_node<Ch> * operator->() const
			{
				if (!current) throw std::runtime_error("-> called on end() iterator!?");
				return current;
			}

			rapidxml::xml_node<Ch> * operator*() const
			{
				if (!current) throw std::runtime_error("* called on end() iterator!?");
				return current;
			}

			bool operator ==(const iterator &it) const {
				return (this->current==it.current) && (this->lst_idx==it.lst_idx) && (&this->parent==&it.parent);
			}
			bool operator !=(const iterator &it) const {
				return !(*this==it);
			}

		private:
			// begin():
			iterator(JointXMLnode<Ch>&pa) : parent(pa),lst_idx(0),current(NULL)
			{
				JointXMLnode<Ch>::TListNodes &lst = parent.getListOfNodes();
				while (!current && lst_idx<lst.size())
				{
					current = lst[lst_idx]->first_node();
					if (!current) lst_idx++;
				}
			}
			// end()
			iterator(JointXMLnode<Ch>&pa, size_t idx) : parent(pa),lst_idx(idx),current(NULL)
			{
			}

			JointXMLnode<Ch> &parent;
			size_t lst_idx;  // => lst.size() means this is "end()"
			rapidxml::xml_node<Ch> *current;

		}; // end class iterator


		iterator begin() { return iterator(*this); }
		iterator end() { return iterator(*this,m_nodes.size()); }

	};
}
