/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

#include <rapidxml.hpp>
#include <stdexcept>
#include <vector>

namespace mvsim
{
/** Proxy class to a set of XML nodes to appear as if they where one. */
template <typename Ch>
class JointXMLnode
{
   public:
	typedef std::vector<const rapidxml::xml_node<Ch>*> TListNodes;

   private:
	TListNodes m_nodes;

   public:
	void add(const rapidxml::xml_node<Ch>* node) { m_nodes.push_back(node); }

	const rapidxml::xml_node<Ch>* first_node(const char* name) const
	{
		const rapidxml::xml_node<Ch>* ret = nullptr;
		for (const auto& node : m_nodes)
		{
			ret = node->first_node(name);
			if (ret != nullptr) return ret;
		}
		return ret;
	}

	TListNodes& getListOfNodes() { return m_nodes; }
	// Iterators-like interface ----------------------
	class iterator
	{
	   public:
		template <typename Ch_>
		friend class JointXMLnode;

		// ++it
		iterator& operator++()
		{
			if (!current)
				throw std::runtime_error("++ called on end() iterator!?");
			current = current->next_sibling();
			JointXMLnode<Ch>::TListNodes& lst = parent.getListOfNodes();
			while (!current && lst_idx < lst.size())
			{
				lst_idx++;
				if (lst_idx < lst.size())
					current = lst[lst_idx]->first_node();
				else
					current = nullptr;
			}
			return *this;
		}

		rapidxml::xml_node<Ch>* operator->() const
		{
			if (!current)
				throw std::runtime_error("-> called on end() iterator!?");
			return current;
		}

		rapidxml::xml_node<Ch>* operator*() const
		{
			if (!current)
				throw std::runtime_error("* called on end() iterator!?");
			return current;
		}

		bool operator==(const iterator& it) const
		{
			return (this->current == it.current) &&
				   (this->lst_idx == it.lst_idx) &&
				   (&this->parent == &it.parent);
		}
		bool operator!=(const iterator& it) const { return !(*this == it); }

	   private:
		// begin():
		iterator(JointXMLnode<Ch>& pa)
			: parent(pa), lst_idx(0), current(nullptr)
		{
			JointXMLnode<Ch>::TListNodes& lst = parent.getListOfNodes();
			while (!current && lst_idx < lst.size())
			{
				current = lst[lst_idx]->first_node();
				if (!current) lst_idx++;
			}
		}
		// end()
		iterator(JointXMLnode<Ch>& pa, size_t idx)
			: parent(pa), lst_idx(idx), current(nullptr)
		{
		}

		JointXMLnode<Ch>& parent;
		size_t lst_idx;	 // => lst.size() means this is "end()"
		rapidxml::xml_node<Ch>* current;

	};	// end class iterator

	iterator begin() { return iterator(*this); }
	iterator end() { return iterator(*this, m_nodes.size()); }
};

}  // namespace mvsim
