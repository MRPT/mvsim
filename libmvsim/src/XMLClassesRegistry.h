/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#pragma once

#include <rapidxml.hpp>

#include <map>
#include <string>

namespace mvsim
{
	/** Storage of XML "class-like" entries used, for example, in the <vehicle> class factory. */
	class XmlClassesRegistry
	{
	private:
		const std::string m_tagname; //!< xml tag
		struct TXMLData
		{
			rapidxml::xml_document<>* xml_doc;
			std::string *xml_data; // Must be kept alloc'ed during the entire life of xml_doc!!

			TXMLData() : xml_doc(NULL), xml_data(NULL) { }
			~TXMLData() 
			{
				if (xml_doc)  delete xml_doc;
				if (xml_data) delete xml_data;
			}
		};
		std::map<std::string,TXMLData> m_classes;

	public:
		/** Define the xml tag, e.g. "vehicle:class" for "<vehicle:class name='xxx'>...</vehicle:class>" */
		XmlClassesRegistry(const std::string & xml_class_tag) : m_tagname(xml_class_tag)
		{}

		/** Return an XML node with the class definition, or NULL if not found */
		const rapidxml::xml_node<char>* get(const std::string &xml_node_class) const;

		/** Register a new class, given its XML definition as a text block */
		void add(const std::string &input_xml_node_class);

	}; // end class

}
