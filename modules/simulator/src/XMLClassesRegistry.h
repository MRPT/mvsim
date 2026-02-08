/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

#include <map>
#include <rapidxml.hpp>
#include <string>

namespace mvsim
{
/** Storage of XML "class-like" entries used, for example, in the <vehicle>
 * class factory. */
class XmlClassesRegistry
{
   private:
	const std::string tagname_;	 //!< xml tag
	struct TXMLData
	{
		rapidxml::xml_document<>* xml_doc;
		std::string* xml_data;	// Must be kept alloc'ed during the entire life
								// of xml_doc!!

		TXMLData() : xml_doc(nullptr), xml_data(nullptr) {}
		~TXMLData()
		{
			if (xml_doc) delete xml_doc;
			if (xml_data) delete xml_data;
		}
	};
	std::map<std::string, TXMLData> classes_;

   public:
	/** Define the xml tag, e.g. "vehicle:class" for "<vehicle:class
	 * name='xxx'>...</vehicle:class>" */
	XmlClassesRegistry(const std::string& xml_class_tag) : tagname_(xml_class_tag) {}

	/** Return an XML node with the class definition, or nullptr if not found */
	const rapidxml::xml_node<char>* get(const std::string& xml_node_class) const;

	/** Register a new class, given its XML definition as a text block */
	void add(const std::string& input_xml_node_class);

};	// end class
}  // namespace mvsim
