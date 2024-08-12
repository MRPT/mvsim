/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "XMLClassesRegistry.h"

#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>

#include <algorithm>  // count()

using namespace mvsim;
using namespace std;

const rapidxml::xml_node<char>* XmlClassesRegistry::get(const std::string& xml_node_class) const
{
	map<string, TXMLData>::const_iterator it = classes_.find(xml_node_class);
	if (it == classes_.end())
		return nullptr;
	else
		return it->second.xml_doc->first_node();
}

void XmlClassesRegistry::add(const std::string& input_xml_node_class)
{
	// Parse the string as if it was an XML file:
	std::string* xml_node_class = new std::string(input_xml_node_class);

	char* input_str = const_cast<char*>(xml_node_class->c_str());
	rapidxml::xml_document<>* xml = new rapidxml::xml_document<>();
	try
	{
		xml->parse<0>(input_str);

		// sanity checks:
		// e.g. "vehicle:class"
		const rapidxml::xml_node<>* root_node = xml->first_node(tagname_.c_str());
		if (!root_node)
			throw runtime_error(
				mrpt::format("[XmlClassesRegistry] Missing XML node <%s>", tagname_.c_str()));

		const rapidxml::xml_attribute<>* att_name = root_node->first_attribute("name");
		if (!att_name || !att_name->value())
			throw runtime_error(mrpt::format(
				"[XmlClassesRegistry] Missing mandatory attribute "
				"'name' in node <%s>",
				tagname_.c_str()));

		const string sClassName = att_name->value();

		// All OK:
		TXMLData& d = classes_[sClassName];
		d.xml_doc = xml;
		d.xml_data = xml_node_class;
	}
	catch (const rapidxml::parse_error& e)
	{
		unsigned int line = static_cast<long>(std::count(input_str, e.where<char>(), '\n') + 1);
		delete xml;
		throw std::runtime_error(mrpt::format(
			"[XmlClassesRegistry] XML parse error (Line %u): %s", static_cast<unsigned>(line),
			e.what()));
	}
	catch (const std::exception&)
	{
		delete xml;
		throw;
	}
}
