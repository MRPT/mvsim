/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#include "XMLClassesRegistry.h"

#include <iostream>

#include <mrpt/version.h>
#if MRPT_VERSION<0x199
#include <mrpt/utils/utils_defs.h>  // mrpt::format()
#else
#include <mrpt/core/format.h>
#include <mrpt/core/bits_math.h>
#endif

#include <algorithm> // count()

using namespace mvsim;
using namespace std;

const rapidxml::xml_node<char>* XmlClassesRegistry::get(
	const std::string& xml_node_vehicle_class) const
{
	map<string, TXMLData>::const_iterator it =
		m_classes.find(xml_node_vehicle_class);
	if (it == m_classes.end())
		return NULL;
	else
		return it->second.xml_doc->first_node();
}

void XmlClassesRegistry::add(const std::string& input_xml_node_vehicle_class)
{
	// Parse the string as if it was an XML file:
	std::string* xml_node_vehicle_class =
		new std::string(input_xml_node_vehicle_class);

	char* input_str = const_cast<char*>(xml_node_vehicle_class->c_str());
	rapidxml::xml_document<>* xml = new rapidxml::xml_document<>();
	try
	{
		xml->parse<0>(input_str);

		// sanity checks:
		const rapidxml::xml_node<>* root_node =
			xml->first_node(m_tagname.c_str());  //"vehicle:class"
		if (!root_node)
			throw runtime_error(
				mrpt::format(
					"[XmlClassesRegistry] Missing XML node <%s>",
					m_tagname.c_str()));

		const rapidxml::xml_attribute<>* att_name =
			root_node->first_attribute("name");
		if (!att_name || !att_name->value())
			throw runtime_error(
				mrpt::format(
					"[VehicleClassesRegistry] Missing mandatory attribute "
					"'name' in node <%s>",
					m_tagname.c_str()));

		const string sClassName = att_name->value();

		// All OK:
		TXMLData& d = m_classes[sClassName];
		d.xml_doc = xml;
		d.xml_data = xml_node_vehicle_class;
	}
	catch (rapidxml::parse_error& e)
	{
		unsigned int line =
			static_cast<long>(std::count(input_str, e.where<char>(), '\n') + 1);
		delete xml;
		throw std::runtime_error(
			mrpt::format(
				"[XmlClassesRegistry] XML parse error (Line %u): %s",
				static_cast<unsigned>(line), e.what()));
	}
	catch (std::exception&)
	{
		delete xml;
		throw;
	}
}
