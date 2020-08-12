/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/filesystem.h>  // extractFileDirectory()
#include <mvsim/World.h>

#include <algorithm>  // count()
#include <iostream>  // for debugging
#include <map>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <stdexcept>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

MRPT_TODO("Replace if-else chain with a node load registry")

void World::load_from_XML(
	const std::string& xml_text, const std::string& fileNameForPath)
{
	using namespace std;
	using namespace rapidxml;

	// Extract base path of file:
	m_base_path =
		mrpt::system::trim(mrpt::system::extractFileDirectory(fileNameForPath));
	// printf("[World] INFO: Using base path='%s'\n",m_base_path.c_str());

	auto lck = mrpt::lockHelper(m_world_cs);  // Protect multithread access

	// Clear the existing world.
	this->clear_all();

	// Parse the XML input:
	rapidxml::xml_document<> xml;
	char* input_str = const_cast<char*>(xml_text.c_str());
	try
	{
		xml.parse<0>(input_str);
	}
	catch (rapidxml::parse_error& e)
	{
		unsigned int line =
			static_cast<long>(std::count(input_str, e.where<char>(), '\n') + 1);
		throw std::runtime_error(mrpt::format(
			"XML parse error (Line %u): %s", static_cast<unsigned>(line),
			e.what()));
	}

	// Sanity checks:
	const xml_node<>* root = xml.first_node();
	if (!root)
		throw runtime_error(
			"XML parse error: No root node found (empty file?)");
	if (0 != strcmp(root->name(), "mvsim_world"))
		throw runtime_error(mrpt::format(
			"XML root element is '%s' ('mvsim_world' expected)", root->name()));

	// Optional: format version attrib:
	const xml_attribute<>* attrb_version = root->first_attribute("version");
	int version_major = 1, version_min = 0;
	if (attrb_version)
	{
		int ret = sscanf(
			attrb_version->value(), "%i.%i", &version_major, &version_min);
		if (ret != 2)
			throw runtime_error(mrpt::format(
				"Error parsing version attribute: '%s' ('%%i.%%i' "
				"expected)",
				attrb_version->value()));
	}

	// Process all nodes:
	// ------------------------------------------------
	xml_node<>* node = root->first_node();
	while (node)
	{
		// <element class='*'> entries:
		if (!strcmp(node->name(), "element"))
		{
			WorldElementBase::Ptr e = WorldElementBase::factory(this, node);
			m_world_elements.emplace_back(e);
			m_simulableObjects.push_back(
				std::dynamic_pointer_cast<Simulable>(e));
		}
		// <vehicle> entries:
		else if (!strcmp(node->name(), "vehicle"))
		{
			VehicleBase::Ptr veh = VehicleBase::factory(this, node);
			// Assign each vehicle a unique "index" number
			veh->setVehicleIndex(m_vehicles.size());

			MRPT_TODO("Check for duplicated names")
			m_vehicles.insert(VehicleList::value_type(veh->getName(), veh));
			m_simulableObjects.push_back(
				std::dynamic_pointer_cast<Simulable>(veh));
		}
		// <vehicle:class> entries:
		else if (!strcmp(node->name(), "vehicle:class"))
		{
			VehicleBase::register_vehicle_class(node);
		}
		// <block> entries:
		else if (!strcmp(node->name(), "block"))
		{
			Block::Ptr block = Block::factory(this, node);
			insertBlock(block);
		}
		// <block:class> entries:
		else if (!strcmp(node->name(), "block:class"))
		{
			Block::register_block_class(node);
		}
		// <gui> </gui> params:
		else if (!strcmp(node->name(), "gui"))
		{
			m_gui_options.parse_from(*node);
		}
		// <walls> </walls> params:
		else if (!strcmp(node->name(), "walls"))
		{
			process_load_walls(*node);
		}
		else
		{
			// Default: Check if it's a parameter:
			if (!parse_xmlnode_as_param(*node, m_other_world_params))
			{
				// Unknown element!!
				MRPT_LOG_WARN_STREAM(
					"[World::load_from_XML] *Warning* Ignoring "
					"unknown XML node type '"
					<< node->name());
			}
		}

		// Move on to next node:
		node = node->next_sibling(nullptr);
	}
}
