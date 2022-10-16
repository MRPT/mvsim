/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/system/filesystem.h>	 // extractFileDirectory()
#include <mvsim/World.h>

#include <algorithm>  // count()
#include <iostream>	 // for debugging
#include <map>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>
#include <stdexcept>

#include "parse_utils.h"
#include "xml_utils.h"

using namespace mvsim;
using namespace std;

MRPT_TODO("Replace if-else chain with a node load registry")

void World::load_from_XML_file(const std::string& xmlFileNamePath)
{
	rapidxml::file<> fil_xml(xmlFileNamePath.c_str());
	load_from_XML(fil_xml.data(), xmlFileNamePath.c_str());
}

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

	// Create anchor object for standalone (environment-attached) sensors:
	DummyInvisibleBlock::Ptr standaloneSensorHost =
		std::make_shared<DummyInvisibleBlock>(this);
	m_simulableObjects.emplace("__standaloneSensorHost", standaloneSensorHost);

	// Parse the XML input:
	const auto [xml, root] = readXmlTextAndGetRoot(xml_text, fileNameForPath);
	(void)xml;	// unused

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

	// ------------------------------------------------
	// Process all xml nodes
	// ------------------------------------------------
	xml_node<>* node = root->first_node();
	while (node)
	{
		internal_recursive_parse_XML(node, m_base_path);

		// Move on to next node:
		node = node->next_sibling(nullptr);
	}
}

void World::internal_recursive_parse_XML(
	const void* nodeOpaquePtr, const std::string& currentBasePath)
{
	using namespace rapidxml;
	using namespace std::string_literals;

	const rapidxml::xml_node<>* node =
		reinterpret_cast<const rapidxml::xml_node<>*>(nodeOpaquePtr);

	// push relative directory state:
	const auto savedBasePath = m_base_path;
	m_base_path = currentBasePath;

	// <element class='*'> entries:
	if (!strcmp(node->name(), "element"))
	{
		WorldElementBase::Ptr e = WorldElementBase::factory(this, node);
		m_world_elements.emplace_back(e);
		m_simulableObjects.emplace(
			e->getName(), std::dynamic_pointer_cast<Simulable>(e));
	}
	// <vehicle> entries:
	else if (!strcmp(node->name(), "vehicle"))
	{
		VehicleBase::Ptr veh = VehicleBase::factory(this, node);
		// Assign each vehicle a unique "index" number
		veh->setVehicleIndex(m_vehicles.size());

		ASSERTMSG_(
			m_vehicles.count(veh->getName()) == 0,
			mrpt::format(
				"Duplicated vehicle name: '%s'", veh->getName().c_str()));

		m_vehicles.insert(VehicleList::value_type(veh->getName(), veh));
		m_simulableObjects.emplace(
			veh->getName(), std::dynamic_pointer_cast<Simulable>(veh));
	}
	// <vehicle:class> entries:
	else if (!strcmp(node->name(), "vehicle:class"))
	{
		VehicleBase::register_vehicle_class(node);
	}
	// top-level <sensor> entries:
	else if (!strcmp(node->name(), "sensor"))
	{
		DummyInvisibleBlock::Ptr standaloneSensorHost =
			std::dynamic_pointer_cast<DummyInvisibleBlock>(
				m_simulableObjects.find("__standaloneSensorHost")->second);
		ASSERT_(standaloneSensorHost);

		SensorBase::Ptr sensor =
			SensorBase::factory(*standaloneSensorHost, node);
		standaloneSensorHost->add_sensor(sensor);
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
	// <include file="path" /> entries:
	else if (!strcmp(node->name(), "include"))
	{
		auto fileAttrb = node->first_attribute("file");
		ASSERTMSG_(
			fileAttrb,
			"XML tag '<include />' must have a 'file=\"xxx\"' attribute)");

		const std::string relFile = mvsim::parse(fileAttrb->value(), {});

		const auto absFile = this->resolvePath(relFile);
		MRPT_LOG_DEBUG_STREAM(
			"XML parser: including file: '" << absFile << "'");

		std::map<std::string, std::string> vars;
		for (auto attr = node->first_attribute(); attr;
			 attr = attr->next_attribute())
		{
			if (strcmp(attr->name(), "file") == 0) continue;
			vars[attr->name()] = attr->value();
		}

		const auto [xml, root] = readXmlAndGetRoot(absFile, vars);
		(void)xml;	// unused

		// recursive parse:
		const auto newBasePath =
			mrpt::system::trim(mrpt::system::extractFileDirectory(absFile));
		internal_recursive_parse_XML(root, newBasePath);
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

	// pop relative directory state:
	m_base_path = savedBasePath;
}
