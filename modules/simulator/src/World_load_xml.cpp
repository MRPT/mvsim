/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <mrpt/core/format.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/system/filesystem.h>	 // extractFileDirectory()
#include <mrpt/topography/conversions.h>
#include <mvsim/World.h>

#include <algorithm>  // count()
#include <map>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>
#include <sstream>
#include <stdexcept>

#include "parse_utils.h"
#include "xml_utils.h"

using namespace mvsim;
using namespace std;

void World::load_from_XML_file(const std::string& xmlFileNamePath)
{
	rapidxml::file<> fil_xml(xmlFileNamePath.c_str());
	load_from_XML(fil_xml.data(), xmlFileNamePath.c_str());
}

void World::load_from_XML(const std::string& xml_text, const std::string& fileNameForPath)
{
	using namespace std;
	using namespace rapidxml;

	std::string fileDir = mrpt::system::extractFileDirectory(fileNameForPath);
	if (fileDir.empty()) fileDir = ".";

	// Extract base path of file:
	basePath_ = mrpt::system::toAbsolutePath(fileDir, false /*canonical*/);
	// printf("[World] INFO: Using base path='%s'\n",basePath_.c_str());

	// Special variables:
	userDefinedVariables_["MVSIM_CURRENT_FILE_DIRECTORY"] = basePath_;

	auto lck = mrpt::lockHelper(world_cs_);	 // Protect multithread access

	// Clear the existing world.
	this->clear_all();

	// Create anchor object for standalone (environment-attached) sensors:
	DummyInvisibleBlock::Ptr standaloneSensorHost = std::make_shared<DummyInvisibleBlock>(this);

	simulableObjectsMtx_.lock();
	simulableObjects_.emplace("__standaloneSensorHost", standaloneSensorHost);
	simulableObjectsMtx_.unlock();

	// Parse the XML input:
	const auto [xml, root] = readXmlTextAndGetRoot(xml_text, fileNameForPath);
	(void)xml;	// unused

	if (0 != strcmp(root->name(), "mvsim_world"))
		throw runtime_error(
			mrpt::format("XML root element is '%s' ('mvsim_world' expected)", root->name()));

	// Optional: format version attrib:
	const xml_attribute<>* attrb_version = root->first_attribute("version");
	int version_major = 1, version_min = 0;
	if (attrb_version)
	{
		int ret = sscanf(attrb_version->value(), "%i.%i", &version_major, &version_min);
		if (ret != 2)
			throw runtime_error(mrpt::format(
				"Error parsing version attribute: '%s' ('%%i.%%i' "
				"expected)",
				attrb_version->value()));
	}

	// register tags:
	register_standard_xml_tag_parsers();

	// ------------------------------------------------
	// Process all xml nodes
	// ------------------------------------------------
	xml_node<>* node = root->first_node();
	while (node)
	{
		internal_recursive_parse_XML({node, basePath_});

		// Move on to next node:
		node = node->next_sibling(nullptr);
	}

	internal_initialize();
}

void World::register_standard_xml_tag_parsers()
{
	if (!xmlParsers_.empty()) return;

	register_tag_parser("vehicle", &World::parse_tag_vehicle);
	register_tag_parser("vehicle:class", &World::parse_tag_vehicle_class);

	register_tag_parser("block", &World::parse_tag_block);
	register_tag_parser("block:class", &World::parse_tag_block_class);

	register_tag_parser("element", &World::parse_tag_element);
	register_tag_parser("sensor", &World::parse_tag_sensor);
	register_tag_parser("gui", &World::parse_tag_gui);
	register_tag_parser("georeference", &World::parse_tag_georeference);
	register_tag_parser("lights", &World::parse_tag_lights);
	register_tag_parser("walls", &World::parse_tag_walls);
	register_tag_parser("include", &World::parse_tag_include);
	register_tag_parser("variable", &World::parse_tag_variable);
	register_tag_parser("for", &World::parse_tag_for);
	register_tag_parser("if", &World::parse_tag_if);
	register_tag_parser("marker", &World::parse_tag_marker);
}

void World::internal_recursive_parse_XML(const XmlParserContext& ctx)
{
	using namespace rapidxml;
	using namespace std::string_literals;

	const rapidxml::xml_node<>* node = ctx.node;
	ASSERT_(node);

	// push relative directory state:
	const auto savedBasePath = basePath_;
	basePath_ = ctx.currentBasePath;
	// Special variables:
	userDefinedVariables_["MVSIM_CURRENT_FILE_DIRECTORY"] = basePath_;

	// Known tag parser?
	if (auto itParser = xmlParsers_.find(node->name()); itParser != xmlParsers_.end())
	{
		itParser->second(ctx);
	}
	else
	{  // Default: Check if it's a parameter:
		if (!parse_xmlnode_as_param(*node, otherWorldParams_))
		{
			// Unknown element!!
			MRPT_LOG_WARN_STREAM(
				"[World::load_from_XML] *Warning* Ignoring unknown XML node type '" << node->name()
																					<< "'");
		}
	}

	// pop relative directory state:
	basePath_ = savedBasePath;
	// Special variables:
	userDefinedVariables_["MVSIM_CURRENT_FILE_DIRECTORY"] = basePath_;
}

void World::parse_tag_element(const XmlParserContext& ctx)
{
	// <element class='*'> entries:
	WorldElementBase::Ptr e = WorldElementBase::factory(this, ctx.node);
	worldElements_.emplace_back(e);

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());
	simulableObjects_.emplace(e->getName(), std::dynamic_pointer_cast<Simulable>(e));
}

void World::parse_tag_vehicle(const XmlParserContext& ctx)
{
	// <vehicle> entries:
	VehicleBase::Ptr veh = VehicleBase::factory(this, ctx.node);
	// Assign each vehicle a unique "index" number
	veh->setVehicleIndex(vehicles_.size());

	ASSERTMSG_(
		vehicles_.count(veh->getName()) == 0,
		mrpt::format("Duplicated vehicle name: '%s'", veh->getName().c_str()));

	vehicles_.insert(VehicleList::value_type(veh->getName(), veh));

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());
	simulableObjects_.emplace(veh->getName(), std::dynamic_pointer_cast<Simulable>(veh));
}

void World::parse_tag_vehicle_class(const XmlParserContext& ctx)
{
	// <vehicle:class> entries:
	VehicleBase::register_vehicle_class(*this, ctx.node);
}

void World::parse_tag_sensor(const XmlParserContext& ctx)
{
	// top-level <sensor> entries:
	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());

	DummyInvisibleBlock::Ptr standaloneSensorHost = std::dynamic_pointer_cast<DummyInvisibleBlock>(
		simulableObjects_.find("__standaloneSensorHost")->second);
	ASSERT_(standaloneSensorHost);

	SensorBase::Ptr sensor = SensorBase::factory(*standaloneSensorHost, ctx.node);
	standaloneSensorHost->add_sensor(sensor);
}

void World::parse_tag_block(const XmlParserContext& ctx)
{
	// <block> entries:
	Block::Ptr block = Block::factory(this, ctx.node);
	insertBlock(block);
	lut2d_objects_is_up_to_date_ = false;
}

void World::parse_tag_block_class(const XmlParserContext& ctx)
{
	//
	Block::register_block_class(*this, ctx.node);
}

void World::parse_tag_gui(const XmlParserContext& ctx)
{
	//
	guiOptions_.parse_from(*ctx.node, *this);
}

void World::parse_tag_lights(const XmlParserContext& ctx)
{
	lightOptions_.parse_from(*ctx.node, *this);
}

void World::parse_tag_georeference(const XmlParserContext& ctx)
{
	georeferenceOptions_.parse_from(*ctx.node, *this);

	// handle UTM:
	auto& g = georeferenceOptions_;

	if (g.world_is_utm)
	{
		ASSERTMSG_(
			g.world_to_enu_rotation == 0,
			"Cannot define both, <world_to_enu_rotation> and <world_is_utm>");

		ASSERTMSG_(
			!g.georefCoord.isClear(),
			"<world_is_utm> requires defining a valid reference geodetic coordinates too.");

		mrpt::topography::GeodeticToUTM(g.georefCoord, g.utmRef, g.utm_zone, g.utm_band);

		MRPT_LOG_INFO_STREAM(
			"Using UTM georeference: utm_zone=" << g.utm_zone << " geoRef: utmRef=" << g.utmRef);
	}
}

void World::parse_tag_walls(const XmlParserContext& ctx) { process_load_walls(*ctx.node); }

void World::parse_tag_include(const XmlParserContext& ctx)
{
	auto fileAttrb = ctx.node->first_attribute("file");
	ASSERTMSG_(fileAttrb, "XML tag '<include />' must have a 'file=\"xxx\"' attribute)");

	const std::string relFile = mvsim::parse(fileAttrb->value(), user_defined_variables());

	const auto absFile = this->local_to_abs_path(relFile);
	MRPT_LOG_DEBUG_STREAM("XML parser: including file: '" << absFile << "'");

	std::map<std::string, std::string> vars;
	// Inherit the user-defined variables from parent scope
	vars = user_defined_variables();
	// Plus new variables as XML attributes, local only:
	for (auto attr = ctx.node->first_attribute(); attr; attr = attr->next_attribute())
	{
		if (strcmp(attr->name(), "file") == 0) continue;
		vars[attr->name()] = attr->value();
	}

	const auto [xml, root] = readXmlAndGetRoot(absFile, vars);
	(void)xml;	// unused

	// recursive parse:
	const auto newBasePath = mrpt::system::extractFileDirectory(absFile);
	internal_recursive_parse_XML({root, newBasePath});
}

void World::parse_tag_variable(const XmlParserContext& ctx)
{
	auto nameAttr = ctx.node->first_attribute("name");
	ASSERTMSG_(nameAttr, "XML tag '<variable />' must have a 'name=\"xxx\"' attribute)");
	const auto name = nameAttr->value();

	auto valueAttr = ctx.node->first_attribute("value");
	ASSERTMSG_(valueAttr, "XML tag '<variable />' must have a 'value=\"xxx\"' attribute)");

	const std::string finalValue = mvsim::parse(valueAttr->value(), userDefinedVariables_);

	thread_local const bool MVSIM_VERBOSE_PARSE = mrpt::get_env<bool>("MVSIM_VERBOSE_PARSE", false);

	if (MVSIM_VERBOSE_PARSE)
	{
		printf(
			"[mvsim] Parsed <variable>: name='%s' value='%s' (original "
			"expression='%s')\n",
			name, finalValue.c_str(), valueAttr->value());
	}

	userDefinedVariables_[name] = finalValue;
}

void World::parse_tag_for(const XmlParserContext& ctx)
{
	auto varAttr = ctx.node->first_attribute("var");
	ASSERTMSG_(varAttr, "XML tag '<for />' must have a 'var=\"xxx\"' attribute)");
	const auto varName = varAttr->value();

	auto varFrom = ctx.node->first_attribute("from");
	ASSERTMSG_(varFrom, "XML tag '<for />' must have a 'from=\"xxx\"' attribute)");
	const auto fromStr = mvsim::parse(varFrom->value(), userDefinedVariables_);

	auto varTo = ctx.node->first_attribute("to");
	ASSERTMSG_(varTo, "XML tag '<for />' must have a 'to=\"xxx\"' attribute)");
	const auto toStr = mvsim::parse(varTo->value(), userDefinedVariables_);

	bool forBodyEmpty = true;

	for (auto childNode = ctx.node->first_node(); childNode; childNode = childNode->next_sibling())
	{
		forBodyEmpty = false;
		for (int curVal = std::stoi(fromStr); curVal <= std::stoi(toStr); curVal++)
		{
			userDefinedVariables_[varName] = std::to_string(curVal);
			internal_recursive_parse_XML({childNode, basePath_});
		}
	}

	if (forBodyEmpty)
	{
		MRPT_LOG_WARN_STREAM(
			"[World::load_from_XML] *Warning* <for ...> </for> loop has no "
			"contents (!): '"
			<< ctx.node->value() << "'");
	}
}

void World::parse_tag_if(const XmlParserContext& ctx)
{
	bool isTrue = evaluate_tag_if(*ctx.node);
	if (!isTrue) return;

	for (auto childNode = ctx.node->first_node(); childNode; childNode = childNode->next_sibling())
	{
		internal_recursive_parse_XML({childNode, basePath_});
	}
}

bool World::evaluate_tag_if(const rapidxml::xml_node<char>& node) const
{
	auto varCond = node.first_attribute("condition");
	ASSERTMSG_(varCond, "XML tag '<if />' must have a 'condition=\"xxx\"' attribute)");
	const auto str = mvsim::parse(varCond->value(), userDefinedVariables_);

	// is it "true"?
	std::optional<int> intVal;
	char* retStr = nullptr;
	const long long ret = std::strtoll(str.c_str(), &retStr, 0 /*auto base*/);
	if (retStr != 0 && retStr != str.c_str()) intVal = ret;

	bool isTrue = str == "y" || str == "Y" || str == "yes" || str == "Yes" || str == "YES" ||
				  str == "true" || str == "True" || str == "TRUE" || str == "on" || str == "ON" ||
				  str == "On" || (intVal.has_value() && intVal.value() != 0);

	return isTrue;
}

void World::parse_tag_marker(const XmlParserContext& ctx)
{
	auto typeAttr = ctx.node->first_attribute("type");
	ASSERTMSG_(typeAttr, "XML tag '<marker />' must have a 'type=\"xxx\"' attribute)");
	const std::string type = typeAttr->value();

	mrpt::img::TColor color{0xff, 0xff, 0xff, 0xff};

	TParameterDefinitions params;
	mrpt::math::TPoint3D translation = {0, 0, 0};
	params["color"] = TParamEntry("%color", &color);
	params["translation"] = TParamEntry("%point3d", &translation);

	// Parse XML params:
	parse_xmlnode_children_as_param(*ctx.node, params, user_defined_variables());

	if (type == "line_strip")
	{
		// line_strip
		// ---------------
		// Walls shape can come from external model file, or from a "shape" entry:
		const auto* xmlPts = ctx.node->first_node("points");
		ASSERTMSG_(xmlPts, "<marker type='line_strip'> requires tag <points>x y z...</points>");

		mrpt::maps::CSimplePointsMap pts;
		std::string parseError;
		std::stringstream ss(mvsim::trim(xmlPts->value()));
		bool parsedOk = pts.load3D_from_text_stream(ss, parseError);
		ASSERTMSG_(
			parsedOk, "Error parsing XYZ data within <marker type='line_strip'>: "s + parseError);

		auto glObj = mrpt::opengl::CSetOfLines::Create();
		glObj->setColor_u8(color);
		glObj->setLocation(translation);

		for (size_t i = 0; i < pts.size(); i++)
		{
			mrpt::math::TPoint3D pt;
			pts.getPoint(i, pt);
			pt += worldRenderOffset();

			if (i == 0)
				glObj->appendLine(pt, pt);
			else
				glObj->appendLineStrip(pt);
		}

		auto lckPhys = mrpt::lockHelper(physical_objects_mtx());
		worldVisual_->insert(glObj);
	}
	else
	{
		THROW_EXCEPTION_FMT("Unknown <marker> of type='%s'>", type.c_str());
	}
}

void World::GeoreferenceOptions::parse_from(
	const rapidxml::xml_node<char>& node, mrpt::system::COutputLogger& logger)
{
	parse_xmlnode_children_as_param(node, params, {}, "[World::GeoreferenceOptions]", &logger);
}
