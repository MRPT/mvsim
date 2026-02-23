/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#include <box2d/b2_distance_joint.h>
#include <box2d/b2_revolute_joint.h>
#include <mrpt/core/format.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/system/filesystem.h>	 // extractFileDirectory()
#include <mrpt/topography/conversions.h>
#include <mvsim/World.h>

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
	if (fileDir.empty())
	{
		fileDir = ".";
	}

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
	{
		throw runtime_error(
			mrpt::format("XML root element is '%s' ('mvsim_world' expected)", root->name()));
	}

	// Optional: format version attrib:
	const xml_attribute<>* attrb_version = root->first_attribute("version");
	int version_major = 1, version_min = 0;
	if (attrb_version)
	{
		int ret = sscanf(attrb_version->value(), "%i.%i", &version_major, &version_min);
		if (ret != 2)
		{
			throw runtime_error(mrpt::format(
				"Error parsing version attribute: '%s' ('%%i.%%i' "
				"expected)",
				attrb_version->value()));
		}
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
	if (!xmlParsers_.empty())
	{
		return;
	}
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

	register_tag_parser("joint", &World::parse_tag_joint);

	register_tag_parser("actor", &World::parse_tag_actor);
	register_tag_parser("actor:class", &World::parse_tag_actor_class);
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

void World::insert_vehicle(const VehicleBase::Ptr& veh)
{
	auto lck = mrpt::lockHelper(world_cs_);

	// Assign each vehicle a unique "index" number
	veh->setVehicleIndex(vehicles_.size());

	ASSERTMSG_(
		vehicles_.count(veh->getName()) == 0,
		mrpt::format("Duplicated vehicle name: '%s'", veh->getName().c_str()));

	vehicles_.insert(VehicleList::value_type(veh->getName(), veh));

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());
	simulableObjects_.emplace(veh->getName(), std::dynamic_pointer_cast<Simulable>(veh));
}

void World::parse_tag_vehicle(const XmlParserContext& ctx)
{
	// <vehicle> entries:
	VehicleBase::Ptr veh = VehicleBase::factory(this, ctx.node);
	insert_vehicle(veh);
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
		if (strcmp(attr->name(), "file") == 0)
		{
			continue;
		}
		vars[attr->name()] = attr->value();
	}

	const auto [xml, root] = readXmlAndGetRoot(absFile, vars);
	(void)xml;	// unused

	// recursive parse all root-level nodes
	// (supports included files with multiple sibling elements):
	const auto newBasePath = mrpt::system::extractFileDirectory(absFile);
	for (auto* n = root; n; n = n->next_sibling())
	{
		if (n->type() != rapidxml::node_element)
		{
			continue;
		}
		internal_recursive_parse_XML({n, newBasePath});
	}
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
	if (!isTrue)
	{
		return;
	}
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
	if (retStr != 0 && retStr != str.c_str())
	{
		intVal = ret;
	}

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
			{
				glObj->appendLine(pt, pt);
			}
			else
			{
				glObj->appendLineStrip(pt);
			}
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

void World::parse_tag_joint(const XmlParserContext& ctx)
{
	using namespace rapidxml;
	using namespace std::string_literals;

	const xml_node<char>* node = ctx.node;
	ASSERT_(node);

	// ---- Parse required attributes ----
	const auto* typeAttr = node->first_attribute("type");
	ASSERTMSG_(
		typeAttr,
		"XML tag '<joint />' must have a 'type' attribute "
		"(\"distance\" or \"revolute\")");
	const std::string typeStr = typeAttr->value();

	const auto* bodyAAttr = node->first_attribute("body_a");
	ASSERTMSG_(bodyAAttr, "XML tag '<joint />' must have a 'body_a' attribute");
	const auto* bodyBAttr = node->first_attribute("body_b");
	ASSERTMSG_(bodyBAttr, "XML tag '<joint />' must have a 'body_b' attribute");

	const auto* anchorAAttr = node->first_attribute("anchor_a");
	ASSERTMSG_(
		anchorAAttr,
		"XML tag '<joint />' must have an 'anchor_a' attribute "
		"(\"x y\" in body-local coords)");
	const auto* anchorBAttr = node->first_attribute("anchor_b");
	ASSERTMSG_(
		anchorBAttr,
		"XML tag '<joint />' must have an 'anchor_b' attribute "
		"(\"x y\" in body-local coords)");

	// ---- Build descriptor ----
	WorldJoint jd;

	// Type
	if (typeStr == "distance" || typeStr == "rope")
	{
		jd.type = WorldJoint::Type::Distance;
	}
	else if (typeStr == "revolute" || typeStr == "pin")
	{
		jd.type = WorldJoint::Type::Revolute;
	}
	else
	{
		THROW_EXCEPTION_FMT(
			"Unknown <joint> type='%s'. "
			"Expected 'distance' (or 'rope') or 'revolute' (or 'pin').",
			typeStr.c_str());
	}

	// Body names (resolve variables)
	jd.bodyA_name = mvsim::parse(bodyAAttr->value(), user_defined_variables());
	jd.bodyB_name = mvsim::parse(bodyBAttr->value(), user_defined_variables());

	// Anchor points: "x y"
	{
		const std::string sA = mvsim::parse(anchorAAttr->value(), user_defined_variables());
		double ax = 0;
		double ay = 0;
		if (sscanf(sA.c_str(), "%lf %lf", &ax, &ay) != 2)
		{
			THROW_EXCEPTION_FMT("Malformed anchor_a='%s'. Expected \"x y\".", sA.c_str());
		}
		jd.anchorA = {ax, ay};
	}
	{
		const std::string sB = mvsim::parse(anchorBAttr->value(), user_defined_variables());
		double bx = 0;
		double by = 0;
		if (sscanf(sB.c_str(), "%lf %lf", &bx, &by) != 2)
		{
			THROW_EXCEPTION_FMT("Malformed anchor_b='%s'. Expected \"x y\".", sB.c_str());
		}
		jd.anchorB = {bx, by};
	}

	// ---- Optional attributes ----
	{
		TParameterDefinitions attribs;
		attribs["max_length"] = TParamEntry("%f", &jd.maxLength);
		attribs["min_length"] = TParamEntry("%f", &jd.minLength);
		attribs["stiffness"] = TParamEntry("%f", &jd.stiffness);
		attribs["damping"] = TParamEntry("%f", &jd.damping);
		attribs["enable_limit"] = TParamEntry("%bool", &jd.enableLimit);
		attribs["lower_angle_deg"] = TParamEntry("%f", &jd.lowerAngle_deg);
		attribs["upper_angle_deg"] = TParamEntry("%f", &jd.upperAngle_deg);

		parse_xmlnode_attribs(*node, attribs, user_defined_variables(), "[World::parse_tag_joint]");
	}

	// ---- Look up the two Simulable bodies ----
	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());

	auto itA = simulableObjects_.find(jd.bodyA_name);
	if (itA == simulableObjects_.end())
	{
		THROW_EXCEPTION_FMT(
			"<joint> body_a='%s' not found among simulable objects. "
			"Make sure the <vehicle> or <block> is defined BEFORE the "
			"<joint> tag.",
			jd.bodyA_name.c_str());
	}
	auto itB = simulableObjects_.find(jd.bodyB_name);
	if (itB == simulableObjects_.end())
	{
		THROW_EXCEPTION_FMT(
			"<joint> body_b='%s' not found among simulable objects. "
			"Make sure the <vehicle> or <block> is defined BEFORE the "
			"<joint> tag.",
			jd.bodyB_name.c_str());
	}

	b2Body* b2bodyA = itA->second->b2d_body();
	b2Body* b2bodyB = itB->second->b2d_body();
	ASSERTMSG_(
		b2bodyA, mrpt::format(
					 "<joint> body_a='%s' has no Box2D body. "
					 "Is it a valid physical object?",
					 jd.bodyA_name.c_str()));
	ASSERTMSG_(
		b2bodyB, mrpt::format(
					 "<joint> body_b='%s' has no Box2D body. "
					 "Is it a valid physical object?",
					 jd.bodyB_name.c_str()));

	// ---- Create the Box2D joint ----
	ASSERT_(box2d_world_);

	switch (jd.type)
	{
		case WorldJoint::Type::Distance:
		{
			b2DistanceJointDef djd;
			djd.bodyA = b2bodyA;
			djd.bodyB = b2bodyB;
			djd.localAnchorA.Set(
				static_cast<float>(jd.anchorA.x), static_cast<float>(jd.anchorA.y));
			djd.localAnchorB.Set(
				static_cast<float>(jd.anchorB.x), static_cast<float>(jd.anchorB.y));
			djd.minLength = jd.minLength;
			djd.maxLength = jd.maxLength;
			// rest length = max for rope behavior
			djd.length = jd.maxLength;
			djd.stiffness = jd.stiffness;
			djd.damping = jd.damping;
			djd.collideConnected = false;

			jd.b2joint = box2d_world_->CreateJoint(&djd);

			MRPT_LOG_INFO_FMT(
				"Created distance joint between '%s' and '%s' "
				"(maxLength=%.3f)",
				jd.bodyA_name.c_str(), jd.bodyB_name.c_str(), jd.maxLength);
			break;
		}
		case WorldJoint::Type::Revolute:
		{
			b2RevoluteJointDef rjd;
			rjd.bodyA = b2bodyA;
			rjd.bodyB = b2bodyB;
			rjd.localAnchorA.Set(
				static_cast<float>(jd.anchorA.x), static_cast<float>(jd.anchorA.y));
			rjd.localAnchorB.Set(
				static_cast<float>(jd.anchorB.x), static_cast<float>(jd.anchorB.y));

			// Compute reference angle from current body angles:
			rjd.referenceAngle = b2bodyB->GetAngle() - b2bodyA->GetAngle();

			if (jd.enableLimit)
			{
				rjd.enableLimit = true;
				rjd.lowerAngle =
					static_cast<float>(static_cast<double>(jd.lowerAngle_deg) * M_PI / 180.0);
				rjd.upperAngle =
					static_cast<float>(static_cast<double>(jd.upperAngle_deg) * M_PI / 180.0);
			}

			rjd.collideConnected = false;

			jd.b2joint = box2d_world_->CreateJoint(&rjd);

			MRPT_LOG_INFO_FMT(
				"Created revolute joint between '%s' and '%s'", jd.bodyA_name.c_str(),
				jd.bodyB_name.c_str());
			break;
		}
		default:
			THROW_EXCEPTION("Unhandled joint type");
	}

	joints_.emplace_back(std::move(jd));
}

void World::parse_tag_actor(const XmlParserContext& ctx)
{
	HumanActor::Ptr actor = HumanActor::factory(this, ctx.node);

	ASSERTMSG_(
		actors_.count(actor->getName()) == 0,
		mrpt::format("Duplicated actor name: '%s'", actor->getName().c_str()));

	actors_.insert(ActorList::value_type(actor->getName(), actor));

	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());
	simulableObjects_.emplace(actor->getName(), std::dynamic_pointer_cast<Simulable>(actor));
}

void World::parse_tag_actor_class(const XmlParserContext& ctx)
{
	HumanActor::register_actor_class(*this, ctx.node);
}
