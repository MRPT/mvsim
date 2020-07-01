/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/poses/CPose2D.h>
#include <mvsim/Block.h>
#include <mvsim/World.h>

#include <map>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>
#include <sstream>  // std::stringstream
#include <string>

#include "JointXMLnode.h"
#include "XMLClassesRegistry.h"
#include "xml_utils.h"

using namespace mvsim;
using namespace std;

static XmlClassesRegistry block_classes_registry("block:class");

// Protected ctor:
Block::Block(World* parent)
	: VisualObject(parent),
	  m_block_index(0),
	  m_mass(30.0),
	  m_block_z_min(0.0),
	  m_block_z_max(1.0),
	  m_block_color(0x00, 0x00, 0xff),
	  m_block_com(.0, .0),
	  m_lateral_friction(0.5),
	  m_ground_friction(0.5),
	  m_restitution(0.01)
{
	// Default shape:
	m_block_poly.emplace_back(-0.5, -0.5);
	m_block_poly.emplace_back(-0.5, 0.5);
	m_block_poly.emplace_back(0.5, 0.5);
	m_block_poly.emplace_back(0.5, -0.5);
	updateMaxRadiusFromPoly();
}

/** Register a new class of vehicles from XML description of type
 * "<vehicle:class name='name'>...</vehicle:class>".  */
void Block::register_block_class(const rapidxml::xml_node<char>* xml_node)
{
	// Sanity checks:
	if (!xml_node)
		throw runtime_error(
			"[Block::register_vehicle_class] XML node is nullptr");
	if (0 != strcmp(xml_node->name(), "block:class"))
		throw runtime_error(mrpt::format(
			"[Block::register_block_class] XML element is '%s' "
			"('block:class' expected)",
			xml_node->name()));

	// rapidxml doesn't allow making copied of objects.
	// So: convert to txt; then re-parse.
	std::stringstream ss;
	ss << *xml_node;

	block_classes_registry.add(ss.str());
}

Block::Ptr Block::factory(World* parent, const rapidxml::xml_node<char>* root)
{
	using namespace std;
	using namespace rapidxml;

	if (!root) throw runtime_error("[Block::factory] XML node is nullptr");
	if (0 != strcmp(root->name(), "block"))
		throw runtime_error(mrpt::format(
			"[Block::factory] XML root element is '%s' ('block' expected)",
			root->name()));

	// "class": When there is a 'class="XXX"' attribute, look for each parameter
	//  in the set of "root" + "class_root" XML nodes:
	// --------------------------------------------------------------------------------
	JointXMLnode<> block_root_node;
	const rapidxml::xml_node<char>* class_root = nullptr;
	{
		block_root_node.add(
			root);  // Always search in root. Also in the class root, if any:
		const xml_attribute<>* block_class = root->first_attribute("class");
		if (block_class)
		{
			const string sClassName = block_class->value();
			class_root = block_classes_registry.get(sClassName);
			if (!class_root)
				throw runtime_error(mrpt::format(
					"[Block::factory] Block class '%s' undefined",
					sClassName.c_str()));
			block_root_node.add(class_root);
		}
	}

	// Build object (we don't use class factory for blocks)
	// ----------------------------------------------------
	Block::Ptr block = Block::Ptr(new Block(parent));

	// Init params
	// -------------------------------------------------
	// attrib: name
	{
		const xml_attribute<>* attrib_name = root->first_attribute("name");
		if (attrib_name && attrib_name->value())
		{
			block->m_name = attrib_name->value();
		}
		else
		{
			// Default name:
			static int cnt = 0;
			block->m_name = mrpt::format("block%i", ++cnt);
		}
	}

	// (Mandatory) initial pose:
	{
		const xml_node<>* node = block_root_node.first_node("init_pose");
		if (!node)
			throw runtime_error(
				"[Block::factory] Missing XML node <init_pose>");

		if (3 != ::sscanf(
					 node->value(), "%lf %lf %lf", &block->m_q.x, &block->m_q.y,
					 &block->m_q.yaw))
			throw runtime_error(
				"[Block::factory] Error parsing <init_pose>...</init_pose>");
		block->m_q.yaw *= M_PI / 180.0;  // deg->rad
	}

	// (Optional) initial vel:
	{
		const xml_node<>* node = block_root_node.first_node("init_vel");
		if (node)
		{
			if (3 != ::sscanf(
						 node->value(), "%lf %lf %lf", &block->m_dq.vx,
						 &block->m_dq.vy, &block->m_dq.omega))
				throw runtime_error(
					"[Block::factory] Error parsing <init_vel>...</init_vel>");
			block->m_dq.omega *= M_PI / 180.0;  // deg->rad

			// Convert twist (velocity) from local -> global coords:
			block->m_dq.rotate(block->m_q.yaw);
		}
	}

	// Custom visualization 3D model:
	// -----------------------------------------------------------
	block->parseVisual(block_root_node.first_node("visual"));

	// Params:
	// -----------------------------------------------------------
	parse_xmlnode_children_as_param(*root, block->m_params, "[Block::factory]");
	if (class_root)
		parse_xmlnode_children_as_param(
			*class_root, block->m_params, "[Block::factory]");

	// Shape node (optional, fallback to default shape if none found)
	const rapidxml::xml_node<char>* xml_shape =
		block_root_node.first_node("shape");
	if (xml_shape)
	{
		mvsim::parse_xmlnode_shape(
			*xml_shape, block->m_block_poly, "[Block::factory]");
		block->updateMaxRadiusFromPoly();
	}

	// Register bodies, fixtures, etc. in Box2D simulator:
	// ----------------------------------------------------
	block->create_multibody_system(*parent->getBox2DWorld());

	if (block->m_b2d_body)
	{
		// Init pos:
		block->m_b2d_body->SetTransform(
			b2Vec2(block->m_q.x, block->m_q.y), block->m_q.yaw);
		// Init vel:
		block->m_b2d_body->SetLinearVelocity(
			b2Vec2(block->m_dq.vx, block->m_dq.vy));
		block->m_b2d_body->SetAngularVelocity(block->m_dq.omega);
	}

	return block;
}

Block::Ptr Block::factory(World* parent, const std::string& xml_text)
{
	// Parse the string as if it was an XML file:
	std::stringstream s;
	s.str(xml_text);

	char* input_str = const_cast<char*>(xml_text.c_str());
	rapidxml::xml_document<> xml;
	try
	{
		xml.parse<0>(input_str);
	}
	catch (rapidxml::parse_error& e)
	{
		unsigned int line =
			static_cast<long>(std::count(input_str, e.where<char>(), '\n') + 1);
		throw std::runtime_error(mrpt::format(
			"[Block::factory] XML parse error (Line %u): %s",
			static_cast<unsigned>(line), e.what()));
	}
	return Block::factory(parent, xml.first_node());
}

void Block::simul_pre_timestep(const TSimulContext& context)
{
	Simulable::simul_pre_timestep(context);
}

/** Override to do any required process right after the integration of dynamic
 * equations for each timestep */
void Block::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);
}

mrpt::poses::CPose3D Block::internalGuiGetVisualPose()
{
	return mrpt::poses::CPose3D(m_q);
}

void Block::internalGuiUpdate(mrpt::opengl::COpenGLScene& scene)
{
	// 1st time call?? -> Create objects
	// ----------------------------------
	if (!m_gl_block)
	{
		m_gl_block = mrpt::opengl::CSetOfObjects::Create();

		// Block shape:
		auto gl_poly = mrpt::opengl::CPolyhedron::CreateCustomPrism(
			m_block_poly, m_block_z_max - m_block_z_min);
		gl_poly->setLocation(0, 0, m_block_z_min);
		gl_poly->setColor_u8(m_block_color);
		m_gl_block->insert(gl_poly);

		scene.insert(m_gl_block);

		// Visualization of forces:
		m_gl_forces = mrpt::opengl::CSetOfLines::Create();
		m_gl_forces->setLineWidth(3.0);
		m_gl_forces->setColor_u8(0xff, 0xff, 0xff);

		scene.insert(m_gl_forces);  // forces are in global coords
	}

	// Update them:
	m_gl_block->setPose(m_q);

	// Other common stuff:
	internal_internalGuiUpdate_forces(scene);
}

void Block::internal_internalGuiUpdate_forces(mrpt::opengl::COpenGLScene& scene)
{
	if (m_world->m_gui_options.show_forces)
	{
		std::lock_guard<std::mutex> csl(m_force_segments_for_rendering_cs);
		m_gl_forces->clear();
		m_gl_forces->appendLines(m_force_segments_for_rendering);
		m_gl_forces->setVisibility(true);
	}
	else
	{
		m_gl_forces->setVisibility(false);
	}
}

void Block::updateMaxRadiusFromPoly()
{
	using namespace mrpt::math;

	m_max_radius = 0.001f;
	for (const auto& segment : m_block_poly)
	{
		const float n = segment.norm();
		mrpt::keep_max(m_max_radius, n);
	}
}

/** Create bodies, fixtures, etc. for the dynamical simulation */
void Block::create_multibody_system(b2World& world)
{
	// Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;

	m_b2d_body = world.CreateBody(&bodyDef);

	// Define shape of block:
	// ------------------------------
	{
		// Convert shape into Box2D format:
		const size_t nPts = m_block_poly.size();
		ASSERT_(nPts >= 3);
		ASSERT_BELOWEQ_(nPts, (size_t)b2_maxPolygonVertices);
		std::vector<b2Vec2> pts(nPts);
		for (size_t i = 0; i < nPts; i++)
			pts[i] = b2Vec2(m_block_poly[i].x, m_block_poly[i].y);

		b2PolygonShape blockPoly;
		blockPoly.Set(&pts[0], nPts);
		// blockPoly.m_radius = 1e-3;  // The "skin" depth of the body

		// Define the dynamic body fixture.
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &blockPoly;
		fixtureDef.restitution = m_restitution;

		// Set the box density to be non-zero, so it will be dynamic.
		b2MassData mass;
		blockPoly.ComputeMass(&mass, 1);  // Mass with density=1 => compute area
		fixtureDef.density = m_mass / mass.mass;

		// Override the default friction.
		fixtureDef.friction = m_lateral_friction;  // 0.3f;

		// Add the shape to the body.
		m_fixture_block = m_b2d_body->CreateFixture(&fixtureDef);

		// Compute center of mass:
		b2MassData vehMass;
		m_fixture_block->GetMassData(&vehMass);
		m_block_com.x = vehMass.center.x;
		m_block_com.y = vehMass.center.y;
	}

	// Create "archor points" to simulate friction with the ground:
	// -----------------------------------------------------------------
	const size_t nContactPoints = 2;
	const double weight_per_contact_point =
		m_mass * getWorldObject()->get_gravity() / nContactPoints;
	const double mu = m_ground_friction;
	const double max_friction = mu * weight_per_contact_point;

	// Location (local coords) of each contact-point:
	const mrpt::math::TPoint2D pt_loc[nContactPoints] = {
		mrpt::math::TPoint2D(m_max_radius, 0),
		mrpt::math::TPoint2D(-m_max_radius, 0)};

	b2FrictionJointDef fjd;

	fjd.bodyA = m_world->getBox2DGroundBody();
	fjd.bodyB = m_b2d_body;

	for (size_t i = 0; i < nContactPoints; i++)
	{
		const b2Vec2 local_pt = b2Vec2(pt_loc[i].x, pt_loc[i].y);

		fjd.localAnchorA = m_b2d_body->GetWorldPoint(local_pt);
		fjd.localAnchorB = local_pt;
		fjd.maxForce = max_friction;
		fjd.maxTorque = 0;

		b2FrictionJoint* b2_friction = dynamic_cast<b2FrictionJoint*>(
			m_world->getBox2DWorld()->CreateJoint(&fjd));
		m_friction_joints.push_back(b2_friction);
	}
}

void Block::apply_force(
	const mrpt::math::TVector2D& force, const mrpt::math::TPoint2D& applyPoint)
{
	ASSERT_(m_b2d_body);
	// Application point -> world coords
	const b2Vec2 wPt =
		m_b2d_body->GetWorldPoint(b2Vec2(applyPoint.x, applyPoint.y));
	m_b2d_body->ApplyForce(b2Vec2(force.x, force.y), wPt, true /*wake up*/);
}
