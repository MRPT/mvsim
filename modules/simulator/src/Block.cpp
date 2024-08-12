/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/bits_math.h>
#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/opengl/CBox.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/version.h>
#include <mvsim/Block.h>
#include <mvsim/World.h>

#include <map>
#include <rapidxml.hpp>
#include <string>

#include "JointXMLnode.h"
#include "XMLClassesRegistry.h"
#include "xml_utils.h"

using namespace mvsim;
using namespace std;

static XmlClassesRegistry block_classes_registry("block:class");

// Protected ctor:
Block::Block(World* parent) : VisualObject(parent), Simulable(parent)
{
	// Default shape:
	block_poly_.emplace_back(-0.5, -0.5);
	block_poly_.emplace_back(-0.5, 0.5);
	block_poly_.emplace_back(0.5, 0.5);
	block_poly_.emplace_back(0.5, -0.5);
	updateMaxRadiusFromPoly();
}

/** Register a new class of vehicles from XML description of type
 * "<vehicle:class name='name'>...</vehicle:class>".  */
void Block::register_block_class(const World& parent, const rapidxml::xml_node<char>* xml_node)
{
	// Sanity checks:
	if (!xml_node) throw runtime_error("[Block::register_vehicle_class] XML node is nullptr");
	if (0 != strcmp(xml_node->name(), "block:class"))
		throw runtime_error(mrpt::format(
			"[Block::register_block_class] XML element is '%s' "
			"('block:class' expected)",
			xml_node->name()));

	// Parse XML to solve for includes:
	block_classes_registry.add(xml_to_str_solving_includes(parent, xml_node));
}

Block::Ptr Block::factory(World* parent, const rapidxml::xml_node<char>* root)
{
	using namespace std;
	using namespace rapidxml;

	if (!root) throw runtime_error("[Block::factory] XML node is nullptr");
	if (0 != strcmp(root->name(), "block"))
		throw runtime_error(mrpt::format(
			"[Block::factory] XML root element is '%s' ('block' expected)", root->name()));

	// "class": When there is a 'class="XXX"' attribute, look for each parameter
	//  in the set of "root" + "class_root" XML nodes:
	// --------------------------------------------------------------------------------
	JointXMLnode<> nodes;
	const rapidxml::xml_node<char>* class_root = nullptr;
	{
		// Always search in root. Also in the class root, if any:
		nodes.add(root);
		if (const xml_attribute<>* block_class = root->first_attribute("class"); block_class)
		{
			const string sClassName = block_class->value();
			if (class_root = block_classes_registry.get(sClassName); !class_root)
				THROW_EXCEPTION_FMT(
					"[Block::factory] Block class '%s' undefined", sClassName.c_str());

			nodes.add(class_root);
		}
	}

	// Build object (we don't use class factory for blocks)
	// ----------------------------------------------------
	Block::Ptr block = std::make_shared<Block>(parent);

	// Init params
	// -------------------------------------------------
	// attrib: name
	{
		const xml_attribute<>* attrib_name = root->first_attribute("name");
		if (attrib_name && attrib_name->value())
		{
			block->name_ = attrib_name->value();
		}
		else
		{
			// Default name:
			static int cnt = 0;
			block->name_ = mrpt::format("block%03i", ++cnt);
		}
	}

	// Common setup for simulable objects:
	// -----------------------------------------------------------
	block->parseSimulable(nodes);

	// Params:
	// -----------------------------------------------------------
	parse_xmlnode_children_as_param(
		*root, block->params_, parent->user_defined_variables(), "[Block::factory]");
	if (class_root)
		parse_xmlnode_children_as_param(
			*class_root, block->params_, parent->user_defined_variables(), "[Block::factory]");

	// Custom visualization 3D model:
	// -----------------------------------------------------------
	block->parseVisual(nodes);

	// Shape node (optional, fallback to default shape if none found)
	if (const auto* xml_shape = nodes.first_node("shape"); xml_shape)
	{
		mvsim::parse_xmlnode_shape(*xml_shape, block->block_poly_, "[Block::factory]");
		block->updateMaxRadiusFromPoly();
	}
	else if (const auto* xml_geom = nodes.first_node("geometry"); xml_geom)
	{
		block->internal_parseGeometry(*xml_geom);
	}

	// Auto shape node from visual?
	if (const rapidxml::xml_node<char>* xml_shape_viz = nodes.first_node("shape_from_visual");
		xml_shape_viz)
	{
		const auto& bbVis = block->collisionShape();
		if (!bbVis.has_value())
		{
			THROW_EXCEPTION(
				"Error: Tag <shape_from_visual/> found but neither <visual> "
				"nor <geometry> entries, while parsing <block>");
		}
		const auto& bb = bbVis.value();

		if (bb.volume() == 0)
		{
			THROW_EXCEPTION(
				"Error: Tag <shape_from_visual/> found but bounding box of "
				"visual object seems incorrect, while parsing <block>");
		}

		// Set contour polygon:
		block->block_poly_ = bb.getContour();
		block->block_z_min(bb.zMin());
		block->block_z_max(bb.zMax());
	}
	else
	{
		// set zmin/zmax to defaults, if not set explicitly by the user in the
		// XML file, and we didn't have a "shaped_from_visual" tag:
		if (block->default_block_z_min_max())
		{
			block->block_z_min(0.0);
			block->block_z_max(1.0);
		}
	}

	block->updateMaxRadiusFromPoly();

	// Register bodies, fixtures, etc. in Box2D simulator:
	// ----------------------------------------------------
	block->create_multibody_system(*parent->getBox2DWorld());

	if (block->b2dBody_)
	{
		// Init pos:
		const auto q = block->getPose();
		const auto dq = block->getTwist();

		block->b2dBody_->SetTransform(b2Vec2(q.x, q.y), q.yaw);
		// Init vel:
		block->b2dBody_->SetLinearVelocity(b2Vec2(dq.vx, dq.vy));
		block->b2dBody_->SetAngularVelocity(dq.omega);
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
		unsigned int line = static_cast<long>(std::count(input_str, e.where<char>(), '\n') + 1);
		throw std::runtime_error(mrpt::format(
			"[Block::factory] XML parse error (Line %u): %s", static_cast<unsigned>(line),
			e.what()));
	}
	return Block::factory(parent, xml.first_node());
}

void Block::simul_pre_timestep(const TSimulContext& context)
{
	Simulable::simul_pre_timestep(context);
}

/** Override to do any required process right after the integration of
 * dynamic equations for each timestep */
void Block::simul_post_timestep(const TSimulContext& context)
{
	Simulable::simul_post_timestep(context);
}

void Block::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly)
{
	// 1st time call?? -> Create objects
	// ----------------------------------
	if (!childrenOnly)
	{
		if (!gl_block_ && viz && physical)
		{
			gl_block_ = mrpt::opengl::CSetOfObjects::Create();
			gl_block_->setName(name_);

			// Block shape:
			auto gl_poly = mrpt::opengl::CPolyhedron::CreateCustomPrism(
				block_poly_, block_z_max_ - block_z_min_);
			gl_poly->setLocation(0, 0, block_z_min_);
			gl_poly->setColor_u8(block_color_);

// Hide back faces:
#if MRPT_VERSION >= 0x240
			// gl_poly->cullFaces(mrpt::opengl::TCullFace::FRONT);
#endif

			gl_block_->insert(gl_poly);

			viz->get().insert(gl_block_);
			physical->get().insert(gl_block_);
		}

		// Update them:
		// If "viz" does not have a value, it's because we are already
		// inside a setPose() change event, so my caller already holds the
		// mutex and we don't need/can't acquire it again:
		const auto objectPose = viz.has_value() ? getPose() : getPoseNoLock();

		if (gl_block_) gl_block_->setPose(objectPose);
	}

	if (!gl_forces_ && viz)
	{
		// Visualization of forces:
		gl_forces_ = mrpt::opengl::CSetOfLines::Create();
		gl_forces_->setLineWidth(3.0);
		gl_forces_->setColor_u8(0xff, 0xff, 0xff);

		viz->get().insert(gl_forces_);	// forces are in global coords
	}

	// Other common stuff:
	if (viz) internal_internalGuiUpdate_forces(viz->get());
}

void Block::internal_internalGuiUpdate_forces(	//
	[[maybe_unused]] mrpt::opengl::COpenGLScene& scene)
{
	if (world_->guiOptions_.show_forces)
	{
		std::lock_guard<std::mutex> csl(force_segments_for_rendering_cs_);
		gl_forces_->clear();
		gl_forces_->appendLines(force_segments_for_rendering_);
		gl_forces_->setVisibility(true);
	}
	else
	{
		gl_forces_->setVisibility(false);
	}
}

void Block::updateMaxRadiusFromPoly()
{
	using namespace mrpt::math;

	maxRadius_ = 0.001f;
	for (const auto& segment : block_poly_)
	{
		const float n = segment.norm();
		mrpt::keep_max(maxRadius_, n);
	}
}

/** Create bodies, fixtures, etc. for the dynamical simulation */
void Block::create_multibody_system(b2World& world)
{
	if (intangible_) return;

	// Update collision shape from shape loaded from XML or set manually:
	{
		Shape2p5 cs;
		cs.setShapeManual(block_poly_, block_z_min_, block_z_max_);
		setCollisionShape(cs);
	}

	// Define the dynamic body. We set its position and call the body
	// factory.
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;

	b2dBody_ = world.CreateBody(&bodyDef);

	// Define shape of block:
	// ------------------------------
	{
		// Convert shape into Box2D format:
		const size_t nPts = block_poly_.size();
		ASSERT_(nPts >= 3);
		ASSERT_LE_(nPts, (size_t)b2_maxPolygonVertices);
		std::vector<b2Vec2> pts(nPts);
		for (size_t i = 0; i < nPts; i++) pts[i] = b2Vec2(block_poly_[i].x, block_poly_[i].y);

		b2PolygonShape blockPoly;
		blockPoly.Set(&pts[0], nPts);

		// FIXED value by design in b2Box: The "skin" depth of the body
		blockPoly.m_radius = 2.5e-3;  // b2_polygonRadius;

		// Define the dynamic body fixture.
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &blockPoly;
		fixtureDef.restitution = restitution_;

		// Set the box density to be non-zero, so it will be dynamic.
		b2MassData mass;
		blockPoly.ComputeMass(&mass, 1);  // Mass with density=1 => compute area
		fixtureDef.density = mass_ / mass.mass;

		// Override the default friction.
		fixtureDef.friction = lateral_friction_;  // 0.3f;

		// Add the shape to the body.
		fixture_block_ = b2dBody_->CreateFixture(&fixtureDef);

		// Static (does not move at all) vs dynamic object:
		b2dBody_->SetType(isStatic_ ? b2_staticBody : b2_dynamicBody);

		// Compute center of mass:
		b2MassData vehMass;
		fixture_block_->GetMassData(&vehMass);
		block_com_.x = vehMass.center.x;
		block_com_.y = vehMass.center.y;
	}

	// Create "archor points" to simulate friction with the ground:
	// -----------------------------------------------------------------
	const size_t nContactPoints = 2;
	const double weight_per_contact_point = mass_ * parent()->get_gravity() / nContactPoints;
	const double mu = groundFriction_;
	const double max_friction = mu * weight_per_contact_point;

	// Location (local coords) of each contact-point:
	const mrpt::math::TPoint2D pt_loc[nContactPoints] = {
		mrpt::math::TPoint2D(maxRadius_, 0), mrpt::math::TPoint2D(-maxRadius_, 0)};

	b2FrictionJointDef fjd;

	fjd.bodyA = world_->getBox2DGroundBody();
	fjd.bodyB = b2dBody_;

	for (size_t i = 0; i < nContactPoints; i++)
	{
		const b2Vec2 local_pt = b2Vec2(pt_loc[i].x, pt_loc[i].y);

		fjd.localAnchorA = b2dBody_->GetWorldPoint(local_pt);
		fjd.localAnchorB = local_pt;
		fjd.maxForce = max_friction;
		fjd.maxTorque = 0;

		b2FrictionJoint* b2_friction =
			dynamic_cast<b2FrictionJoint*>(world_->getBox2DWorld()->CreateJoint(&fjd));
		friction_joints_.push_back(b2_friction);
	}
}

void Block::apply_force(const mrpt::math::TVector2D& force, const mrpt::math::TPoint2D& applyPoint)
{
	if (intangible_) return;
	ASSERT_(b2dBody_);
	// Application point -> world coords
	const b2Vec2 wPt = b2dBody_->GetWorldPoint(b2Vec2(applyPoint.x, applyPoint.y));
	b2dBody_->ApplyForce(b2Vec2(force.x, force.y), wPt, true /*wake up*/);
}

bool Block::isStatic() const
{
	if (intangible_) return true;
	ASSERT_(b2dBody_);
	return b2dBody_->GetType() == b2_staticBody;
}

void Block::setIsStatic(bool b)
{
	if (intangible_) return;
	ASSERT_(b2dBody_);
	b2dBody_->SetType(b ? b2_staticBody : b2_dynamicBody);
}

// Protected ctor:
DummyInvisibleBlock::DummyInvisibleBlock(World* parent) : VisualObject(parent), Simulable(parent) {}

void DummyInvisibleBlock::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	[[maybe_unused]] bool childrenOnly)
{
	if (!viz || !physical) return;

	for (auto& s : sensors_) s->guiUpdate(viz, physical);
}

void Block::internal_parseGeometry(const rapidxml::xml_node<char>& xml_geom_node)
{
	std::string type;  // cylinder, sphere, etc.
	float radius = 0;
	float length = 0, lx = 0, ly = 0, lz = 0;
	int vertex_count = 0;

	const TParameterDefinitions params = {
		{"type", {"%s", &type}},
		{"radius", {"%f", &radius}},
		{"length", {"%f", &length}},
		{"lx", {"%f", &lx}},
		{"ly", {"%f", &ly}},
		{"lz", {"%f", &lz}},
		{"vertex_count", {"%i", &vertex_count}},
	};

	parse_xmlnode_attribs(
		xml_geom_node, params, world_->user_defined_variables(), "[Block::internal_parseGeometry]");

	if (type.empty())
	{
		THROW_EXCEPTION(
			"Geometry type attribute is missing, i.e. <geometry type='...' ... "
			"/>");
	}

	if (type == "cylinder")
	{
		ASSERTMSG_(radius > 0, "Missing 'radius' attribute for cylinder geometry");
		ASSERTMSG_(length > 0, "Missing 'length' attribute for cylinder geometry");

		if (vertex_count == 0) vertex_count = 10;  // default

		auto glCyl = mrpt::opengl::CCylinder::Create();
		glCyl->setHeight(length);
		glCyl->setRadius(radius);
		glCyl->setSlicesCount(vertex_count);
		glCyl->setColor_u8(block_color_);
		addCustomVisualization(glCyl);
	}
	else if (type == "sphere")
	{
		ASSERTMSG_(radius > 0, "Missing 'radius' attribute for cylinder geometry");

		if (vertex_count == 0) vertex_count = 10;  // default

		auto glSph = mrpt::opengl::CSphere::Create(radius, vertex_count);
		glSph->setColor_u8(block_color_);
		addCustomVisualization(glSph);
	}
	else if (type == "box")
	{
		ASSERTMSG_(lx > 0, "Missing 'lx' attribute for box geometry");
		ASSERTMSG_(ly > 0, "Missing 'ly' attribute for box geometry");
		ASSERTMSG_(lz > 0, "Missing 'lz' attribute for box geometry");

		auto glBox = mrpt::opengl::CBox::Create();
		glBox->setBoxCorners({0, 0, 0}, {lx, ly, lz});
		glBox->setColor_u8(block_color_);
		addCustomVisualization(glBox);
	}
	else
	{
		THROW_EXCEPTION_FMT("Unknown type in <geometry type='%s'...>", type.c_str());
	}
}

bool Block::default_block_z_min_max() const
{
	// true if any of the limits is a nan:
	return block_z_max_ != block_z_max_ || block_z_min_ != block_z_min_;
}
