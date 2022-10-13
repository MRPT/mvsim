/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/FrictionModels/DefaultFriction.h>  // For use as default model
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/World.h>

#include <map>
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>
#include <rapidxml_utils.hpp>
#include <sstream>	// std::stringstream
#include <string>

#include "JointXMLnode.h"
#include "XMLClassesRegistry.h"
#include "parse_utils.h"
#include "xml_utils.h"

using namespace mvsim;
using namespace std;

static XmlClassesRegistry veh_classes_registry("vehicle:class");

TClassFactory_vehicleDynamics mvsim::classFactory_vehicleDynamics;

// Explicit registration calls seem to be one (the unique?) way to assure
// registration takes place:
void register_all_veh_dynamics()
{
	static bool done = false;
	if (done)
		return;
	else
		done = true;

	REGISTER_VEHICLE_DYNAMICS("differential", DynamicsDifferential)
	REGISTER_VEHICLE_DYNAMICS(
		"differential_3_wheels", DynamicsDifferential_3_wheels)
	REGISTER_VEHICLE_DYNAMICS(
		"differential_4_wheels", DynamicsDifferential_4_wheels)
	REGISTER_VEHICLE_DYNAMICS("ackermann", DynamicsAckermann)
	REGISTER_VEHICLE_DYNAMICS(
		"ackermann_drivetrain", DynamicsAckermannDrivetrain)
}

constexpr char VehicleBase::DL_TIMESTAMP[];
constexpr char VehicleBase::LOGGER_POSE[];
constexpr char VehicleBase::LOGGER_WHEEL[];

constexpr char VehicleBase::PL_Q_X[];
constexpr char VehicleBase::PL_Q_Y[];
constexpr char VehicleBase::PL_Q_Z[];
constexpr char VehicleBase::PL_Q_YAW[];
constexpr char VehicleBase::PL_Q_PITCH[];
constexpr char VehicleBase::PL_Q_ROLL[];
constexpr char VehicleBase::PL_DQ_X[];
constexpr char VehicleBase::PL_DQ_Y[];
constexpr char VehicleBase::PL_DQ_Z[];

constexpr char VehicleBase::WL_TORQUE[];
constexpr char VehicleBase::WL_WEIGHT[];
constexpr char VehicleBase::WL_VEL_X[];
constexpr char VehicleBase::WL_VEL_Y[];
constexpr char VehicleBase::WL_FRIC_X[];
constexpr char VehicleBase::WL_FRIC_Y[];

// Protected ctor:
VehicleBase::VehicleBase(World* parent, size_t nWheels)
	: VisualObject(parent),
	  Simulable(parent),
	  m_fixture_wheels(nWheels, nullptr)
{
	// Create wheels:
	for (size_t i = 0; i < nWheels; i++) m_wheels_info.emplace_back(parent);

	// Default shape:
	m_chassis_poly.emplace_back(-0.4, -0.5);
	m_chassis_poly.emplace_back(-0.4, 0.5);
	m_chassis_poly.emplace_back(0.4, 0.5);
	m_chassis_poly.emplace_back(0.6, 0.3);
	m_chassis_poly.emplace_back(0.6, -0.3);
	m_chassis_poly.emplace_back(0.4, -0.5);
	updateMaxRadiusFromPoly();
}

/** Register a new class of vehicles from XML description of type
 * "<vehicle:class name='name'>...</vehicle:class>".  */
void VehicleBase::register_vehicle_class(
	const rapidxml::xml_node<char>* xml_node)
{
	// Sanity checks:
	if (!xml_node)
		throw runtime_error(
			"[VehicleBase::register_vehicle_class] XML node is nullptr");
	if (0 != strcmp(xml_node->name(), "vehicle:class"))
		throw runtime_error(mrpt::format(
			"[VehicleBase::register_vehicle_class] XML element is '%s' "
			"('vehicle:class' expected)",
			xml_node->name()));

	// rapidxml doesn't allow making copied of objects.
	// So: convert to txt; then re-parse.
	std::stringstream ss;
	ss << *xml_node;

	veh_classes_registry.add(ss.str());
}

/** Class factory: Creates a vehicle from XML description of type
 * "<vehicle>...</vehicle>".  */
VehicleBase::Ptr VehicleBase::factory(
	World* parent, const rapidxml::xml_node<char>* root)
{
	register_all_veh_dynamics();

	using namespace std;
	using namespace rapidxml;

	if (!root)
		throw runtime_error("[VehicleBase::factory] XML node is nullptr");
	if (0 != strcmp(root->name(), "vehicle"))
		throw runtime_error(mrpt::format(
			"[VehicleBase::factory] XML root element is '%s' ('vehicle' "
			"expected)",
			root->name()));

	// "class": When a vehicle has a 'class="XXX"' attribute, look for each
	// parameter
	//  in the set of "root" + "class_root" XML nodes:
	// --------------------------------------------------------------------------------
	JointXMLnode<> veh_root_node;

	std::vector<XML_Doc_Data::Ptr> scopedLifeDocs;

	// Solve includes:
	for (auto n = root->first_node(); n; n = n->next_sibling())
	{
		if (strcmp(n->name(), "include") != 0) continue;

		auto fileAttrb = n->first_attribute("file");
		ASSERTMSG_(
			fileAttrb,
			"XML tag '<include />' must have a 'file=\"xxx\"' attribute)");

		const std::string relFile = mvsim::parse(fileAttrb->value(), {});
		const auto absFile = parent->resolvePath(relFile);
		parent->logStr(
			mrpt::system::LVL_DEBUG,
			mrpt::format("XML parser: including file: '%s'", absFile.c_str()));

		std::map<std::string, std::string> vars;
		for (auto attr = n->first_attribute(); attr;
			 attr = attr->next_attribute())
		{
			if (strcmp(attr->name(), "file") == 0) continue;
			vars[attr->name()] = attr->value();
		}

		// Delay the replacement of this variable (used in Sensors) until "veh"
		// is constructed and we actually have a name:
		std::set<std::string> varsRetain = {
			"NAME" /*sensor name*/, "PARENT_NAME" /*vehicle name*/};

		const auto [xml, nRoot] = readXmlAndGetRoot(absFile, vars, varsRetain);

		// the XML document object must exist during this whole function scope
		scopedLifeDocs.emplace_back(xml);

		// recursive parse:
		const auto newBasePath =
			mrpt::system::trim(mrpt::system::extractFileDirectory(absFile));

		veh_root_node.add(nRoot->parent());
	}

	// ---
	{
		// Always search in root. Also in the class root, if any:
		veh_root_node.add(root);

		const xml_attribute<>* veh_class = root->first_attribute("class");
		if (veh_class)
		{
			const string sClassName = veh_class->value();
			const rapidxml::xml_node<char>* class_root =
				veh_classes_registry.get(sClassName);
			if (!class_root)
				throw runtime_error(mrpt::format(
					"[VehicleBase::factory] Vehicle class '%s' undefined",
					sClassName.c_str()));

			veh_root_node.add(class_root);
			// cout << *class_root;
		}
	}

	// Class factory according to: <dynamics class="XXX">
	// -------------------------------------------------
	const xml_node<>* dyn_node = veh_root_node.first_node("dynamics");
	if (!dyn_node)
		throw runtime_error(
			"[VehicleBase::factory] Missing XML node <dynamics>");

	const xml_attribute<>* dyn_class = dyn_node->first_attribute("class");
	if (!dyn_class || !dyn_class->value())
		throw runtime_error(
			"[VehicleBase::factory] Missing mandatory attribute 'class' in "
			"node <dynamics>");

	VehicleBase::Ptr veh =
		classFactory_vehicleDynamics.create(dyn_class->value(), parent);
	if (!veh)
		throw runtime_error(mrpt::format(
			"[VehicleBase::factory] Unknown vehicle dynamics class '%s'",
			dyn_class->value()));

	// Initialize here all common params shared by any polymorphic class:
	// -------------------------------------------------
	// attrib: name
	{
		const xml_attribute<>* attrib_name = root->first_attribute("name");
		if (attrib_name && attrib_name->value())
		{
			veh->m_name = attrib_name->value();
		}
		else
		{
			// Default name:
			static int cnt = 0;
			veh->m_name = mrpt::format("veh%i", ++cnt);
		}
	}

	// Common setup for simulable objects:
	// -----------------------------------------------------------
	veh->parseSimulable(veh_root_node);

	// Custom visualization 3D model:
	// -----------------------------------------------------------
	veh->parseVisual(veh_root_node.first_node("visual"));

	// Initialize class-specific params (mass, chassis shape, etc.)
	// ---------------------------------------------------------------
	veh->dynamics_load_params_from_xml(dyn_node);

	// Auto shape node from visual?
	if (const rapidxml::xml_node<char>* xml_chassis =
			dyn_node->first_node("chassis");
		xml_chassis)
	{
		if (const rapidxml::xml_node<char>* sfv =
				xml_chassis->first_node("shape_from_visual");
			sfv)
		{
			mrpt::math::TPoint3D bbmin, bbmax;
			veh->getVisualModelBoundingBox(bbmin, bbmax);
			if (bbmin == bbmax)
			{
				THROW_EXCEPTION(
					"Error: Tag <shape_from_visual/> found but bounding box of "
					"visual object seems incorrect.");
			}

			auto& poly = veh->m_chassis_poly;
			poly.clear();
			poly.emplace_back(bbmin.x, bbmin.y);
			poly.emplace_back(bbmin.x, bbmax.y);
			poly.emplace_back(bbmax.x, bbmax.y);
			poly.emplace_back(bbmax.x, bbmin.y);
		}
	}
	veh->updateMaxRadiusFromPoly();

	// <Optional> Log path. If not specified, app folder will be used
	// -----------------------------------------------------------
	{
		const xml_node<>* log_path_node = veh_root_node.first_node("log_path");
		if (log_path_node)
		{
			// Parse:
			veh->m_log_path = log_path_node->value();
		}
	}

	veh->initLoggers();

	// Register bodies, fixtures, etc. in Box2D simulator:
	// ----------------------------------------------------
	veh->create_multibody_system(*parent->getBox2DWorld());

	if (veh->m_b2d_body)
	{
		// Init pos:
		const auto q = veh->getPose();
		const auto dq = veh->getTwist();

		veh->m_b2d_body->SetTransform(b2Vec2(q.x, q.y), q.yaw);
		// Init vel:
		veh->m_b2d_body->SetLinearVelocity(b2Vec2(dq.vx, dq.vy));
		veh->m_b2d_body->SetAngularVelocity(dq.omega);
	}

	// Friction model:
	// Parse <friction> node, or assume default linear model:
	// -----------------------------------------------------------
	{
		const xml_node<>* frict_node = veh_root_node.first_node("friction");
		if (!frict_node)
		{
			// Default:
			veh->m_friction = std::shared_ptr<FrictionBase>(
				new DefaultFriction(*veh, nullptr /*default params*/));
		}
		else
		{
			// Parse:
			veh->m_friction = std::shared_ptr<FrictionBase>(
				FrictionBase::factory(*veh, frict_node));
			ASSERT_(veh->m_friction);
		}
	}

	// Sensors: <sensor class='XXX'> entries
	// -------------------------------------------------
	for (const auto& xmlNode : veh_root_node)
	{
		if (!strcmp(xmlNode->name(), "sensor"))
		{
			SensorBase::Ptr se = SensorBase::factory(*veh, xmlNode);
			veh->m_sensors.push_back(SensorBase::Ptr(se));
		}
	}

	return veh;
}

VehicleBase::Ptr VehicleBase::factory(
	World* parent, const std::string& xml_text)
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
			"[VehicleBase::factory] XML parse error (Line %u): %s",
			static_cast<unsigned>(line), e.what()));
	}
	return VehicleBase::factory(parent, xml.first_node());
}

void VehicleBase::simul_pre_timestep(const TSimulContext& context)
{
	Simulable::simul_pre_timestep(context);
	for (auto& s : m_sensors) s->simul_pre_timestep(context);

	// Update wheels position (they may turn, etc. as in an Ackermann
	// configuration)
	for (size_t i = 0; i < m_fixture_wheels.size(); i++)
	{
		b2PolygonShape* wheelShape =
			dynamic_cast<b2PolygonShape*>(m_fixture_wheels[i]->GetShape());
		wheelShape->SetAsBox(
			m_wheels_info[i].diameter * 0.5, m_wheels_info[i].width * 0.5,
			b2Vec2(m_wheels_info[i].x, m_wheels_info[i].y),
			m_wheels_info[i].yaw);
	}

	// Apply motor forces/torques:
	this->invoke_motor_controllers(context, m_torque_per_wheel);

	// Apply friction model at each wheel:
	const size_t nW = getNumWheels();
	ASSERT_EQUAL_(m_torque_per_wheel.size(), nW);

	const double gravity = getWorldObject()->get_gravity();
	const double massPerWheel =
		getChassisMass() / nW;	// Part of the vehicle weight on each wheel.
	const double weightPerWheel = massPerWheel * gravity;

	std::vector<mrpt::math::TPoint2D> wheels_vels;
	getWheelsVelocityLocal(wheels_vels, getVelocityLocal());

	ASSERT_EQUAL_(wheels_vels.size(), nW);

	std::vector<mrpt::math::TSegment3D>
		force_vectors;	// For visualization only

	for (size_t i = 0; i < nW; i++)
	{
		// prepare data:
		Wheel& w = getWheelInfo(i);

		FrictionBase::TFrictionInput fi(context, w);
		fi.motor_torque =
			-m_torque_per_wheel[i];	 // "-" => Forwards is negative
		fi.weight = weightPerWheel;
		fi.wheel_speed = wheels_vels[i];

		m_friction->setLogger(
			getLoggerPtr(LOGGER_WHEEL + std::to_string(i + 1)));
		// eval friction:
		mrpt::math::TPoint2D net_force_;
		m_friction->evaluate_friction(fi, net_force_);

		// Apply force:
		const b2Vec2 wForce = m_b2d_body->GetWorldVector(b2Vec2(
			net_force_.x, net_force_.y));  // Force vector -> world coords
		const b2Vec2 wPt = m_b2d_body->GetWorldPoint(
			b2Vec2(w.x, w.y));	// Application point -> world coords
		// printf("w%i: Lx=%6.3f Ly=%6.3f  | Gx=%11.9f
		// Gy=%11.9f\n",(int)i,net_force_.x,net_force_.y,wForce.x,wForce.y);

		m_b2d_body->ApplyForce(wForce, wPt, true /*wake up*/);

		// log
		{
			m_loggers[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				DL_TIMESTAMP, context.simul_time);
			m_loggers[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_TORQUE, fi.motor_torque);
			m_loggers[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_WEIGHT, fi.weight);
			m_loggers[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_VEL_X, fi.wheel_speed.x);
			m_loggers[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_VEL_Y, fi.wheel_speed.y);
			m_loggers[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_FRIC_X, net_force_.x);
			m_loggers[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_FRIC_Y, net_force_.y);
		}

		// save it for optional rendering:
		if (m_world->m_gui_options.show_forces)
		{
			const double forceScale =
				m_world->m_gui_options.force_scale;	 // [meters/N]
			const mrpt::math::TPoint3D pt1(
				wPt.x, wPt.y, m_chassis_z_max * 1.1 + getPose().z);
			const mrpt::math::TPoint3D pt2 =
				pt1 + mrpt::math::TPoint3D(wForce.x, wForce.y, 0) * forceScale;
			force_vectors.push_back(mrpt::math::TSegment3D(pt1, pt2));
		}
	}

	// Save forces for optional rendering:
	if (m_world->m_gui_options.show_forces)
	{
		std::lock_guard<std::mutex> csl(m_force_segments_for_rendering_cs);
		m_force_segments_for_rendering = force_vectors;
	}
}

/** Override to do any required process right after the integration of dynamic
 * equations for each timestep */
void VehicleBase::simul_post_timestep(const TSimulContext& context)
{
	// Common part (update m_q, m_dq)
	Simulable::simul_post_timestep(context);
	for (auto& s : m_sensors) s->simul_post_timestep(context);

	// Integrate wheels' rotation:
	const size_t nW = getNumWheels();

	for (size_t i = 0; i < nW; i++)
	{
		// prepare data:
		Wheel& w = getWheelInfo(i);

		// Explicit Euler:
		w.setPhi(w.getPhi() + w.getW() * context.dt);

		// Wrap wheel spin position (angle), so it doesn't
		// become excessively large (it's actually unbound, but we don't want to
		// lose 'double' accuracy):
		const double cur_abs_phi = std::abs(w.getPhi());
		if (cur_abs_phi > 1e4)
			w.setPhi(
				::fmod(cur_abs_phi, 2 * M_PI) *
				(w.getPhi() < 0.0 ? -1.0 : 1.0));
	}

	const auto q = getPose();
	const auto dq = getTwist();

	m_loggers[LOGGER_POSE]->updateColumn(DL_TIMESTAMP, context.simul_time);
	m_loggers[LOGGER_POSE]->updateColumn(PL_Q_X, q.x);
	m_loggers[LOGGER_POSE]->updateColumn(PL_Q_Y, q.y);
	m_loggers[LOGGER_POSE]->updateColumn(PL_Q_Z, q.z);
	m_loggers[LOGGER_POSE]->updateColumn(PL_Q_YAW, q.yaw);
	m_loggers[LOGGER_POSE]->updateColumn(PL_Q_PITCH, q.pitch);
	m_loggers[LOGGER_POSE]->updateColumn(PL_Q_ROLL, q.roll);
	m_loggers[LOGGER_POSE]->updateColumn(PL_DQ_X, dq.vx);
	m_loggers[LOGGER_POSE]->updateColumn(PL_DQ_Y, dq.vy);
	m_loggers[LOGGER_POSE]->updateColumn(PL_DQ_Z, dq.omega);

	{
		writeLogStrings();
	}
}

/** Last time-step velocity of each wheel's center point (in local coords) */
void VehicleBase::getWheelsVelocityLocal(
	std::vector<mrpt::math::TPoint2D>& vels,
	const mrpt::math::TTwist2D& veh_vel_local) const
{
	// Each wheel velocity is:
	// v_w = v_veh + \omega \times wheel_pos
	// =>
	// v_w = v_veh + ( -w*y, w*x )

	const double w = veh_vel_local.omega;  // vehicle w

	const size_t nW = this->getNumWheels();
	vels.resize(nW);
	for (size_t i = 0; i < nW; i++)
	{
		const Wheel& wheel = getWheelInfo(i);

		vels[i].x = veh_vel_local.vx - w * wheel.y;
		vels[i].y = veh_vel_local.vy + w * wheel.x;
	}
}

mrpt::poses::CPose3D VehicleBase::internalGuiGetVisualPose()
{
	return mrpt::poses::CPose3D(getPose());
}

void VehicleBase::internal_internalGuiUpdate_sensors(
	mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical)
{
	for (auto& s : m_sensors) s->guiUpdate(viz, physical);
}

void VehicleBase::internal_internalGuiUpdate_forces(  //
	[[maybe_unused]] mrpt::opengl::COpenGLScene& scene)
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

void VehicleBase::updateMaxRadiusFromPoly()
{
	using namespace mrpt::math;

	m_max_radius = 0.001f;
	for (const auto& pt : m_chassis_poly)
	{
		const float n = pt.norm();
		mrpt::keep_max(m_max_radius, n);
	}
}

/** Create bodies, fixtures, etc. for the dynamical simulation */
void VehicleBase::create_multibody_system(b2World& world)
{
	// Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;

	m_b2d_body = world.CreateBody(&bodyDef);

	// Define shape of chassis:
	// ------------------------------
	{
		// Convert shape into Box2D format:
		const size_t nPts = m_chassis_poly.size();
		ASSERT_(nPts >= 3);
		ASSERT_LE_(nPts, (size_t)b2_maxPolygonVertices);
		std::vector<b2Vec2> pts(nPts);
		for (size_t i = 0; i < nPts; i++)
			pts[i] = b2Vec2(m_chassis_poly[i].x, m_chassis_poly[i].y);

		b2PolygonShape chassisPoly;
		chassisPoly.Set(&pts[0], nPts);
		// chassisPoly.m_radius = 1e-3;  // The "skin" depth of the body

		// Define the dynamic body fixture.
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &chassisPoly;
		fixtureDef.restitution = 0.01;

		// Set the box density to be non-zero, so it will be dynamic.
		b2MassData mass;
		chassisPoly.ComputeMass(
			&mass, 1);	// Mass with density=1 => compute area
		fixtureDef.density = m_chassis_mass / mass.mass;

		// Override the default friction.
		fixtureDef.friction = 0.3f;

		// Add the shape to the body.
		m_fixture_chassis = m_b2d_body->CreateFixture(&fixtureDef);

		// Compute center of mass:
		b2MassData vehMass;
		m_fixture_chassis->GetMassData(&vehMass);
		m_chassis_com.x = vehMass.center.x;
		m_chassis_com.y = vehMass.center.y;
	}

	// Define shape of wheels:
	// ------------------------------
	ASSERT_EQUAL_(m_fixture_wheels.size(), m_wheels_info.size());

	for (size_t i = 0; i < m_wheels_info.size(); i++)
	{
		b2PolygonShape wheelShape;
		wheelShape.SetAsBox(
			m_wheels_info[i].diameter * 0.5, m_wheels_info[i].width * 0.5,
			b2Vec2(m_wheels_info[i].x, m_wheels_info[i].y),
			m_wheels_info[i].yaw);

		// Define the dynamic body fixture.
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &wheelShape;
		fixtureDef.restitution = 0.05;

		// Set the box density to be non-zero, so it will be dynamic.
		b2MassData mass;
		wheelShape.ComputeMass(
			&mass, 1);	// Mass with density=1 => compute area
		fixtureDef.density = m_wheels_info[i].mass / mass.mass;

		// Override the default friction.
		fixtureDef.friction = 0.5f;

		m_fixture_wheels[i] = m_b2d_body->CreateFixture(&fixtureDef);
	}
}

void VehicleBase::internalGuiUpdate(
	mrpt::opengl::COpenGLScene& viz, mrpt::opengl::COpenGLScene& physical,
	bool childrenOnly)
{
	auto lck = mrpt::lockHelper(m_gui_mtx);

	// 1st time call?? -> Create objects
	// ----------------------------------
	const size_t nWs = this->getNumWheels();
	if (!m_gl_chassis)
	{
		m_gl_chassis = mrpt::opengl::CSetOfObjects::Create();
		m_gl_chassis->setName("vehicle_chassis_"s + m_name);

		// Wheels shape:
		m_gl_wheels.resize(nWs);
		for (size_t i = 0; i < nWs; i++)
		{
			m_gl_wheels[i] = mrpt::opengl::CSetOfObjects::Create();
			this->getWheelInfo(i).getAs3DObject(*m_gl_wheels[i]);
			m_gl_chassis->insert(m_gl_wheels[i]);
		}

		if (!childrenOnly)
		{
			// Robot shape:
			auto gl_poly = mrpt::opengl::CPolyhedron::CreateCustomPrism(
				m_chassis_poly, m_chassis_z_max - m_chassis_z_min);
			gl_poly->setLocation(0, 0, m_chassis_z_min);
			gl_poly->setColor_u8(m_chassis_color);
			m_gl_chassis->insert(gl_poly);
		}

		viz.insert(m_gl_chassis);
		physical.insert(m_gl_chassis);
	}

	// Update them:
	// ----------------------------------
	m_gl_chassis->setPose(getPose());

	for (size_t i = 0; i < nWs; i++)
	{
		const Wheel& w = getWheelInfo(i);
		m_gl_wheels[i]->setPose(mrpt::math::TPose3D(
			w.x, w.y, 0.5 * w.diameter, w.yaw, w.getPhi(), 0.0));
	}

	// Init on first use:
	if (!m_gl_forces)
	{
		// Visualization of forces:
		m_gl_forces = mrpt::opengl::CSetOfLines::Create();
		m_gl_forces->setLineWidth(3.0);
		m_gl_forces->setColor_u8(0xff, 0xff, 0xff);
		viz.insert(m_gl_forces);  // forces are in global coords
	}

	// Other common stuff:
	internal_internalGuiUpdate_sensors(viz, physical);
	internal_internalGuiUpdate_forces(viz);
}

void VehicleBase::initLoggers()
{
	m_loggers[LOGGER_POSE] = std::make_shared<CSVLogger>();
	//  m_loggers[LOGGER_POSE]->addColumn(DL_TIMESTAMP);
	//  m_loggers[LOGGER_POSE]->addColumn(PL_Q_X);
	//  m_loggers[LOGGER_POSE]->addColumn(PL_Q_Y);
	//  m_loggers[LOGGER_POSE]->addColumn(PL_Q_Z);
	//  m_loggers[LOGGER_POSE]->addColumn(PL_Q_YAW);
	//  m_loggers[LOGGER_POSE]->addColumn(PL_Q_PITCH);
	//  m_loggers[LOGGER_POSE]->addColumn(PL_Q_ROLL);
	//  m_loggers[LOGGER_POSE]->addColumn(PL_DQ_X);
	//  m_loggers[LOGGER_POSE]->addColumn(PL_DQ_Y);
	//  m_loggers[LOGGER_POSE]->addColumn(PL_DQ_Z);
	m_loggers[LOGGER_POSE]->setFilepath(
		m_log_path + "mvsim_" + m_name + LOGGER_POSE + ".log");

	for (size_t i = 0; i < getNumWheels(); i++)
	{
		m_loggers[LOGGER_WHEEL + std::to_string(i + 1)] =
			std::make_shared<CSVLogger>();
		//    m_loggers[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(DL_TIMESTAMP);
		//    m_loggers[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_TORQUE);
		//    m_loggers[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_WEIGHT);
		//    m_loggers[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_VEL_X);
		//    m_loggers[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_VEL_Y);
		//    m_loggers[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_FRIC_X);
		//    m_loggers[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_FRIC_Y);
		m_loggers[LOGGER_WHEEL + std::to_string(i + 1)]->setFilepath(
			m_log_path + "mvsim_" + m_name + LOGGER_WHEEL +
			std::to_string(i + 1) + ".log");
	}
}

void VehicleBase::writeLogStrings()
{
	std::map<std::string, std::shared_ptr<CSVLogger>>::iterator it;
	for (it = m_loggers.begin(); it != m_loggers.end(); ++it)
	{
		it->second->writeRow();
	}
}

void VehicleBase::apply_force(
	const mrpt::math::TVector2D& force, const mrpt::math::TPoint2D& applyPoint)
{
	ASSERT_(m_b2d_body);
	const b2Vec2 wPt = m_b2d_body->GetWorldPoint(b2Vec2(
		applyPoint.x, applyPoint.y));  // Application point -> world coords
	m_b2d_body->ApplyForce(b2Vec2(force.x, force.y), wPt, true /*wake up*/);
}

void VehicleBase::registerOnServer(mvsim::Client& c)
{
	// register myself, and my children objects:
	Simulable::registerOnServer(c);
	for (auto& sensor : m_sensors) sensor->registerOnServer(c);
}
