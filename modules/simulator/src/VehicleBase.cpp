/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
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
	: VisualObject(parent), Simulable(parent), fixture_wheels_(nWheels, nullptr)
{
	// Create wheels:
	for (size_t i = 0; i < nWheels; i++) wheels_info_.emplace_back(parent);

	// Default shape:
	chassis_poly_.emplace_back(-0.4, -0.5);
	chassis_poly_.emplace_back(-0.4, 0.5);
	chassis_poly_.emplace_back(0.4, 0.5);
	chassis_poly_.emplace_back(0.6, 0.3);
	chassis_poly_.emplace_back(0.6, -0.3);
	chassis_poly_.emplace_back(0.4, -0.5);
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
	JointXMLnode<> nodes;

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
		const auto absFile = parent->local_to_abs_path(relFile);
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

		nodes.add(nRoot->parent());
	}

	// ---
	{
		// Always search in root. Also in the class root, if any:
		nodes.add(root);

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

			nodes.add(class_root);
		}
	}

	// Class factory according to: <dynamics class="XXX">
	// -------------------------------------------------
	const xml_node<>* dyn_node = nodes.first_node("dynamics");
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
			veh->name_ = attrib_name->value();
		}
		else
		{
			// Default name:
			static int cnt = 0;
			veh->name_ = mrpt::format("veh%i", ++cnt);
		}
	}

	// Common setup for simulable objects:
	// -----------------------------------------------------------
	veh->parseSimulable(nodes);

	// Custom visualization 3D model:
	// -----------------------------------------------------------
	veh->parseVisual(nodes);

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
			const auto& bbVis = veh->collisionShape();
			if (!bbVis.has_value())
			{
				THROW_EXCEPTION(
					"Error: Tag <shape_from_visual/> found but no "
					"<visual> entry seems to have been found while parsing "
					"<vehicle>");
			}
			const auto& bb = bbVis.value();
			if (bb.volume() == 0)
			{
				THROW_EXCEPTION(
					"Error: Tag <shape_from_visual/> found but bounding box of "
					"visual object seems incorrect, while parsing <vehicle>");
			}

			// Set contour polygon:
			veh->chassis_poly_ = bb.getContour();
			veh->chassis_z_min_ = bb.zMin();
			veh->chassis_z_max_ = bb.zMax();
		}
		else
		{
			// Update collision shape from shape loaded from XML:
			Shape2p5 cs;
			cs.setShapeManual(
				veh->chassis_poly_, veh->chassis_z_min_, veh->chassis_z_max_);

			veh->setCollisionShape(cs);
		}
	}

	veh->updateMaxRadiusFromPoly();

	// <Optional> Log path. If not specified, app folder will be used
	// -----------------------------------------------------------
	{
		const xml_node<>* log_path_node = nodes.first_node("log_path");
		if (log_path_node)
		{
			// Parse:
			veh->log_path_ = log_path_node->value();
		}
	}

	veh->initLoggers();

	// Register bodies, fixtures, etc. in Box2D simulator:
	// ----------------------------------------------------
	veh->create_multibody_system(*parent->getBox2DWorld());

	if (veh->b2dBody_)
	{
		// Init pos:
		const auto q = veh->getPose();
		const auto dq = veh->getTwist();

		veh->b2dBody_->SetTransform(b2Vec2(q.x, q.y), q.yaw);
		// Init vel:
		veh->b2dBody_->SetLinearVelocity(b2Vec2(dq.vx, dq.vy));
		veh->b2dBody_->SetAngularVelocity(dq.omega);
	}

	// Friction model:
	// Parse <friction> node, or assume default linear model:
	// -----------------------------------------------------------
	{
		const xml_node<>* frict_node = nodes.first_node("friction");
		if (!frict_node)
		{
			// Default:
			veh->friction_ =
				std::make_shared<DefaultFriction>(*veh, nullptr /*default */);
		}
		else
		{
			// Parse:
			veh->friction_ = std::shared_ptr<FrictionBase>(
				FrictionBase::factory(*veh, frict_node));
			ASSERT_(veh->friction_);
		}
	}

	// Sensors: <sensor class='XXX'> entries
	// -------------------------------------------------
	for (const auto& xmlNode : nodes)
	{
		if (!strcmp(xmlNode->name(), "sensor"))
		{
			SensorBase::Ptr se = SensorBase::factory(*veh, xmlNode);
			veh->sensors_.push_back(SensorBase::Ptr(se));
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
	for (auto& s : sensors_) s->simul_pre_timestep(context);

	// Update wheels position (they may turn, etc. as in an Ackermann
	// configuration)
	for (size_t i = 0; i < fixture_wheels_.size(); i++)
	{
		b2PolygonShape* wheelShape =
			dynamic_cast<b2PolygonShape*>(fixture_wheels_[i]->GetShape());
		wheelShape->SetAsBox(
			wheels_info_[i].diameter * 0.5, wheels_info_[i].width * 0.5,
			b2Vec2(wheels_info_[i].x, wheels_info_[i].y), wheels_info_[i].yaw);
	}

	// Apply motor forces/torques:
	const std::vector<double> wheelTorque = invoke_motor_controllers(context);

	// Apply friction model at each wheel:
	const size_t nW = getNumWheels();
	ASSERT_EQUAL_(wheelTorque.size(), nW);

	// Part of the vehicle weight on each wheel:
	const double gravity = parent()->get_gravity();
	MRPT_TODO("Use chassis cog point");
	const double massPerWheel = getChassisMass() / nW;
	const double weightPerWheel = massPerWheel * gravity;

	const std::vector<mrpt::math::TVector2D> wheelLocalVels =
		getWheelsVelocityLocal(getVelocityLocal());

	ASSERT_EQUAL_(wheelLocalVels.size(), nW);

	// For visualization only
	std::vector<mrpt::math::TSegment3D> forceVectors, torqueVectors;

	for (size_t i = 0; i < nW; i++)
	{
		// prepare data:
		Wheel& w = getWheelInfo(i);

		FrictionBase::TFrictionInput fi(context, w);
		fi.motorTorque = -wheelTorque[i];  // "-" => Forwards is negative
		fi.weight = weightPerWheel;
		fi.wheelCogLocalVel = wheelLocalVels[i];

		friction_->setLogger(
			getLoggerPtr(LOGGER_WHEEL + std::to_string(i + 1)));

		// eval friction (in the frame of the vehicle):
		const mrpt::math::TPoint2D F_r = friction_->evaluate_friction(fi);

		// Apply force:
		// Force vector -> world coords
		const b2Vec2 wForce = b2dBody_->GetWorldVector(b2Vec2(F_r.x, F_r.y));
		// Application point -> world coords
		const b2Vec2 wPt = b2dBody_->GetWorldPoint(b2Vec2(w.x, w.y));

		// printf("w%i: Lx=%6.3f Ly=%6.3f  | Gx=%11.9f
		// Gy=%11.9f\n",(int)i,net_force_.x,net_force_.y,wForce.x,wForce.y);

		b2dBody_->ApplyForce(wForce, wPt, true /*wake up*/);

		// log
		{
			loggers_[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				DL_TIMESTAMP, context.simul_time);
			loggers_[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_TORQUE, fi.motorTorque);
			loggers_[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_WEIGHT, fi.weight);
			loggers_[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_VEL_X, fi.wheelCogLocalVel.x);
			loggers_[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_VEL_Y, fi.wheelCogLocalVel.y);
			loggers_[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_FRIC_X, F_r.x);
			loggers_[LOGGER_WHEEL + std::to_string(i + 1)]->updateColumn(
				WL_FRIC_Y, F_r.y);
		}

		// save it for optional rendering:
		if (world_->guiOptions_.show_forces)
		{
			// [meters/N]
			const double forceScale = world_->guiOptions_.force_scale;

			const mrpt::math::TPoint3D pt1(
				wPt.x, wPt.y, chassis_z_max_ * 1.1 + getPose().z);
			const mrpt::math::TPoint3D pt2 =
				pt1 + mrpt::math::TPoint3D(wForce.x, wForce.y, 0) * forceScale;

			forceVectors.emplace_back(pt1, pt2);

			const mrpt::math::TPoint3D pt2_t =
				pt1 + mrpt::math::TPoint3D(0, 0, wheelTorque[i]) * forceScale;

			torqueVectors.emplace_back(pt1, pt2_t);
		}
	}

	// Save forces for optional rendering:
	if (world_->guiOptions_.show_forces)
	{
		std::lock_guard<std::mutex> csl(forceSegmentsForRenderingMtx_);
		forceSegmentsForRendering_ = forceVectors;
		torqueSegmentsForRendering_ = torqueVectors;
	}
}

/** Override to do any required process right after the integration of dynamic
 * equations for each timestep */
void VehicleBase::simul_post_timestep(const TSimulContext& context)
{
	// Common part (update q_, dq_)
	Simulable::simul_post_timestep(context);
	for (auto& s : sensors_) s->simul_post_timestep(context);

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

	loggers_[LOGGER_POSE]->updateColumn(DL_TIMESTAMP, context.simul_time);
	loggers_[LOGGER_POSE]->updateColumn(PL_Q_X, q.x);
	loggers_[LOGGER_POSE]->updateColumn(PL_Q_Y, q.y);
	loggers_[LOGGER_POSE]->updateColumn(PL_Q_Z, q.z);
	loggers_[LOGGER_POSE]->updateColumn(PL_Q_YAW, q.yaw);
	loggers_[LOGGER_POSE]->updateColumn(PL_Q_PITCH, q.pitch);
	loggers_[LOGGER_POSE]->updateColumn(PL_Q_ROLL, q.roll);
	loggers_[LOGGER_POSE]->updateColumn(PL_DQ_X, dq.vx);
	loggers_[LOGGER_POSE]->updateColumn(PL_DQ_Y, dq.vy);
	loggers_[LOGGER_POSE]->updateColumn(PL_DQ_Z, dq.omega);

	{
		writeLogStrings();
	}
}

/** Last time-step velocity of each wheel's center point (in local coords) */
std::vector<mrpt::math::TVector2D> VehicleBase::getWheelsVelocityLocal(
	const mrpt::math::TTwist2D& veh_vel_local) const
{
	// Each wheel velocity is:
	// v_w = v_veh + \omega \times wheel_pos
	// =>
	// v_w = v_veh + ( -w*y, w*x )

	const double w = veh_vel_local.omega;  // vehicle w

	const size_t nW = this->getNumWheels();
	std::vector<mrpt::math::TVector2D> vels(nW);

	for (size_t i = 0; i < nW; i++)
	{
		const Wheel& wheel = getWheelInfo(i);

		vels[i].x = veh_vel_local.vx - w * wheel.y;
		vels[i].y = veh_vel_local.vy + w * wheel.x;
	}
	return vels;
}

void VehicleBase::internal_internalGuiUpdate_sensors(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical)
{
	for (auto& s : sensors_) s->guiUpdate(viz, physical);
}

void VehicleBase::internal_internalGuiUpdate_forces(  //
	[[maybe_unused]] mrpt::opengl::COpenGLScene& scene)
{
	if (world_->guiOptions_.show_forces)
	{
		std::lock_guard<std::mutex> csl(forceSegmentsForRenderingMtx_);
		glForces_->clear();
		glForces_->appendLines(forceSegmentsForRendering_);
		glForces_->setVisibility(true);

		glMotorTorques_->clear();
		glMotorTorques_->appendLines(torqueSegmentsForRendering_);
		glMotorTorques_->setVisibility(true);
	}
	else
	{
		glForces_->setVisibility(false);
		glMotorTorques_->setVisibility(false);
	}
}

void VehicleBase::updateMaxRadiusFromPoly()
{
	using namespace mrpt::math;

	maxRadius_ = 0.001f;
	for (const auto& pt : chassis_poly_)
	{
		const float n = pt.norm();
		mrpt::keep_max(maxRadius_, n);
	}
}

/** Create bodies, fixtures, etc. for the dynamical simulation */
void VehicleBase::create_multibody_system(b2World& world)
{
	// Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;

	b2dBody_ = world.CreateBody(&bodyDef);

	// Define shape of chassis:
	// ------------------------------
	{
		// Convert shape into Box2D format:
		const size_t nPts = chassis_poly_.size();
		ASSERT_(nPts >= 3);
		ASSERT_LE_(nPts, (size_t)b2_maxPolygonVertices);
		std::vector<b2Vec2> pts(nPts);
		for (size_t i = 0; i < nPts; i++)
			pts[i] = b2Vec2(chassis_poly_[i].x, chassis_poly_[i].y);

		b2PolygonShape chassisPoly;
		chassisPoly.Set(&pts[0], nPts);

		// FIXED value by design in b2Box: The "skin" depth of the body
		chassisPoly.m_radius = 2.5e-3;	// b2_polygonRadius;

		// Define the dynamic body fixture.
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &chassisPoly;
		fixtureDef.restitution = 0.01;

		// Set the box density to be non-zero, so it will be dynamic.
		b2MassData mass;
		// Mass with density=1 => compute area
		chassisPoly.ComputeMass(&mass, 1);
		fixtureDef.density = chassis_mass_ / mass.mass;

		// Override the default friction.
		fixtureDef.friction = 0;

		// Add the shape to the body.
		fixture_chassis_ = b2dBody_->CreateFixture(&fixtureDef);

		// Compute center of mass:
		b2MassData vehMass;
		fixture_chassis_->GetMassData(&vehMass);
		chassis_com_.x = vehMass.center.x;
		chassis_com_.y = vehMass.center.y;
	}

	// Define shape of wheels:
	// ------------------------------
	ASSERT_EQUAL_(fixture_wheels_.size(), wheels_info_.size());

	for (size_t i = 0; i < wheels_info_.size(); i++)
	{
		b2PolygonShape wheelShape;
		wheelShape.SetAsBox(
			wheels_info_[i].diameter * 0.5, wheels_info_[i].width * 0.5,
			b2Vec2(wheels_info_[i].x, wheels_info_[i].y), wheels_info_[i].yaw);

		// Define the dynamic body fixture.
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &wheelShape;
		fixtureDef.restitution = 0.05;

		// Set the box density to be non-zero, so it will be dynamic.
		b2MassData mass;
		wheelShape.ComputeMass(
			&mass, 1);	// Mass with density=1 => compute area
		fixtureDef.density = wheels_info_[i].mass / mass.mass;

		// Override the default friction.
		fixtureDef.friction = 0.5f;

		fixture_wheels_[i] = b2dBody_->CreateFixture(&fixtureDef);
	}
}

void VehicleBase::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical,
	bool childrenOnly)
{
	// 1st time call?? -> Create objects
	// ----------------------------------
	const size_t nWs = this->getNumWheels();
	if (!glChassis_ && viz && physical)
	{
		glChassis_ = mrpt::opengl::CSetOfObjects::Create();
		glChassis_->setName("vehicle_chassis_"s + name_);

		// Wheels shape:
		glWheels_.resize(nWs);
		for (size_t i = 0; i < nWs; i++)
		{
			glWheels_[i] = mrpt::opengl::CSetOfObjects::Create();
			this->getWheelInfo(i).getAs3DObject(*glWheels_[i]);
			glChassis_->insert(glWheels_[i]);
		}

		if (!childrenOnly)
		{
			// Robot shape:
			auto gl_poly = mrpt::opengl::CPolyhedron::CreateCustomPrism(
				chassis_poly_, chassis_z_max_ - chassis_z_min_);
			gl_poly->setLocation(0, 0, chassis_z_min_);
			gl_poly->setColor_u8(chassis_color_);
			glChassis_->insert(gl_poly);
		}

		viz->get().insert(glChassis_);
		physical->get().insert(glChassis_);
	}

	// Update them:
	// ----------------------------------
	// If "viz" does not have a value, it's because we are already inside a
	// setPose() change event, so my caller already holds the mutex and we don't
	// need/can't acquire it again:
	const auto objectPose = viz.has_value() ? getPose() : getPoseNoLock();

	if (glChassis_)
	{
		glChassis_->setPose(objectPose);
		for (size_t i = 0; i < nWs; i++)
		{
			const Wheel& w = getWheelInfo(i);
			glWheels_[i]->setPose(mrpt::math::TPose3D(
				w.x, w.y, 0.5 * w.diameter, w.yaw, w.getPhi(), 0.0));

			if (!w.linked_yaw_object_name.empty())
			{
				auto glLinked = VisualObject::glCustomVisual_->getByName(
					w.linked_yaw_object_name);
				if (!glLinked)
				{
					THROW_EXCEPTION_FMT(
						"Wheel #%zu has linked_yaw_object_name='%s' but parent "
						"vehicle '%s' does not have any custom visual group "
						"with that name.",
						i, w.linked_yaw_object_name.c_str(), name_.c_str());
				}
				auto p = glLinked->getPose();
				p.yaw = w.yaw + w.linked_yaw_offset;
				glLinked->setPose(p);
			}
		}
	}

	// Init on first use:
	if (!glForces_ && viz)
	{
		// Visualization of forces:
		glForces_ = mrpt::opengl::CSetOfLines::Create();
		glForces_->setLineWidth(3.0);
		glForces_->setColor_u8(0xff, 0xff, 0xff);
		viz->get().insert(glForces_);  // forces are in global coords
	}
	if (!glMotorTorques_ && viz)
	{
		// Visualization of forces:
		glMotorTorques_ = mrpt::opengl::CSetOfLines::Create();
		glMotorTorques_->setLineWidth(3.0);
		glMotorTorques_->setColor_u8(0xff, 0x00, 0x00);
		viz->get().insert(glMotorTorques_);	 // torques are in global coords
	}

	// Other common stuff:
	if (viz && physical)
	{
		internal_internalGuiUpdate_sensors(viz->get(), physical->get());
		internal_internalGuiUpdate_forces(viz->get());
	}
}

void VehicleBase::initLoggers()
{
	loggers_[LOGGER_POSE] = std::make_shared<CSVLogger>();
	//  loggers_[LOGGER_POSE]->addColumn(DL_TIMESTAMP);
	//  loggers_[LOGGER_POSE]->addColumn(PL_Q_X);
	//  loggers_[LOGGER_POSE]->addColumn(PL_Q_Y);
	//  loggers_[LOGGER_POSE]->addColumn(PL_Q_Z);
	//  loggers_[LOGGER_POSE]->addColumn(PL_Q_YAW);
	//  loggers_[LOGGER_POSE]->addColumn(PL_Q_PITCH);
	//  loggers_[LOGGER_POSE]->addColumn(PL_Q_ROLL);
	//  loggers_[LOGGER_POSE]->addColumn(PL_DQ_X);
	//  loggers_[LOGGER_POSE]->addColumn(PL_DQ_Y);
	//  loggers_[LOGGER_POSE]->addColumn(PL_DQ_Z);
	loggers_[LOGGER_POSE]->setFilepath(
		log_path_ + "mvsim_" + name_ + LOGGER_POSE + ".log");

	for (size_t i = 0; i < getNumWheels(); i++)
	{
		loggers_[LOGGER_WHEEL + std::to_string(i + 1)] =
			std::make_shared<CSVLogger>();
		//    loggers_[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(DL_TIMESTAMP);
		//    loggers_[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_TORQUE);
		//    loggers_[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_WEIGHT);
		//    loggers_[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_VEL_X);
		//    loggers_[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_VEL_Y);
		//    loggers_[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_FRIC_X);
		//    loggers_[LOGGER_WHEEL + std::to_string(i +
		//    1)]->addColumn(WL_FRIC_Y);
		loggers_[LOGGER_WHEEL + std::to_string(i + 1)]->setFilepath(
			log_path_ + "mvsim_" + name_ + LOGGER_WHEEL +
			std::to_string(i + 1) + ".log");
	}
}

void VehicleBase::writeLogStrings()
{
	std::map<std::string, std::shared_ptr<CSVLogger>>::iterator it;
	for (it = loggers_.begin(); it != loggers_.end(); ++it)
	{
		it->second->writeRow();
	}
}

void VehicleBase::apply_force(
	const mrpt::math::TVector2D& force, const mrpt::math::TPoint2D& applyPoint)
{
	ASSERT_(b2dBody_);
	const b2Vec2 wPt = b2dBody_->GetWorldPoint(b2Vec2(
		applyPoint.x, applyPoint.y));  // Application point -> world coords
	b2dBody_->ApplyForce(b2Vec2(force.x, force.y), wPt, true /*wake up*/);
}

void VehicleBase::registerOnServer(mvsim::Client& c)
{
	// register myself, and my children objects:
	Simulable::registerOnServer(c);
	for (auto& sensor : sensors_) sensor->registerOnServer(c);
}

void VehicleBase::chassisAndWheelsVisible(bool visible)
{
	if (glChassis_) glChassis_->setVisibility(visible);
	for (auto& glW : glWheels_)
	{
		if (glW) glW->setVisibility(visible);
	}
}
