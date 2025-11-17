/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/random.h>
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
	{
		return;
	}

	done = true;

	REGISTER_VEHICLE_DYNAMICS("differential", DynamicsDifferential)
	REGISTER_VEHICLE_DYNAMICS("differential_3_wheels", DynamicsDifferential_3_wheels)
	REGISTER_VEHICLE_DYNAMICS("differential_4_wheels", DynamicsDifferential_4_wheels)
	REGISTER_VEHICLE_DYNAMICS("ackermann", DynamicsAckermann)
	REGISTER_VEHICLE_DYNAMICS("ackermann_drivetrain", DynamicsAckermannDrivetrain)
}

// Protected ctor:
VehicleBase::VehicleBase(World* parent, size_t nWheels)
	: VisualObject(parent), Simulable(parent), fixture_wheels_(nWheels, nullptr)
{
	// Create wheels:
	for (size_t i = 0; i < nWheels; i++)
	{
		wheels_info_.emplace_back(parent);
	}

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
	const World& parent, const rapidxml::xml_node<char>* xml_node)
{
	// Sanity checks:
	if (!xml_node)
	{
		throw runtime_error("[VehicleBase::register_vehicle_class] XML node is nullptr");
	}
	if (0 != strcmp(xml_node->name(), "vehicle:class"))
	{
		throw runtime_error(mrpt::format(
			"[VehicleBase::register_vehicle_class] XML element is '%s' "
			"('vehicle:class' expected)",
			xml_node->name()));
	}

	// Delay the replacement of this variable (used in Sensors) until
	// "veh" is constructed and we actually have a name:
	const std::set<std::string> varsRetain = {
		"NAME" /*sensor name*/, "PARENT_NAME" /*vehicle name*/};

	// Parse XML to solve for includes:
	veh_classes_registry.add(xml_to_str_solving_includes(parent, xml_node, varsRetain));
}

/** Class factory: Creates a vehicle from XML description of type
 * "<vehicle>...</vehicle>".  */
VehicleBase::Ptr VehicleBase::factory(World* parent, const rapidxml::xml_node<char>* root)
{
	register_all_veh_dynamics();

	using namespace std;
	using namespace rapidxml;

	if (!root)
	{
		throw runtime_error("[VehicleBase::factory] XML node is nullptr");
	}
	if (0 != strcmp(root->name(), "vehicle"))
	{
		throw runtime_error(mrpt::format(
			"[VehicleBase::factory] XML root element is '%s' ('vehicle' "
			"expected)",
			root->name()));
	}

	// "class": When a vehicle has a 'class="XXX"' attribute, look for each
	// parameter
	//  in the set of "root" + "class_root" XML nodes:
	// --------------------------------------------------------------------------------
	JointXMLnode<> nodes;

	std::vector<XML_Doc_Data::Ptr> scopedLifeDocs;

	// Solve includes:
	for (auto n = root->first_node(); n; n = n->next_sibling())
	{
		if (strcmp(n->name(), "include") != 0)
		{
			continue;
		}

		auto fileAttrb = n->first_attribute("file");
		ASSERTMSG_(fileAttrb, "XML tag '<include />' must have a 'file=\"xxx\"' attribute)");

		const std::string relFile =
			mvsim::parse(fileAttrb->value(), parent->user_defined_variables());
		const auto absFile = parent->local_to_abs_path(relFile);
		parent->logStr(
			mrpt::system::LVL_DEBUG,
			mrpt::format("XML parser: including file: '%s'", absFile.c_str()));

		std::map<std::string, std::string> vars;
		// Inherit the user-defined variables from parent scope
		vars = parent->user_defined_variables();
		// Plus new ones:
		for (auto attr = n->first_attribute(); attr; attr = attr->next_attribute())
		{
			if (strcmp(attr->name(), "file") == 0)
			{
				continue;
			}
			vars[attr->name()] = attr->value();
		}

		// Delay the replacement of this variable (used in Sensors) until "veh"
		// is constructed and we actually have a name:
		std::set<std::string> varsRetain = {"NAME" /*sensor name*/, "PARENT_NAME" /*vehicle name*/};

		const auto [xml, nRoot] = readXmlAndGetRoot(absFile, vars, varsRetain);

		// the XML document object must exist during this whole function scope
		scopedLifeDocs.emplace_back(xml);

		// recursive parse:
		nodes.add(nRoot->parent());
	}

	// ---
	// Always search in root. Also in the class root, if any:
	nodes.add(root);

	if (const xml_attribute<>* veh_class = root->first_attribute("class"); veh_class)
	{
		const string sClassName = veh_class->value();
		const rapidxml::xml_node<char>* class_root = veh_classes_registry.get(sClassName);
		if (!class_root)
		{
			throw runtime_error(mrpt::format(
				"[VehicleBase::factory] Vehicle class '%s' undefined", sClassName.c_str()));
		}

		nodes.add(class_root);
	}

	// Class factory according to: <dynamics class="XXX">
	// -------------------------------------------------

	const xml_node<>* dyn_node = nodes.first_node("dynamics");
	if (!dyn_node)
	{
		throw runtime_error("[VehicleBase::factory] Missing XML node <dynamics>");
	}

	const xml_attribute<>* dyn_class = dyn_node->first_attribute("class");
	if (!dyn_class || !dyn_class->value())
	{
		throw runtime_error(
			"[VehicleBase::factory] Missing mandatory attribute 'class' in "
			"node <dynamics>");
	}

	VehicleBase::Ptr veh = classFactory_vehicleDynamics.create(dyn_class->value(), parent);
	if (!veh)
	{
		throw runtime_error(mrpt::format(
			"[VehicleBase::factory] Unknown vehicle dynamics class '%s'", dyn_class->value()));
	}

	// Initialize here all common params shared by any polymorphic class:
	// -------------------------------------------------
	// attrib: name
	{
		const xml_attribute<>* attrib_name = root->first_attribute("name");
		if (attrib_name && attrib_name->value())
		{
			veh->name_ = mvsim::parse_variables(attrib_name->value(), {}, {});
		}
		else
		{
			// Default name:
			static int cnt = 0;
			veh->name_ = mrpt::format("veh%i", ++cnt);
		}
	}

	// Custom visualization 3D model:
	// -----------------------------------------------------------
	veh->parseVisual(nodes);

	// Initialize common params:
	// ---------------------------------------------------------------
	// <chassis ...> </chassis>
	if (const rapidxml::xml_node<char>* xml_chassis = dyn_node->first_node("chassis"); xml_chassis)
	{
		const std::map<std::string, std::string> varValues = {{"NAME", veh->name_}};

		// Attribs:
		TParameterDefinitions attribs;
		attribs["mass"] = TParamEntry("%lf", &veh->chassis_mass_);
		attribs["zmin"] = TParamEntry("%lf", &veh->chassis_z_min_);
		attribs["zmax"] = TParamEntry("%lf", &veh->chassis_z_max_);
		attribs["color"] = TParamEntry("%color", &veh->chassis_color_);

		parse_xmlnode_attribs(*xml_chassis, attribs, varValues, "[VehicleBase::factory]");

		// Shape node (optional, fallback to default shape if none found)
		const rapidxml::xml_node<char>* xml_shape = xml_chassis->first_node("shape");
		if (xml_shape)
		{
			mvsim::parse_xmlnode_shape(*xml_shape, veh->chassis_poly_, "[VehicleBase::factory]");
		}
	}

	// Initialize noisy odometry generation:
	auto& odoParam = veh->odometry_noise_;

	if (const xml_node<>* odom_node = nodes.first_node("odometry"); odom_node != nullptr)
	{
		TParameterDefinitions attribs;

		attribs["x_multiplier"] = TParamEntry("%f", &odoParam.x_multiplier);
		attribs["y_multiplier"] = TParamEntry("%f", &odoParam.y_multiplier);
		attribs["yaw_multiplier"] = TParamEntry("%f", &odoParam.yaw_multiplier);

		parse_xmlnode_attribs(*odom_node, attribs, {}, "[VehicleBase::factory]");
	}

	// Initialize class-specific params (chassis shape, etc.)
	// ---------------------------------------------------------------
	veh->dynamics_load_params_from_xml(dyn_node);

	// Auto shape node from visual?
	if (const rapidxml::xml_node<char>* xml_chassis = dyn_node->first_node("chassis"); xml_chassis)
	{
		if (const rapidxml::xml_node<char>* sfv = xml_chassis->first_node("shape_from_visual"); sfv)
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
			cs.setShapeManual(veh->chassis_poly_, veh->chassis_z_min_, veh->chassis_z_max_);

			veh->setCollisionShape(cs);
		}
	}

	veh->updateMaxRadiusFromPoly();

	// Common setup for simulable objects:
	// -----------------------------------------------------------
	veh->parseSimulable(nodes);

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
		const auto q = parent->applyWorldRenderOffset(veh->getPose());
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
			for (size_t i = 0; i < veh->getNumWheels(); i++)
			{
				veh->frictions_.push_back(
					std::make_shared<DefaultFriction>(*veh, nullptr /*default */));
			}
		}
		else
		{
			// Parse user parameters:
			for (size_t i = 0; i < veh->getNumWheels(); i++)
			{
				const auto friction =
					std::shared_ptr<FrictionBase>(FrictionBase::factory(*veh, frict_node));
				ASSERT_(friction);
				veh->frictions_.push_back(friction);
			}
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

VehicleBase::Ptr VehicleBase::factory(World* parent, const std::string& xml_text)
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
			"[VehicleBase::factory] XML parse error (Line %u): %s", static_cast<unsigned>(line),
			e.what()));
	}
	return VehicleBase::factory(parent, xml.first_node());
}

void VehicleBase::simul_pre_timestep(const TSimulContext& context)
{
	Simulable::simul_pre_timestep(context);
	for (auto& s : sensors_)
	{
		s->simul_pre_timestep(context);
	}

	for (size_t i = 0; i < getNumWheels(); i++)
	{
		if (auto& l = loggers_[LOGGER_IDX_WHEELS + i]; l->isRecording())
		{
			if (!frictions_.at(i)->hasLogger())
			{
				frictions_.at(i)->setLogger(l);
			}
		}
	}

	// Update wheels position (they may turn, etc. as in an Ackermann
	// configuration)
	for (size_t i = 0; i < fixture_wheels_.size(); i++)
	{
		b2PolygonShape* wheelShape = dynamic_cast<b2PolygonShape*>(fixture_wheels_[i]->GetShape());
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

	// TODO: Use chassis cog point to estimate load on each wheel, caching it.
	// TODO-TODO: Use dynamic load transfer too!
	const double massPerWheel = getChassisMass() / static_cast<double>(nW);
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
		fi.Fz = weightPerWheel;
		fi.wheelCogLocalVel = wheelLocalVels[i];

		// eval friction (in the frame of the vehicle):
		const mrpt::math::TPoint2D F_r = frictions_.at(i)->evaluate_friction(fi);

		// Apply force:
		// Force vector -> world coords
		const b2Vec2 wForce = b2dBody_->GetWorldVector(b2Vec2(F_r.x, F_r.y));
		// Application point -> world coords
		const b2Vec2 wPt = b2dBody_->GetWorldPoint(b2Vec2(w.x, w.y));

		// printf("w%i: Lx=%6.3f Ly=%6.3f  | Gx=%11.9f
		// Gy=%11.9f\n",(int)i,net_force_.x,net_force_.y,wForce.x,wForce.y);

		b2dBody_->ApplyForce(wForce, wPt, true /*wake up*/);

		// log
		if (auto& l = loggers_[LOGGER_IDX_WHEELS + i]; l->isRecording())
		{
			auto& logger = *l;

			logger.updateColumn(DL_TIMESTAMP, context.simul_time);
			logger.updateColumn("wheel_pos_x", w.x);
			logger.updateColumn("wheel_pos_y", w.y);

			logger.updateColumn(DL_TIMESTAMP, context.simul_time);
			logger.updateColumn(WL_TORQUE, fi.motorTorque);
			logger.updateColumn(WL_FORCE_Z, fi.Fz);
			logger.updateColumn(WL_VEL_X, fi.wheelCogLocalVel.x);
			logger.updateColumn(WL_VEL_Y, fi.wheelCogLocalVel.y);
			logger.updateColumn(WL_FRIC_X, F_r.x);
			logger.updateColumn(WL_FRIC_Y, F_r.y);
		}

		// save it for optional rendering:
		if (world_->guiOptions_.show_forces)
		{
			// [meters/N]
			const double forceScale = world_->guiOptions_.force_scale;

			const mrpt::math::TPoint3D pt1(wPt.x, wPt.y, chassis_z_max_ * 1.1 + getPose().z);
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
	invoke_motor_controllers_post_step(context);

	// Common part (update q_, dq_)
	Simulable::simul_post_timestep(context);
	for (auto& s : sensors_)
	{
		s->simul_post_timestep(context);
	}

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
		{
			w.setPhi(::fmod(cur_abs_phi, 2 * M_PI) * (w.getPhi() < 0.0 ? -1.0 : 1.0));
		}
	}

	// Estimate 2D pose increment as 2D-projected 3D pose increment, plus noise and bias:
	{
		const auto curPose = getCPose3D();
		const auto odoIncr = odometry_pre_simul_pose_.has_value()
								 ? (mrpt::poses::CPose2D(curPose - *odometry_pre_simul_pose_))
								 : mrpt::poses::CPose2D();
		odometry_pre_simul_pose_ = curPose;

		odometry_ += odometry_noise_.actualDeltaToNoisyOdo(odoIncr);
	}

	const auto q = getPose();
	const auto dq = getTwist();

	if (auto& l = loggers_[LOGGER_IDX_POSE]; l->isRecording())
	{
		auto& logger = *l;
		logger.updateColumn(DL_TIMESTAMP, context.simul_time);
		logger.updateColumn(PL_Q_X, q.x);
		logger.updateColumn(PL_Q_Y, q.y);
		logger.updateColumn(PL_Q_Z, q.z);
		logger.updateColumn(PL_Q_YAW, q.yaw);
		logger.updateColumn(PL_Q_PITCH, q.pitch);
		logger.updateColumn(PL_Q_ROLL, q.roll);
		logger.updateColumn(PL_DQ_X, dq.vx);
		logger.updateColumn(PL_DQ_Y, dq.vy);
		logger.updateColumn(PL_DQ_Z, dq.omega);
		logger.updateColumn(PL_ODO_X, odometry_.x());
		logger.updateColumn(PL_ODO_Y, odometry_.y());
		logger.updateColumn(PL_ODO_YAW, odometry_.phi());
	}

	writeLogStrings();
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

bool mvsim::VehicleBase::isLogging() const
{
	if (loggers_.empty())
	{
		return false;
	}
	auto& l = *loggers_.begin();
	return l && l->isOpen();
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
		for (size_t i = 0; i < nPts; i++) pts[i] = b2Vec2(chassis_poly_[i].x, chassis_poly_[i].y);

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
		wheelShape.ComputeMass(&mass, 1);  // Mass with density=1 => compute area
		fixtureDef.density = wheels_info_[i].mass / mass.mass;

		// Override the default friction.
		fixtureDef.friction = 0.5f;

		fixture_wheels_[i] = b2dBody_->CreateFixture(&fixtureDef);
	}
}

void VehicleBase::internalGuiUpdate(
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& viz,
	const mrpt::optional_ref<mrpt::opengl::COpenGLScene>& physical, bool childrenOnly)
{
	// 1st time call?? -> Create objects
	// ----------------------------------
	const size_t nWs = this->getNumWheels();
	if (!glChassisViz_ && viz && physical)
	{
		glChassisViz_ = mrpt::opengl::CSetOfObjects::Create();
		glChassisViz_->setName("vehicle_chassis_"s + name_);

		glChassisPhysical_ = mrpt::opengl::CSetOfObjects::Create();
		glChassisPhysical_->setName("vehicle_chassis_"s + name_);

		// Wheels shape:
		glWheelsViz_.resize(nWs);
		glWheelsPhysical_.resize(nWs);
		for (size_t i = 0; i < nWs; i++)
		{
			glWheelsViz_[i] = mrpt::opengl::CSetOfObjects::Create();
			this->getWheelInfo(i).getAs3DObject(*glWheelsViz_[i], false);
			glChassisViz_->insert(glWheelsViz_[i]);

			glWheelsPhysical_[i] = mrpt::opengl::CSetOfObjects::Create();
			this->getWheelInfo(i).getAs3DObject(*glWheelsPhysical_[i], true);
			glChassisPhysical_->insert(glWheelsPhysical_[i]);
		}

		if (!childrenOnly)
		{
			// Robot shape:
			auto gl_poly = mrpt::opengl::CPolyhedron::CreateCustomPrism(
				chassis_poly_, chassis_z_max_ - chassis_z_min_);
			gl_poly->setLocation(0, 0, chassis_z_min_);
			gl_poly->setColor_u8(chassis_color_);
			glChassisViz_->insert(gl_poly);
			glChassisPhysical_->insert(gl_poly);
		}

		viz->get().insert(glChassisViz_);
		physical->get().insert(glChassisPhysical_);

		glInit_ = true;
	}

	// Update them:
	// ----------------------------------
	// If "viz" does not have a value, it's because we are already inside a
	// setPose() change event, so my caller already holds the mutex and we don't
	// need/can't acquire it again:
	const auto objectPose = viz.has_value() ? getPose() : getPoseNoLock();
	const auto pp = parent()->applyWorldRenderOffset(objectPose);

	if (glInit_)
	{
		glChassisViz_->setPose(pp);
		glChassisPhysical_->setPose(pp);
		for (size_t i = 0; i < nWs; i++)
		{
			const Wheel& w = getWheelInfo(i);
			glWheelsPhysical_[i]->setPose(
				mrpt::math::TPose3D(w.x, w.y, 0.5 * w.diameter, w.yaw, w.getPhi(), 0.0));
			glWheelsViz_[i]->setPose(
				mrpt::math::TPose3D(w.x, w.y, 0.5 * w.diameter, w.yaw, w.getPhi(), 0.0));

			if (!w.linked_yaw_object_name.empty())
			{
				auto glLinked = VisualObject::glCustomVisual_->getByName(w.linked_yaw_object_name);
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
		glForces_->setPose(parent()->applyWorldRenderOffset(mrpt::poses::CPose3D::Identity()));

		viz->get().insert(glForces_);  // forces are in global coords
	}
	if (!glMotorTorques_ && viz)
	{
		// Visualization of forces:
		glMotorTorques_ = mrpt::opengl::CSetOfLines::Create();
		glMotorTorques_->setLineWidth(3.0);
		glMotorTorques_->setColor_u8(0xff, 0x00, 0x00);
		glMotorTorques_->setPose(
			parent()->applyWorldRenderOffset(mrpt::poses::CPose3D::Identity()));
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
	loggers_.clear();

	//[0]: logger for vehicle pose:
	loggers_.resize(1 + getNumWheels());

	loggers_[LOGGER_IDX_POSE] = std::make_shared<CSVLogger>();
	loggers_[LOGGER_IDX_POSE]->setFilepath(log_path_ + "mvsim_" + name_ + "_pose.csv");

	for (size_t i = 0; i < getNumWheels(); i++)
	{
		loggers_[LOGGER_IDX_WHEELS + i] = std::make_shared<CSVLogger>();
		loggers_[LOGGER_IDX_WHEELS + i]->setFilepath(
			log_path_ + "mvsim_" + name_ + "_wheel_" + std::to_string(i + 1) + ".csv");
	}
}

void VehicleBase::writeLogStrings()
{
	for (auto& logger : loggers_)
	{
		if (!logger->isRecording())
		{
			continue;
		}

		logger->writeRow();
	}
}

void VehicleBase::apply_force(
	const mrpt::math::TVector2D& force, const mrpt::math::TPoint2D& applyPoint)
{
	ASSERT_(b2dBody_);
	// Application point -> world coords
	const b2Vec2 wPt = b2dBody_->GetWorldPoint(b2Vec2(applyPoint.x, applyPoint.y));
	b2dBody_->ApplyForce(b2Vec2(force.x, force.y), wPt, true /*wake up*/);
}

void VehicleBase::registerOnServer(mvsim::Client& c)
{
	// register myself, and my children objects:
	Simulable::registerOnServer(c);
	for (auto& sensor : sensors_)
	{
		sensor->registerOnServer(c);
	}
}

void VehicleBase::chassisAndWheelsVisible(bool visible)
{
	if (glChassisViz_)
	{
		glChassisViz_->setVisibility(visible);
	}
	for (auto& glW : glWheelsViz_)
	{
		if (glW)
		{
			glW->setVisibility(visible);
		}
	}
}

VehicleBase::OdometryNoise::OdometryNoise()
{
	auto& rng = mrpt::random::getRandomGenerator();
	x_multiplier = rng.drawGaussian1D(1.0, 0.02);
	y_multiplier = rng.drawGaussian1D(1.0, 0.02);
	yaw_multiplier = rng.drawGaussian1D(1.0, 0.02);
}
