/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CPolyhedron.h>
#include <mvsim/FrictionModels/DefaultFriction.h>  // For use as default model
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/World.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
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
	: CVisualObject(parent), Simulable(parent), fixture_wheels_(nWheels, nullptr)
{
	// VehicleBase manages its own physical-scene representation via
	// glChassisPhysical_, so the custom visual (which carries wheel viz
	// objects with XYZ-corner markers) must NOT be inserted into the physical
	// scene, otherwise camera/lidar sensors would see the XYZ corners.
	insertCustomVizIntoPhysical_ = false;

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
VehicleBase::Ptr VehicleBase::factory(World* parent, const rapidxml::xml_node<char>* rootIn)
{
	register_all_veh_dynamics();

	using namespace std;
	using namespace rapidxml;

	if (!rootIn)
	{
		throw runtime_error("[VehicleBase::factory] XML node is nullptr");
	}
	if (0 != strcmp(rootIn->name(), "vehicle"))
	{
		throw runtime_error(mrpt::format(
			"[VehicleBase::factory] XML root element is '%s' ('vehicle' "
			"expected)",
			rootIn->name()));
	}

	// "class": When a vehicle has a 'class="XXX"' attribute, look for each
	// parameter
	//  in the set of "root" + "class_root" XML nodes:
	// --------------------------------------------------------------------------------
	JointXMLnode<> nodes;

	// Resolve <include> and <if> tags found at any depth inside the instance
	// definition. Variables NAME and PARENT_NAME (used in sensors) are kept
	// until "veh" is constructed and we actually have a name.
	// The resolved document must exist during this whole function scope.
	const std::set<std::string> varsRetain = {
		"NAME" /*sensor name*/, "PARENT_NAME" /*vehicle name*/};
	const auto [resolvedDoc, root] = xml_node_solving_includes(*parent, rootIn, varsRetain);
	(void)resolvedDoc;

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
			// World variables, e.g. for names in <for> loops:
			veh->name_ =
				mvsim::parse_variables(attrib_name->value(), parent->user_defined_variables(), {});
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
		attribs["linear_damping"] = TParamEntry("%lf", &veh->linear_damping_);
		attribs["angular_damping"] = TParamEntry("%lf", &veh->angular_damping_);

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

	// <suspension> (optional, at the vehicle level, so an instance can
	// override the one of its class):
	if (const xml_node<>* susp_node = nodes.first_node("suspension"); susp_node)
	{
		auto& cfg = veh->suspensionConfig_;
		const std::map<std::string, std::string> varValues = {{"NAME", veh->name_}};
		const char* ctx = "[VehicleBase::factory] <suspension>";

		TParameterDefinitions attribs;
		attribs["enabled"] = TParamEntry("%bool", &cfg.enabled);
		parse_xmlnode_attribs(*susp_node, attribs, varValues, ctx);

		double I_xx = 0;
		double I_yy = 0;
		double cogHeight = 0;
		TParameterDefinitions params;
		params["frequency_hz"] = TParamEntry("%lf", &cfg.frequency_hz);
		params["damping_ratio"] = TParamEntry("%lf", &cfg.damping_ratio);
		params["I_xx"] = TParamEntry("%lf", &I_xx);
		params["I_yy"] = TParamEntry("%lf", &I_yy);
		params["cog_height"] = TParamEntry("%lf", &cogHeight);
		parse_xmlnode_children_as_param(*susp_node, params, varValues, ctx);
		if (I_xx > 0)
		{
			cfg.I_xx = I_xx;
		}
		if (I_yy > 0)
		{
			cfg.I_yy = I_yy;
		}
		if (cogHeight > 0)
		{
			cfg.cog_height = cogHeight;
		}

		// Per-corner overrides: <wheel index="0" stiffness="..." damping="..."/>
		for (const xml_node<>* n = susp_node->first_node("wheel"); n; n = n->next_sibling("wheel"))
		{
			int index = -1;
			double k = -1;
			double c = -1;
			TParameterDefinitions wp;
			wp["index"] = TParamEntry("%i", &index);
			wp["stiffness"] = TParamEntry("%lf", &k);
			wp["damping"] = TParamEntry("%lf", &c);
			parse_xmlnode_attribs(*n, wp, varValues, ctx);
			if (index < 0)
			{
				THROW_EXCEPTION("<suspension><wheel> requires an 'index' attribute");
			}
			if (k > 0)
			{
				cfg.stiffness[index] = k;
			}
			if (c >= 0)
			{
				cfg.damping[index] = c;
			}
		}
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

	// <Optional> auto_start_recording
	if (const xml_node<>* asr_node = nodes.first_node("auto_start_recording"); asr_node)
	{
		const std::string val = asr_node->value();
		veh->auto_start_recording_ = (val == "true" || val == "1");
	}
	if (veh->auto_start_recording_)
	{
		veh->setRecording(true);
	}

	// Register bodies, fixtures, etc. in Box2D simulator:
	// ----------------------------------------------------
	veh->create_multibody_system(*parent->getBox2DWorld());

	if (veh->b2dBody_)
	{
		// Init pos:
		const auto q = parent->applyWorldRenderOffset(veh->getPose());
		const auto dq_com = veh->getComVelocityGlobal();

		veh->b2dBody_->SetTransform(
			b2Vec2(static_cast<float>(q.x), static_cast<float>(q.y)), static_cast<float>(q.yaw));

		// Init vel:
		veh->b2dBody_->SetLinearVelocity(
			b2Vec2(static_cast<float>(dq_com.vx), static_cast<float>(dq_com.vy)));
		veh->b2dBody_->SetAngularVelocity(static_cast<float>(dq_com.omega));
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

	veh->initSuspension();

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

namespace
{
// Vertical loads N_i at the support points p_i satisfying the force and moment
// balance A*N = b, A rows: (1, x_i, y_i). Statically indeterminate for more
// than 3 points: minimum-norm solution N = A^T (A A^T)^-1 b, removing points
// that would need negative loads (the body would tip over them).
// Returns false if the balance cannot be satisfied with non-negative loads.
bool solveSupportLoads(
	const std::vector<mrpt::math::TPoint2D>& p, const double b[3], std::vector<double>& loads)
{
	const size_t n = p.size();
	loads.assign(n, 0.0);
	std::vector<bool> active(n, true);
	for (size_t iter = 0; iter < n; iter++)
	{
		mrpt::math::CMatrixDouble33 AAt;
		AAt.setZero();
		for (size_t i = 0; i < n; i++)
		{
			if (!active[i])
			{
				continue;
			}
			const double row[3] = {1.0, p[i].x, p[i].y};
			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 3; c++)
				{
					AAt(r, c) += row[r] * row[c];
				}
			}
		}
		// Regularization: ignores the equations with no support (e.g. the
		// pitch balance of a 2-wheel differential robot)
		const double eps = 1e-9 * (AAt(0, 0) + AAt(1, 1) + AAt(2, 2));
		for (int r = 0; r < 3; r++)
		{
			AAt(r, r) += eps;
		}
		const mrpt::math::CMatrixDouble33 AAtInv = AAt.inverse_LLt();
		double lambda[3];
		for (int r = 0; r < 3; r++)
		{
			lambda[r] = AAtInv(r, 0) * b[0] + AAtInv(r, 1) * b[1] + AAtInv(r, 2) * b[2];
		}

		std::optional<size_t> mostNegative;
		for (size_t i = 0; i < n; i++)
		{
			loads[i] = active[i] ? lambda[0] + lambda[1] * p[i].x + lambda[2] * p[i].y : 0.0;
			if (loads[i] < 0 && (!mostNegative || loads[i] < loads[*mostNegative]))
			{
				mostNegative = i;
			}
		}
		if (!mostNegative)
		{
			break;
		}
		active[*mostNegative] = false;
	}

	// Check the balance, skipping the equations with no support:
	double sum[3] = {0, 0, 0};
	double spread[2] = {0, 0};
	for (size_t i = 0; i < n; i++)
	{
		loads[i] = std::max(0.0, loads[i]);
		sum[0] += loads[i];
		sum[1] += loads[i] * p[i].x;
		sum[2] += loads[i] * p[i].y;
		spread[0] = std::max(spread[0], std::abs(p[i].x - p[0].x));
		spread[1] = std::max(spread[1], std::abs(p[i].y - p[0].y));
	}
	const double tol = 1e-3 * std::abs(b[0]);
	bool ok = std::abs(sum[0] - b[0]) < tol;
	for (int k = 0; k < 2; k++)
	{
		if (spread[k] > 1e-6)
		{
			ok = ok && std::abs(sum[k + 1] - b[k + 1]) < tol * (1.0 + spread[k]);
		}
	}
	return ok;
}
}  // namespace

std::vector<double> VehicleBase::computeWheelLoads(const TSimulContext& context)
{
	const size_t nW = getNumWheels();
	const double gravity = parent()->get_gravity();
	const double M = b2dBody_ ? b2dBody_->GetMass() : 0;
	std::vector<double> loads(nW, 0.0);
	if (nW == 0 || M <= 0)
	{
		return loads;
	}

	// Center of mass (chassis + wheels), and its height over the ground:
	const b2Vec2 com = b2dBody_->GetLocalCenter();
	double h = chassis_mass_ * 0.5 * (chassis_z_min_ + chassis_z_max_);
	for (size_t i = 0; i < nW; i++)
	{
		h += wheels_info_[i].mass * 0.5 * wheels_info_[i].diameter;
	}
	h /= M;

	// Acceleration in the vehicle frame (low-pass filtered, to avoid a
	// feedback loop with the friction forces it depends on):
	{
		const auto acc = getLinearAcceleration();
		const double yaw = getPose().yaw;
		const mrpt::math::TVector2D accLocal(
			std::cos(yaw) * acc.x + std::sin(yaw) * acc.y,
			-std::sin(yaw) * acc.x + std::cos(yaw) * acc.y);
		constexpr double tau = 0.05;  // [s]
		const double a = context.dt / (tau + context.dt);
		accelForLoads_.x += a * (accLocal.x - accelForLoads_.x);
		accelForLoads_.y += a * (accLocal.y - accelForLoads_.y);
	}

	// With suspension, its springs and dampers carry the load transfer, and
	// the balance below gives their static loads:
	const mrpt::math::TVector2D acc =
		suspension_.initialized() ? mrpt::math::TVector2D(0, 0) : accelForLoads_;

	// Force and moment balance (no pitch/roll motion) of the rigid vehicle, in
	// its frame, with normal forces N_i at the wheel contact points (x_i,y_i):
	//  sum(N_i)       = M*gz
	//  sum(N_i * x_i) = M*(gz*xc - h*(ax - gx))
	//  sum(N_i * y_i) = M*(gz*yc - h*(ay - gy))
	// with (gx,gy,gz) the gravity in the vehicle frame (gz>0 downwards).
	const double gx = slopeDir_.x * gravity;
	const double gy = slopeDir_.y * gravity;
	const double gz = -slopeDir_.z * gravity;
	const double b[3] = {
		M * gz, M * (gz * com.x - h * (acc.x - gx)), M * (gz * com.y - h * (acc.y - gy))};

	std::vector<mrpt::math::TPoint2D> supports;
	for (size_t i = 0; i < nW; i++)
	{
		supports.emplace_back(wheels_info_[i].x, wheels_info_[i].y);
	}
	std::vector<double> sol;
	if (!solveSupportLoads(supports, b, sol))
	{
		// It cannot stand on its wheels (e.g. a semi-trailer): it also rests
		// on its joint anchors (whose load is not passed to the other body):
		for (const auto& j : world_->getListOfJoints())
		{
			if (j.bodyA_name == name_)
			{
				supports.emplace_back(j.anchorA);
			}
			else if (j.bodyB_name == name_)
			{
				supports.emplace_back(j.anchorB);
			}
		}
		if (!solveSupportLoads(supports, b, sol))
		{
			// Still not balanced (e.g. a parked trailer, resting on its legs):
			// best effort, keep the distribution but carry the whole weight:
			double sum = 0;
			for (size_t i = 0; i < nW; i++)
			{
				sum += sol[i];
			}
			for (size_t i = 0; sum > 0 && i < nW; i++)
			{
				sol[i] *= b[0] / sum;
			}
		}
	}
	std::copy(sol.begin(), sol.begin() + nW, loads.begin());

	if (suspension_.initialized())
	{
		// A wheel cannot pull the ground:
		const auto dN = suspension_.cornerLoadChanges();
		for (size_t i = 0; i < nW; i++)
		{
			loads[i] = std::max(0.0, loads[i] + dN[i]);
		}
	}
	return loads;
}

void VehicleBase::initSuspension()
{
	const auto& cfg = suspensionConfig_;
	if (!cfg.enabled)
	{
		return;
	}
	const size_t nW = getNumWheels();
	const double g = parent()->get_gravity();

	Suspension::Params p;
	p.sprungMass = chassis_mass_;
	p.com = chassis_com_;
	p.comHeight = cfg.cog_height.value_or(0.5 * (chassis_z_min_ + chassis_z_max_));
	p.gravity = g;

	// Default inertias: those of a box with the chassis footprint and height
	mrpt::math::TPoint2D bbMin;
	mrpt::math::TPoint2D bbMax;
	chassis_poly_.getBoundingBox(bbMin, bbMax);
	const double L = bbMax.x - bbMin.x;
	const double W = bbMax.y - bbMin.y;
	const double H = chassis_z_max_ - chassis_z_min_;
	p.I_xx = cfg.I_xx.value_or(chassis_mass_ * (W * W + H * H) / 12.0);
	p.I_yy = cfg.I_yy.value_or(chassis_mass_ * (L * L + H * H) / 12.0);

	// Static load of each corner, to size its spring and damper for the given
	// heave frequency and damping ratio (k_i = m_i*w^2, c_i = 2*zeta*w*m_i):
	std::vector<mrpt::math::TPoint2D> supports;
	for (size_t i = 0; i < nW; i++)
	{
		supports.emplace_back(wheels_info_[i].x, wheels_info_[i].y);
	}
	const double mg = chassis_mass_ * g;
	const double b[3] = {mg, mg * chassis_com_.x, mg * chassis_com_.y};
	std::vector<double> staticLoads;
	if (!solveSupportLoads(supports, b, staticLoads))
	{
		THROW_EXCEPTION_FMT(
			"[%s] <suspension> requires a vehicle that stands on its wheels (e.g. not a "
			"semi-trailer)",
			name_.c_str());
	}
	double sumLoads = 0;
	for (const double n : staticLoads)
	{
		sumLoads += n;
	}

	const double w = 2 * M_PI * cfg.frequency_hz;
	for (size_t i = 0; i < nW; i++)
	{
		const double m_i = sumLoads > 0 ? chassis_mass_ * staticLoads[i] / sumLoads : 0.0;
		Suspension::Corner c;
		c.pos = supports[i];
		c.stiffness = cfg.stiffness.count(i) ? cfg.stiffness.at(i) : m_i * w * w;
		c.damping = cfg.damping.count(i) ? cfg.damping.at(i) : 2 * cfg.damping_ratio * w * m_i;
		p.corners.push_back(c);
	}

	suspension_.init(p);
	world_->logFmt(
		mrpt::system::LVL_INFO, "[%s] Suspension modes: %s", name_.c_str(),
		suspension_.modesAsString().c_str());
}

void VehicleBase::updateSuspension(const TSimulContext& context)
{
	if (!suspension_.initialized())
	{
		return;
	}
	const double dt = context.dt;
	const auto q = getPose();

	// Velocity of the wheels' body at the suspension pivot (under the chassis
	// center of mass): along its normal, and roll and pitch rates
	Suspension::Vector3 baseRates = Suspension::Vector3::Zero();
	if (suspensionPrevBasePose_ && dt > 0)
	{
		const auto& q0 = *suspensionPrevBasePose_;
		const auto R = mrpt::poses::CPose3D(q).getRotationMatrix();
		const double dzNormal =
			R(0, 2) * (q.x - q0.x) + R(1, 2) * (q.y - q0.y) + R(2, 2) * (q.z - q0.z);
		// Ignore teleports (e.g. the vehicle moved by the user):
		constexpr double maxNormalStep = 0.25;	// [m]
		if (std::abs(dzNormal) < maxNormalStep)
		{
			const double rollRate = mrpt::math::wrapToPi(q.roll - q0.roll) / dt;
			const double pitchRate = mrpt::math::wrapToPi(q.pitch - q0.pitch) / dt;
			const auto& c = suspension_.params().com;
			baseRates[Suspension::HEAVE] = dzNormal / dt + c.y * rollRate - c.x * pitchRate;
			baseRates[Suspension::ROLL] = rollRate;
			baseRates[Suspension::PITCH] = pitchRate;
		}
	}
	suspensionPrevBasePose_ = q;

	// Acceleration in the vehicle frame. The tires cannot exceed ~1g, so larger
	// values are spikes (e.g. collisions, or the velocity steps of ideal
	// controllers) that would kick the chassis:
	const auto acc = getLinearAcceleration();
	mrpt::math::TVector2D accLocal(
		std::cos(q.yaw) * acc.x + std::sin(q.yaw) * acc.y,
		-std::sin(q.yaw) * acc.x + std::cos(q.yaw) * acc.y);
	const double maxAcc = 2.0 * parent()->get_gravity();
	if (const double n = accLocal.norm(); n > maxAcc)
	{
		accLocal *= maxAcc / n;
	}

	suspension_.step(dt, baseRates, accLocal);

	std::lock_guard<std::mutex> lck(chassisRelPoseMtx_);
	chassisRelPose_ = suspension_.relativePose();
}

std::optional<mrpt::poses::CPose3D> VehicleBase::chassisRelativePose() const
{
	std::lock_guard<std::mutex> lck(chassisRelPoseMtx_);
	return chassisRelPose_;
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
		if (auto& l = loggers_[LOGGER_IDX_WHEELS + i]; l->isActive())
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

	const double gravity = parent()->get_gravity();
	const double dt = context.dt;

	// Normal force on each wheel, and its share of the in-plane gravity force:
	wheelLoads_ = computeWheelLoads(context);
	const std::vector<double>& wheelLoads = wheelLoads_;
	double sumLoads = 0;
	for (const double n : wheelLoads)
	{
		sumLoads += n;
	}

	// The friction impulses are solved wheel by wheel (sequential impulses, a
	// Gauss-Seidel iteration), with the chassis velocity estimate including
	// the impulses of the rest of wheels: computing all of them from the same
	// velocity over-corrects it (and ends up in a limit cycle) when the wheels
	// are far from the center of mass compared to the yaw radius of gyration.
	// A few sweeps make the chassis and wheels reach a consistent solution
	// within the step. The order is reversed on every step to avoid a
	// left/right bias.
	const double M = b2dBody_->GetMass();
	const b2Vec2 com = b2dBody_->GetLocalCenter();
	const double Icom = b2dBody_->GetInertia() - M * (com.x * com.x + com.y * com.y);
	const bool sequentialImpulses = M > 0 && Icom > 0 && !idealControllerActive_;
	constexpr int numSweeps = 4;
	reverseWheelOrder_ = !reverseWheelOrder_;

	// Effective chassis mass at point r along direction n:
	const auto effectiveMass = [&](const mrpt::math::TVector2D& r, double nx, double ny)
	{
		const double rxn = r.x * ny - r.y * nx;
		return 1.0 / (1.0 / M + rxn * rxn / Icom);
	};
	// Adds the velocity change of the chassis reference point due to force F
	// (vehicle frame) at point r (from the COM) during dt, times "sign":
	const auto addImpulse = [&](mrpt::math::TTwist2D& v, const mrpt::math::TVector2D& r,
								const mrpt::math::TVector2D& F, double sign)
	{
		const double dOmega = sign * (r.x * F.y - r.y * F.x) * dt / Icom;
		// Velocity of the reference point (origin) = COM velocity + w x (-com)
		v.vx += sign * F.x * dt / M + dOmega * com.y;
		v.vy += sign * F.y * dt / M - dOmega * com.x;
		v.omega += dOmega;
	};

	// Inputs that do not change during the solve:
	std::vector<FrictionBase::TFrictionInput> inputs;
	inputs.reserve(nW);
	std::vector<double> wheelSpin0(nW);
	std::vector<mrpt::math::TVector2D> rFromCom(nW);
	for (size_t i = 0; i < nW; i++)
	{
		Wheel& w = getWheelInfo(i);
		auto& fi = inputs.emplace_back(context, w);
		wheelSpin0[i] = w.getW();
		rFromCom[i] = {w.x - com.x, w.y - com.y};

		fi.motorTorque = -wheelTorque[i];  // "-" => Forwards is negative
		if (i < wheelMotors_.size())
		{
			fi.motor = wheelMotors_[i];
		}
		// Fz excludes the wheel own weight:
		fi.Fz = wheelLoads[i] - w.mass * gravity;
		if (sequentialImpulses)
		{
			const double c = std::cos(w.yaw);
			const double sn = std::sin(w.yaw);
			fi.longitudinalMass = effectiveMass(rFromCom[i], c, sn);
			fi.lateralMass = effectiveMass(rFromCom[i], -sn, c);
		}

		// Share of the gravity force along the ground plane (vehicle frame):
		const double share = sumLoads > 0 ? wheelLoads[i] / sumLoads : 1.0 / nW;
		fi.gravSlopeForce = {slopeDir_.x * M * gravity * share, slopeDir_.y * M * gravity * share};

		// Query per-region friction overrides at the wheel's world position:
		const b2Vec2 wPtQuery = b2dBody_->GetWorldPoint(b2Vec2(w.x, w.y));
		const mrpt::math::TPoint3D wheelWorldPos(wPtQuery.x, wPtQuery.y, 0);
		if (auto prop = world_->getPropertyAt("friction_mu", wheelWorldPos))
		{
			if (auto* val = std::any_cast<double>(&*prop))
			{
				fi.mu_override = *val;
			}
		}
		if (auto prop = world_->getPropertyAt("friction_C_rr", wheelWorldPos))
		{
			if (auto* val = std::any_cast<double>(&*prop))
			{
				fi.C_rr_override = *val;
			}
		}
	}

	// Solve: friction forces (vehicle frame) at each wheel
	std::vector<mrpt::math::TVector2D> frictionForces(nW, mrpt::math::TVector2D(0, 0));
	mrpt::math::TTwist2D vel = getRefVelocityLocal();  // body frame
	for (int sweep = 0; sweep < (sequentialImpulses ? numSweeps : 1); sweep++)
	{
		for (size_t k = 0; k < nW; k++)
		{
			const size_t i = reverseWheelOrder_ ? nW - 1 - k : k;
			auto& fi = inputs[i];
			Wheel& w = fi.wheel;

			// Solve this wheel against the impulses of the rest of them:
			const mrpt::math::TVector2D netForce(
				frictionForces[i].x + fi.gravSlopeForce.x,
				frictionForces[i].y + fi.gravSlopeForce.y);
			if (sequentialImpulses && sweep > 0)
			{
				addImpulse(vel, rFromCom[i], netForce, -1.0);
			}
			w.setW(wheelSpin0[i]);
			fi.wheelCogLocalVel = {vel.vx - vel.omega * w.y, vel.vy + vel.omega * w.x};

			// eval friction (in the frame of the vehicle):
			frictionForces[i] = frictions_.at(i)->evaluate_friction(fi);

			if (sequentialImpulses)
			{
				addImpulse(
					vel, rFromCom[i],
					{frictionForces[i].x + fi.gravSlopeForce.x,
					 frictionForces[i].y + fi.gravSlopeForce.y},
					+1.0);
			}
		}
	}

	// For visualization only
	std::vector<mrpt::math::TSegment3D> forceVectors, torqueVectors;

	for (size_t i = 0; i < nW; i++)
	{
		const auto& fi = inputs[i];
		const Wheel& w = fi.wheel;
		const mrpt::math::TVector2D& F_r = frictionForces[i];

		// Force vector -> world coords
		const b2Vec2 wForce = b2dBody_->GetWorldVector(b2Vec2(F_r.x, F_r.y));
		// Application point -> world coords
		const b2Vec2 wPt = b2dBody_->GetWorldPoint(b2Vec2(w.x, w.y));

		// Apply force to the chassis (skip for ideal controllers that
		// directly impose the vehicle twist , friction reaction forces would
		// corrupt the kinematically-imposed trajectory):
		if (!idealControllerActive_)
		{
			b2dBody_->ApplyForce(wForce, wPt, true /*wake up*/);

			// Apply the gravity slope force at this wheel's contact point.
			// The friction model already counteracts this force in its output,
			// so the net result is zero when the vehicle is held stationary by
			// friction, or a residual downhill force when friction is exceeded.
			const b2Vec2 wGravForce =
				b2dBody_->GetWorldVector(b2Vec2(fi.gravSlopeForce.x, fi.gravSlopeForce.y));
			b2dBody_->ApplyForce(wGravForce, wPt, true /*wake up*/);
		}

		// log
		if (auto& l = loggers_[LOGGER_IDX_WHEELS + i]; l->isActive())
		{
			auto& logger = *l;

			logger.updateColumn(DL_TIMESTAMP, context.simul_time);
			logger.updateColumn("wheel_pos_x", w.x);
			logger.updateColumn("wheel_pos_y", w.y);

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
	// Integrate wheels' rotation (before the controllers, which may report it):
	for (size_t i = 0; i < getNumWheels(); i++)
	{
		getWheelInfo(i).integrateSpin(context.dt);
	}

	invoke_motor_controllers_post_step(context);

	// Common part (update q_, dq_)
	Simulable::simul_post_timestep(context);

	// Before the sensors, which are mounted on the chassis:
	updateSuspension(context);

	for (auto& s : sensors_)
	{
		const double t0 = mrpt::Clock::nowDouble();
		s->simul_post_timestep(context);
		if (world_)
		{
			world_->internalAddSensorProcessingTime(
				getName() + "/" + s->getName(), mrpt::Clock::nowDouble() - t0,
				true /*simulation thread*/);
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
	const auto dq = getRefVelocityLocal();

	if (auto& l = loggers_[LOGGER_IDX_POSE]; l->isActive())
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
		logger.updateColumn(PL_DQ_W, dq.omega);
		logger.updateColumn(PL_ODO_X, odometry_.x());
		logger.updateColumn(PL_ODO_Y, odometry_.y());
		logger.updateColumn(PL_ODO_YAW, odometry_.phi());
		if (suspension_.initialized())
		{
			const auto& d = suspension_.displacement();
			logger.updateColumn(PL_SUSP_Z, d[Suspension::HEAVE]);
			logger.updateColumn(PL_SUSP_ROLL, d[Suspension::ROLL]);
			logger.updateColumn(PL_SUSP_PITCH, d[Suspension::PITCH]);
		}
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
	const mrpt::optional_ref<mrpt::viz::Scene>& viz,
	const mrpt::optional_ref<mrpt::viz::Scene>& physical)
{
	for (auto& s : sensors_)
	{
		s->guiUpdate(viz, physical);
	}
}

void VehicleBase::internal_internalGuiUpdate_forces(  //
	[[maybe_unused]] mrpt::viz::Scene& scene)
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

void VehicleBase::internal_internalGuiUpdate_trajectory()
{
	if (glTrajectory_)
	{
		glTrajectory_->setVisibility(world_->guiOptions_.show_trajectories);
	}
}

double mvsim::VehicleBase::estimateSlopeTorquePerWheel(size_t nDrivenWheels) const
{
	if (nDrivenWheels == 0)
	{
		nDrivenWheels = getNumWheels();
	}
	if (nDrivenWheels == 0)
	{
		return 0;
	}

	// Vehicle pitch from 3D pose:
	const auto [yaw, pitch, roll] = getCPose3D().getYawPitchRoll();

	const double g = parent()->get_gravity();
	const double m = getChassisMass();
	// Average wheel radius:
	double R = 0;
	for (size_t i = 0; i < getNumWheels(); i++)
	{
		R += getWheelInfo(i).diameter;
	}
	R *= 0.5 / static_cast<double>(getNumWheels());

	// Torque per driven wheel to hold the vehicle on the slope:
	// F_grade = m * g * sin(pitch), shared across driven wheels,
	// converted to torque: tau = F * R / nDrivenWheels
	return m * g * std::sin(pitch) * R / static_cast<double>(nDrivenWheels);
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
	bodyDef.linearDamping = linear_damping_;
	bodyDef.angularDamping = angular_damping_;

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
		{
			pts[i] = b2Vec2(chassis_poly_[i].x, chassis_poly_[i].y);
		}

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
	const mrpt::optional_ref<mrpt::viz::Scene>& viz,
	const mrpt::optional_ref<mrpt::viz::Scene>& physical, bool childrenOnly)
{
	// 1st time call?? -> Create objects
	// ----------------------------------
	const size_t nWs = this->getNumWheels();
	if (!glChassisViz_ && viz && physical)
	{
		glChassisViz_ = mrpt::viz::CSetOfObjects::Create();
		glChassisViz_->setName("vehicle_chassis_"s + name_);

		glChassisPhysical_ = mrpt::viz::CSetOfObjects::Create();
		glChassisPhysical_->setName("vehicle_chassis_"s + name_);

		// Wheels shape:
		glWheelsViz_.resize(nWs);
		glWheelsPhysical_.resize(nWs);
		for (size_t i = 0; i < nWs; i++)
		{
			glWheelsViz_[i] = mrpt::viz::CSetOfObjects::Create();
			this->getWheelInfo(i).getAs3DObject(*glWheelsViz_[i], false);
			// When a custom visual exists the wheel viz objects are parented
			// under glCustomVisual_ instead of glChassisViz_.  This way the
			// single glCustomVisual_->setPose() call in VisualObject::guiUpdate
			// moves chassis body and all wheels atomically, eliminating the
			// GUI-thread race that caused wheels to visually jump ahead/behind
			// the chassis at high speed.
			if (childrenOnly)
			{
				glCustomVisual_->insert(glWheelsViz_[i]);
			}
			else
			{
				glChassisViz_->insert(glWheelsViz_[i]);
			}

			glWheelsPhysical_[i] = mrpt::viz::CSetOfObjects::Create();
			this->getWheelInfo(i).getAs3DObject(*glWheelsPhysical_[i], true);
			glChassisPhysical_->insert(glWheelsPhysical_[i]);
		}

		if (!childrenOnly)
		{
			// Robot shape: the chassis prism, with a darker band at its bottom
			// (like a bumper) to give it some depth:
			const double chassisH = chassis_z_max_ - chassis_z_min_;
			const double bumperH = 0.2 * chassisH;

			auto gl_bumper = mrpt::viz::CPolyhedron::CreateCustomPrism(chassis_poly_, bumperH);
			gl_bumper->setLocation(0, 0, chassis_z_min_);
			gl_bumper->setColor_u8(mrpt::img::TColor(
				static_cast<uint8_t>(chassis_color_.R * 0.55),
				static_cast<uint8_t>(chassis_color_.G * 0.55),
				static_cast<uint8_t>(chassis_color_.B * 0.55), chassis_color_.A));
			glChassisViz_->insert(gl_bumper);
			glChassisPhysical_->insert(gl_bumper);

			auto gl_poly =
				mrpt::viz::CPolyhedron::CreateCustomPrism(chassis_poly_, chassisH - bumperH);
			gl_poly->setLocation(0, 0, chassis_z_min_ + bumperH);
			gl_poly->setColor_u8(chassis_color_);
			glChassisViz_->insert(gl_poly);
			glChassisPhysical_->insert(gl_poly);
		}

		// Only add glChassisViz_ to the scene when it actually has content
		// (i.e. no custom visual).  When childrenOnly=true, glChassisViz_
		// would be an empty, unused container.
		if (!childrenOnly)
		{
			insertIntoScene(viz->get(), glChassisViz_);
		}
		insertIntoScene(physical->get(), glChassisPhysical_);

		glInit_ = true;
	}

	// Update them:
	// ----------------------------------
	// If "viz" does not have a value, it's because we are already inside a
	// setPose() change event, so my caller already holds the mutex and we don't
	// need/can't acquire it again:
	const auto objectPose = viz.has_value() ? getChassisPose() : getChassisPoseNoLock();
	const auto pp = parent()->applyWorldRenderOffset(objectPose);

	// The wheels are children of the chassis visuals, but they do not move
	// with its suspension:
	const auto chassisRel = chassisRelativePose();
	const auto onWheelsBody = [&](const mrpt::math::TPose3D& p)
	{ return chassisRel ? (mrpt::poses::CPose3D(p) - *chassisRel).asTPose() : p; };

	if (glInit_)
	{
		// When a custom visual exists, glCustomVisual_ is already updated by
		// VisualObject::guiUpdate() and the wheels are children of it, so no
		// separate chassis viz pose update is needed.
		if (!childrenOnly)
		{
			glChassisViz_->setPose(pp);
		}
		glChassisPhysical_->setPose(pp);
		for (size_t i = 0; i < nWs; i++)
		{
			const Wheel& w = getWheelInfo(i);
			const auto wheelPose = onWheelsBody(
				mrpt::math::TPose3D(w.x, w.y, 0.5 * w.diameter, w.yaw, w.getPhi(), 0.0));
			glWheelsPhysical_[i]->setPose(wheelPose);
			glWheelsViz_[i]->setPose(wheelPose);

			if (!w.linked_yaw_object_name.empty())
			{
				auto glLinked = CVisualObject::glCustomVisual_->getByName(w.linked_yaw_object_name);
				if (!glLinked)
				{
					THROW_EXCEPTION_FMT(
						"Wheel #%zu has linked_yaw_object_name='%s' but parent "
						"vehicle '%s' does not have any custom visual group "
						"with that name.",
						i, w.linked_yaw_object_name.c_str(), name_.c_str());
				}
				if (!linkedObjectsPose0_.count(w.linked_yaw_object_name))
				{
					linkedObjectsPose0_[w.linked_yaw_object_name] = glLinked->getPose();
				}
				auto p = linkedObjectsPose0_.at(w.linked_yaw_object_name);
				p.yaw = w.yaw + w.linked_yaw_offset;
				glLinked->setPose(onWheelsBody(p));
			}
		}
	}

	// Init on first use:
	if (!glForces_ && viz)
	{
		// Visualization of forces:
		glForces_ = mrpt::viz::CSetOfLines::Create();
		glForces_->setLineWidth(3.0);
		glForces_->setColor_u8(0xff, 0xff, 0xff);
		glForces_->setPose(parent()->applyWorldRenderOffset(mrpt::poses::CPose3D::Identity()));

		insertIntoScene(viz->get(), glForces_);	 // forces are in global coords
	}
	if (!glMotorTorques_ && viz)
	{
		// Visualization of forces:
		glMotorTorques_ = mrpt::viz::CSetOfLines::Create();
		glMotorTorques_->setLineWidth(3.0);
		glMotorTorques_->setColor_u8(0xff, 0x00, 0x00);
		glMotorTorques_->setPose(
			parent()->applyWorldRenderOffset(mrpt::poses::CPose3D::Identity()));
		insertIntoScene(viz->get(), glMotorTorques_);  // torques are in global coords
	}

	if (!glTrajectory_ && viz)
	{
		// Visualization of the exact reproducible trajectory (if any):
		std::vector<mrpt::math::TPoint2D> trajPts;
		double trajHeight = 0.5;
		if (auto* ctrl = getControllerInterface();
			ctrl && ctrl->getTrajectoryPlotPoints(trajPts, trajHeight) && trajPts.size() >= 2)
		{
			glTrajectory_ = mrpt::viz::CSetOfLines::Create();
			glTrajectory_->setLineWidth(2.5);
			glTrajectory_->setColor_u8(0x00, 0xff, 0xff);
			glTrajectory_->setPose(
				parent()->applyWorldRenderOffset(mrpt::poses::CPose3D::Identity()));
			for (size_t i = 0; i + 1 < trajPts.size(); i++)
			{
				glTrajectory_->appendLine(
					trajPts[i].x, trajPts[i].y, trajHeight, trajPts[i + 1].x, trajPts[i + 1].y,
					trajHeight);
			}
			glTrajectory_->setVisibility(world_->guiOptions_.show_trajectories);
			insertIntoScene(viz->get(), glTrajectory_);	 // trajectory is in global coords
		}
	}

	// Other common stuff:
	if (viz && physical)
	{
		internal_internalGuiUpdate_sensors(viz->get(), physical->get());
		internal_internalGuiUpdate_forces(viz->get());
		internal_internalGuiUpdate_trajectory();
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
		if (!logger->isActive())
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
