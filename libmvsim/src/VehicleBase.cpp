/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mvsim/World.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/FrictionModels/DefaultFriction.h> // For use as default model

#include "JointXMLnode.h"
#include "XMLClassesRegistry.h"
#include "xml_utils.h"

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include <rapidxml_print.hpp>
#include <mrpt/utils/utils_defs.h>  // mrpt::format()
#include <mrpt/poses/CPose2D.h>
#include <mrpt/opengl/CPolyhedron.h>

#include <sstream>      // std::stringstream
#include <map>
#include <string>

using namespace mvsim;
using namespace std;

XmlClassesRegistry veh_classes_registry("vehicle:class");

TClassFactory_vehicleDynamics mvsim::classFactory_vehicleDynamics;


// Explicit registration calls seem to be one (the unique?) way to assure registration takes place:
void register_all_veh_dynamics()
{
	static bool done = false;
	if (done) return; else done=true;

	REGISTER_VEHICLE_DYNAMICS("differential",DynamicsDifferential)
	REGISTER_VEHICLE_DYNAMICS("ackermann",DynamicsAckermann)
}

// Protected ctor:
VehicleBase::VehicleBase(World *parent, size_t nWheels) :
	VisualObject(parent),
	m_vehicle_index(0),
	m_b2d_vehicle_body(NULL),
	m_q(0,0,0,0,0,0),
	m_dq(0,0,0),
	m_chassis_mass(15.0),
	m_chassis_z_min(0.05),
	m_chassis_z_max(0.6),
	m_chassis_color(0xff,0x00,0x00),
	m_chassis_com(.0,.0),
	m_wheels_info(nWheels),
	m_fixture_wheels(nWheels, NULL)
{
	using namespace mrpt::math;
	// Default shape:
	m_chassis_poly.push_back( TPoint2D(-0.4, -0.5) );
	m_chassis_poly.push_back( TPoint2D(-0.4,  0.5) );
	m_chassis_poly.push_back( TPoint2D( 0.4,  0.5) );
	m_chassis_poly.push_back( TPoint2D( 0.6,  0.3) );
	m_chassis_poly.push_back( TPoint2D( 0.6, -0.3) );
	m_chassis_poly.push_back( TPoint2D( 0.4, -0.5) );
	updateMaxRadiusFromPoly();

}

void VehicleBase::simul_post_timestep_common(const TSimulContext &context)
{
	if (m_b2d_vehicle_body)
	{
		// Pos:
		const b2Vec2 &pos = m_b2d_vehicle_body->GetPosition();
		const float32 angle = m_b2d_vehicle_body->GetAngle();
		m_q.x=pos(0);
		m_q.y=pos(1);
		m_q.yaw=angle;
		// The rest (z,pitch,roll) will be always 0, unless other world-element modifies them! (e.g. elevation map)

		// Vel:
		const b2Vec2 &vel = m_b2d_vehicle_body->GetLinearVelocity();
		const float32 w = m_b2d_vehicle_body->GetAngularVelocity();
		m_dq.vals[0]=vel(0);
		m_dq.vals[1]=vel(1);
		m_dq.vals[2]=w;
	}
}


/** Register a new class of vehicles from XML description of type "<vehicle:class name='name'>...</vehicle:class>".  */
void VehicleBase::register_vehicle_class(const rapidxml::xml_node<char> *xml_node)
{
	// Sanity checks:
	if (!xml_node) throw runtime_error("[VehicleBase::register_vehicle_class] XML node is NULL");
	if (0!=strcmp(xml_node->name(),"vehicle:class")) throw runtime_error(mrpt::format("[VehicleBase::register_vehicle_class] XML element is '%s' ('vehicle:class' expected)",xml_node->name()));

	// rapidxml doesn't allow making copied of objects.
	// So: convert to txt; then re-parse.
	std::stringstream ss;
	ss << *xml_node;

	veh_classes_registry.add( ss.str() );
}


/** Class factory: Creates a vehicle from XML description of type "<vehicle>...</vehicle>".  */
VehicleBase* VehicleBase::factory(World* parent, const rapidxml::xml_node<char> *root)
{
	register_all_veh_dynamics();

	using namespace std;
	using namespace rapidxml;

	if (!root) throw runtime_error("[VehicleBase::factory] XML node is NULL");
	if (0!=strcmp(root->name(),"vehicle")) throw runtime_error(mrpt::format("[VehicleBase::factory] XML root element is '%s' ('vehicle' expected)",root->name()));

	// "class": When a vehicle has a 'class="XXX"' attribute, look for each parameter
	//  in the set of "root" + "class_root" XML nodes:
	// --------------------------------------------------------------------------------
	JointXMLnode<> veh_root_node;
	{
		veh_root_node.add(root); // Always search in root. Also in the class root, if any:

		const xml_attribute<> *veh_class = root->first_attribute("class");
		if (veh_class)
		{
			const string sClassName = veh_class->value();
			const rapidxml::xml_node<char>* class_root= veh_classes_registry.get(sClassName);
			if (!class_root)
				throw runtime_error(mrpt::format("[VehicleBase::factory] Vehicle class '%s' undefined",sClassName.c_str() ));

			veh_root_node.add(class_root);
			//cout << *class_root;
		}
	}

	// Class factory according to: <dynamics class="XXX">
	// -------------------------------------------------
	const xml_node<> *dyn_node = veh_root_node.first_node("dynamics");
	if (!dyn_node) throw runtime_error("[VehicleBase::factory] Missing XML node <dynamics>");

	const xml_attribute<> *dyn_class = dyn_node->first_attribute("class");
	if (!dyn_class || !dyn_class->value() ) throw runtime_error("[VehicleBase::factory] Missing mandatory attribute 'class' in node <dynamics>");

	VehicleBase *veh = classFactory_vehicleDynamics.create(dyn_class->value(),parent);
	if (!veh)
		throw runtime_error(mrpt::format("[VehicleBase::factory] Unknown vehicle dynamics class '%s'",dyn_class->value()));

	// Initialize here all common params shared by any polymorphic class:
	// -------------------------------------------------
	// attrib: name
	{
		const xml_attribute<> *attrib_name = root->first_attribute("name");
		if (attrib_name && attrib_name->value())
		{
			veh->m_name = attrib_name->value();
		}
		else
		{
			// Default name:
			static int cnt =0;
			veh->m_name = mrpt::format("veh%i",++cnt);
		}
	}


	// (Mandatory) initial pose:
	{
		const xml_node<> *node = veh_root_node.first_node("init_pose");
		if (!node) throw runtime_error("[VehicleBase::factory] Missing XML node <init_pose>");

		if (3!= ::sscanf(node->value(),"%lf %lf %lf",&veh->m_q.x,&veh->m_q.y,&veh->m_q.yaw))
			throw runtime_error("[VehicleBase::factory] Error parsing <init_pose>...</init_pose>");
		veh->m_q.yaw *= M_PI/180.0; // deg->rad
	}

	// (Optional) initial vel:
	{
		const xml_node<> *node = veh_root_node.first_node("init_vel");
		if (node)
		{
			if (3!= ::sscanf(node->value(),"%lf %lf %lf",&veh->m_dq.vals[0],&veh->m_dq.vals[1],&veh->m_dq.vals[2]))
				throw runtime_error("[VehicleBase::factory] Error parsing <init_vel>...</init_vel>");
			veh->m_dq.vals[2] *= M_PI/180.0; // deg->rad

			// Convert twist (velocity) from local -> global coords:
			const mrpt::poses::CPose2D pose(0,0,veh->m_q.yaw); // Only the rotation
			pose.composePoint(
				veh->m_dq.vals[0], veh->m_dq.vals[1],
				veh->m_dq.vals[0], veh->m_dq.vals[1] );
		}
	}

	// Initialize class-specific params:
	// -------------------------------------------------
	veh->dynamics_load_params_from_xml(dyn_node);

	// Register bodies, fixtures, etc. in Box2D simulator:
	// ----------------------------------------------------
	b2World* b2world = parent->getBox2DWorld();
	veh->create_multibody_system(b2world);

	if (veh->m_b2d_vehicle_body)
	{
		// Init pos:
		veh->m_b2d_vehicle_body->SetTransform( b2Vec2( veh->m_q.x, veh->m_q.y ),  veh->m_q.yaw );
		// Init vel:
		veh->m_b2d_vehicle_body->SetLinearVelocity( b2Vec2(veh->m_dq.vals[0], veh->m_dq.vals[1] ) );
		veh->m_b2d_vehicle_body->SetAngularVelocity(veh->m_dq.vals[2] );
	}


	// Friction model:
	// Parse <friction> node, or assume default linear model:
	// -----------------------------------------------------------
	{
		const xml_node<> *frict_node = veh_root_node.first_node("friction");
		if (!frict_node)
		{
			// Default:
			veh->m_friction = std::shared_ptr<FrictionBase>( new DefaultFriction(*veh, NULL /*default params*/) );
		}
		else
		{
			// Parse:
			veh->m_friction = std::shared_ptr<FrictionBase>(  FrictionBase::factory(*veh,frict_node) );
			ASSERT_(veh->m_friction)
		}
	}



	// Sensors: <sensor class='XXX'> entries
	// -------------------------------------------------
	for (JointXMLnode<>::iterator it=veh_root_node.begin(); it!=veh_root_node.end();++it)
	{
		if (!strcmp(it->name(),"sensor"))
		{
			SensorBase *se = SensorBase::factory(*veh,*it);
      veh->m_sensors.push_back( SensorBase::Ptr(se));
		}
	}

	return veh;
}

VehicleBase* VehicleBase::factory(World* parent, const std::string &xml_text)
{
	// Parse the string as if it was an XML file:
	std::stringstream s;
	s.str( xml_text );

	char* input_str = const_cast<char*>(xml_text.c_str());
	rapidxml::xml_document<> xml;
	try {
		xml.parse<0>(input_str);
	}
	catch (rapidxml::parse_error &e) {
		unsigned int line = static_cast<long>(std::count(input_str, e.where<char>(), '\n') + 1);
		throw std::runtime_error( mrpt::format("[VehicleBase::factory] XML parse error (Line %u): %s", static_cast<unsigned>(line), e.what() ) );
	}
	return VehicleBase::factory(parent,xml.first_node());
}

void VehicleBase::simul_pre_timestep(const TSimulContext &context)
{
	// Update wheels position (they may turn, etc. as in an Ackermann configuration)
	for (size_t i=0;i<m_fixture_wheels.size();i++)
	{
		b2PolygonShape *wheelShape = dynamic_cast<b2PolygonShape*>( m_fixture_wheels[i]->GetShape() );
		wheelShape->SetAsBox(
			m_wheels_info[i].diameter*0.5, m_wheels_info[i].width*0.5,
			b2Vec2(m_wheels_info[i].x,m_wheels_info[i].y), m_wheels_info[i].yaw );
	}


	// Apply motor forces/torques:
	this->invoke_motor_controllers(context,m_torque_per_wheel);

	// Apply friction model at each wheel:
	const size_t nW = getNumWheels();
	ASSERT_EQUAL_(m_torque_per_wheel.size(),nW);

	const double gravity = getWorldObject()->get_gravity();
	const double massPerWheel = getChassisMass()/nW; // Part of the vehicle weight on each wheel.
	const double weightPerWheel = massPerWheel* gravity;

	std::vector<mrpt::math::TPoint2D> wheels_vels;
	getWheelsVelocityLocal(wheels_vels,getVelocityLocal());

	ASSERT_EQUAL_(wheels_vels.size(),nW);

	std::vector<mrpt::math::TSegment3D> force_vectors; // For visualization only

	for (size_t i=0;i<nW;i++)
	{
		// prepare data:
		Wheel &w = getWheelInfo(i);

		FrictionBase::TFrictionInput fi(context,w);
		fi.motor_torque = -m_torque_per_wheel[i];  // "-" => Forwards is negative
		fi.weight = weightPerWheel;
		fi.wheel_speed = wheels_vels[i];

		// eval friction:
		mrpt::math::TPoint2D net_force_;
		m_friction->evaluate_friction(fi,net_force_);

		// Apply force:
		const b2Vec2 wForce = m_b2d_vehicle_body->GetWorldVector(b2Vec2(net_force_.x,net_force_.y)); // Force vector -> world coords
		const b2Vec2 wPt    = m_b2d_vehicle_body->GetWorldPoint( b2Vec2( w.x, w.y) ); // Application point -> world coords
		//printf("w%i: Lx=%6.3f Ly=%6.3f  | Gx=%11.9f Gy=%11.9f\n",(int)i,net_force_.x,net_force_.y,wForce.x,wForce.y);

		m_b2d_vehicle_body->ApplyForce( wForce,wPt, true/*wake up*/);

		// save it for optional rendering:
		if (m_world->m_gui_options.show_forces)
		{
			const double forceScale =  m_world->m_gui_options.force_scale; // [meters/N]
			const mrpt::math::TPoint3D pt1(wPt.x,wPt.y, m_chassis_z_max*1.1 + m_q.z );
			const mrpt::math::TPoint3D pt2 = pt1 + mrpt::math::TPoint3D(wForce.x,wForce.y, 0)*forceScale;
			force_vectors.push_back( mrpt::math::TSegment3D( pt1,pt2 ));
		}
	}

	// Save forces for optional rendering:
	if (m_world->m_gui_options.show_forces)
	{
    std::lock_guard<std::mutex> csl( m_force_segments_for_rendering_cs );
		m_force_segments_for_rendering = force_vectors;
	}
}

/** Override to do any required process right after the integration of dynamic equations for each timestep */
void VehicleBase::simul_post_timestep(const TSimulContext &context)
{
	// Integrate wheels' rotation:
	const size_t nW = getNumWheels();

	for (size_t i=0;i<nW;i++)
	{
		// prepare data:
		Wheel &w = getWheelInfo(i);

		// Explicit Euler:
		w.setPhi( w.getPhi() + w.getW() * context.dt);

		// Wrap wheel spin position (angle), so it doesn't 
		// become excessively large (it's actually unbound, but we don't want to lose 'double' accuracy):
		const double cur_abs_phi = std::abs(w.getPhi());
		if (cur_abs_phi>1e4)
			w.setPhi( ::fmod(cur_abs_phi, 2*M_PI) * (w.getPhi()<0.0 ? -1.0 : 1.0) );
	}
}


/** Last time-step velocity of each wheel's center point (in local coords) */
void VehicleBase::getWheelsVelocityLocal(std::vector<mrpt::math::TPoint2D> &vels, const vec3 &veh_vel_local ) const
{
	// Each wheel velocity is:
	// v_w = v_veh + \omega \times wheel_pos
	// =>
	// v_w = v_veh + ( -w*y, w*x )

	const double w = veh_vel_local.vals[2]; // vehicle w

	const size_t nW = this->getNumWheels();
	vels.resize(nW);
	for (size_t i=0;i<nW;i++)
	{
		const Wheel &wheel = getWheelInfo(i);

		vels[i].x = veh_vel_local.vals[0] - w * wheel.y;
		vels[i].y = veh_vel_local.vals[1] + w * wheel.x;
	}
}


/** Last time-step velocity (of the ref. point, in local coords) */
vec3 VehicleBase::getVelocityLocal() const
{
	vec3 local_vel;
	local_vel.vals[2] = m_dq.vals[2]; // omega remains the same.

	const mrpt::poses::CPose2D p(0,0, -m_q.yaw); // "-" means inverse pose
	p.composePoint(
		m_dq.vals[0],m_dq.vals[1],
		local_vel.vals[0],local_vel.vals[1]);
	return local_vel;
}

mrpt::poses::CPose2D VehicleBase::getCPose2D() const
{
	return mrpt::poses::CPose2D( mrpt::math::TPose2D(m_q));
}

/** To be called at derived classes' gui_update() */
void VehicleBase::gui_update_common( mrpt::opengl::COpenGLScene &scene, bool defaultVehicleBody )
{
	// 1st time call?? -> Create objects
	// ----------------------------------
	if (defaultVehicleBody)
	{
		const size_t nWs = this->getNumWheels();
		if (!m_gl_chassis)
		{
      m_gl_chassis = mrpt::make_aligned_shared<mrpt::opengl::CSetOfObjects>();

			// Wheels shape:
			m_gl_wheels.resize(nWs);
			for (size_t i=0;i<nWs;i++)
			{
        m_gl_wheels[i]= mrpt::make_aligned_shared<mrpt::opengl::CSetOfObjects>();
				this->getWheelInfo(i).getAs3DObject(*m_gl_wheels[i]);
				m_gl_chassis->insert(m_gl_wheels[i]);
			}
			// Robot shape:
      mrpt::opengl::CPolyhedron::Ptr gl_poly = mrpt::opengl::CPolyhedron::CreateCustomPrism( m_chassis_poly, m_chassis_z_max-m_chassis_z_min);
			gl_poly->setLocation(0,0, m_chassis_z_min);
			gl_poly->setColor( mrpt::utils::TColorf(m_chassis_color) );
			m_gl_chassis->insert(gl_poly);

			SCENE_INSERT_Z_ORDER(scene,1, m_gl_chassis);

			// Visualization of forces:
      m_gl_forces = mrpt::make_aligned_shared<mrpt::opengl::CSetOfLines>();
			m_gl_forces->setLineWidth(3.0);
			m_gl_forces->setColor_u8(mrpt::utils::TColor(0xff,0xff,0xff));

			SCENE_INSERT_Z_ORDER(scene,3, m_gl_forces);  // forces are in global coords

		}


		// Update them:
		// ----------------------------------
		m_gl_chassis->setPose(m_q);

		for (size_t i=0;i<nWs;i++)
		{
			const Wheel & w = getWheelInfo(i);
			m_gl_wheels[i]->setPose( mrpt::math::TPose3D( w.x,w.y, 0.5*w.diameter, w.yaw, w.getPhi(), 0.0) );
		}
	}


	// Other common stuff:
	internal_gui_update_sensors(scene);
	internal_gui_update_forces(scene);
}


void VehicleBase::internal_gui_update_sensors( mrpt::opengl::COpenGLScene &scene)
{
	for (TListSensors::iterator it=m_sensors.begin();it!=m_sensors.end();++it)
		(*it)->gui_update(scene);
}

void VehicleBase::internal_gui_update_forces( mrpt::opengl::COpenGLScene &scene)
{
	if (m_world->m_gui_options.show_forces)
	{
    std::lock_guard<std::mutex> csl( m_force_segments_for_rendering_cs );
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

	m_max_radius=0.001f;
	for (TPolygon2D::const_iterator it=m_chassis_poly.begin();it!=m_chassis_poly.end();++it)
	{
		const float n=it->norm();
		mrpt::utils::keep_max(m_max_radius,n);
	}
}

/** Create bodies, fixtures, etc. for the dynamical simulation */
void VehicleBase::create_multibody_system(b2World* world)
{
	// Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;

	m_b2d_vehicle_body = world->CreateBody(&bodyDef);

	// Define shape of chassis:
	// ------------------------------
	{
		// Convert shape into Box2D format:
		const size_t nPts = m_chassis_poly.size();
		ASSERT_(nPts>=3)
		ASSERT_BELOWEQ_(nPts, (size_t)b2_maxPolygonVertices)
		std::vector<b2Vec2> pts(nPts);
		for (size_t i=0;i<nPts;i++) pts[i] = b2Vec2( m_chassis_poly[i].x, m_chassis_poly[i].y );

		b2PolygonShape chassisPoly;
		chassisPoly.Set(&pts[0],nPts);
		//chassisPoly.m_radius = 1e-3;  // The "skin" depth of the body

		// Define the dynamic body fixture.
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &chassisPoly;
		fixtureDef.restitution = 0.01;

		// Set the box density to be non-zero, so it will be dynamic.
		b2MassData mass;
		chassisPoly.ComputeMass(&mass,1);  // Mass with density=1 => compute area
		fixtureDef.density = m_chassis_mass / mass.mass;

		// Override the default friction.
		fixtureDef.friction = 0.3f;

		// Add the shape to the body.
		m_fixture_chassis = m_b2d_vehicle_body->CreateFixture(&fixtureDef);

		// Compute center of mass:
		b2MassData vehMass;
		m_fixture_chassis->GetMassData(&vehMass);
		m_chassis_com.x = vehMass.center.x;
		m_chassis_com.y = vehMass.center.y;
	}

	// Define shape of wheels:
	// ------------------------------
	ASSERT_EQUAL_(m_fixture_wheels.size(),m_wheels_info.size())

	for (size_t i=0;i<m_wheels_info.size();i++)
	{
		b2PolygonShape wheelShape;
		wheelShape.SetAsBox(
			m_wheels_info[i].diameter*0.5, m_wheels_info[i].width*0.5,
			b2Vec2(m_wheels_info[i].x,m_wheels_info[i].y), m_wheels_info[i].yaw );

		// Define the dynamic body fixture.
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &wheelShape;
		fixtureDef.restitution = 0.05;

		// Set the box density to be non-zero, so it will be dynamic.
		b2MassData mass;
		wheelShape.ComputeMass(&mass,1);  // Mass with density=1 => compute area
		fixtureDef.density = m_wheels_info[i].mass / mass.mass;

		// Override the default friction.
		fixtureDef.friction = 0.5f;

		m_fixture_wheels[i] = m_b2d_vehicle_body->CreateFixture(&fixtureDef);
	}
}

void VehicleBase::gui_update( mrpt::opengl::COpenGLScene &scene)
{
	this->gui_update_common(scene); // Common part: update sensors, etc.
}

void VehicleBase::apply_force(double fx, double fy, double local_ptx, double local_pty)
{
	ASSERT_(m_b2d_vehicle_body)
	const b2Vec2 wPt = m_b2d_vehicle_body->GetWorldPoint( b2Vec2( local_ptx, local_pty) ); // Application point -> world coords
	m_b2d_vehicle_body->ApplyForce( b2Vec2(fx,fy), wPt, true/*wake up*/);
}
