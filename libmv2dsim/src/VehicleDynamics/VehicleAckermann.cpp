/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleDynamics/VehicleAckermann.h>
#include <mv2dsim/World.h>

#include "xml_utils.h"

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <rapidxml.hpp>
#include <cmath>

using namespace mv2dsim;
using namespace std;

// Ctor:
DynamicsAckermann::DynamicsAckermann(World *parent) :
	VehicleBase(parent),
	m_chassis_mass(500.0),
	m_chassis_z_min(0.20),
	m_chassis_z_max(1.40),
	m_chassis_color(0xa0,0x00,0x00)
{
	using namespace mrpt::math;

	// Default shape:
	m_chassis_poly.push_back( TPoint2D(-0.8, -1.0) );
	m_chassis_poly.push_back( TPoint2D(-0.8,  1.0) );
	m_chassis_poly.push_back( TPoint2D( 1.5,  0.9) );
	m_chassis_poly.push_back( TPoint2D( 1.8,  0.8) );
	m_chassis_poly.push_back( TPoint2D( 1.8, -0.8) );
	m_chassis_poly.push_back( TPoint2D( 1.5, -0.9) );
	updateMaxRadiusFromPoly();

	m_fixture_chassis = NULL;
	for (int i=0;i<4;i++) m_fixture_wheels[i]=NULL;

	// Default values:
	// rear-left:
	m_wheels_info[0].x = 0;
	m_wheels_info[0].y = -0.9;
	// rear-right:
	m_wheels_info[1].x = 0;
	m_wheels_info[1].y = 0.9;
	// Front-left:
	m_wheels_info[2].x = 1.3;
	m_wheels_info[2].y = -0.9;
	// Front-right:
	m_wheels_info[3].x = 1.3;
	m_wheels_info[3].y = 0.9;
}

/** The derived-class part of load_params_from_xml() */
void DynamicsAckermann::dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node)
{
	// <chassis ...> </chassis>
	const rapidxml::xml_node<char> * xml_chassis = xml_node->first_node("chassis");
	if (xml_chassis)
	{
		// Attribs:	
		std::map<std::string,TParamEntry> attribs;
		attribs["mass"] = TParamEntry("%lf", &this->m_chassis_mass);
		attribs["zmin"] = TParamEntry("%lf", &this->m_chassis_z_min );
		attribs["zmax"] = TParamEntry("%lf", &this->m_chassis_z_max );
		attribs["color"] = TParamEntry("%color", &this->m_chassis_color );

		parse_xmlnode_attribs(*xml_chassis, attribs,"[DynamicsAckermann::dynamics_load_params_from_xml]" );

		// Shape node (optional, fallback to default shape if none found)
		const rapidxml::xml_node<char> * xml_shape = xml_chassis->first_node("shape");
		if (xml_shape)
			mv2dsim::parse_xmlnode_shape(*xml_shape, m_chassis_poly, "[DynamicsAckermann::dynamics_load_params_from_xml]");
	}

	//<rl_wheel pos="0  1" mass="6.0" width="0.30" diameter="0.62" />
	//<rr_wheel pos="0 -1" mass="6.0" width="0.30" diameter="0.62" />
	//<fl_wheel mass="6.0" width="0.30" diameter="0.62" />
	//<fr_wheel mass="6.0" width="0.30" diameter="0.62" />
	const char*w_names[4] = {
		"rl_wheel", // 0
		"rr_wheel", // 1
		"fl_wheel", // 2
		"fr_wheel"  // 3
	};
	// Load common params:
	for (int i=0;i<4;i++)
	{
		const rapidxml::xml_node<char> * xml_wheel = xml_node->first_node(w_names[i]);
		if (xml_wheel) m_wheels_info[i].loadFromXML(xml_wheel);		
	}

	//<f_wheels_x>1.3</f_wheels_x>
	//<f_wheels_d>2.0</f_wheels_d>
	// Load front ackermann wheels params:
	{
		double front_x = 1.3; 
		double front_d = 2.0;
		std::map<std::string,TParamEntry> ack_ps;
		ack_ps["f_wheels_x"] = TParamEntry("%lf", &front_x);
		ack_ps["f_wheels_d"] = TParamEntry("%lf", &front_d);

		// Front-left:
		m_wheels_info[2].x = front_x;
		m_wheels_info[2].y = -0.5*front_d;
		// Front-right:
		m_wheels_info[3].x = front_x;
		m_wheels_info[3].y = 0.5*front_d;
	}

	// Vehicle controller:
	// -------------------------------------------------
	{
		const rapidxml::xml_node<char> * xml_control = xml_node->first_node("controller");
		if (xml_control)
		{
			rapidxml::xml_attribute<char> *control_class = xml_control->first_attribute("class");
			if (!control_class || !control_class->value()) throw runtime_error("[DynamicsAckermann] Missing 'class' attribute in <controller> XML node");

			const std::string sCtrlClass = std::string(control_class->value());
			if (sCtrlClass==ControllerRawForces::class_name())    m_controller = ControllerBasePtr(new ControllerRawForces(*this) );
			else throw runtime_error(mrpt::format("[DynamicsAckermann] Unknown 'class'='%s' in <controller> XML node",sCtrlClass.c_str()));

			m_controller->load_config(*xml_control);
		}
	}
	// Default controller: 
	if (!m_controller) m_controller = ControllerBasePtr(new ControllerRawForces(*this) );


}

/** Create bodies, fixtures, etc. for the dynamical simulation */
void DynamicsAckermann::create_multibody_system(b2World* world)
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
	}

	// Define shape of wheels:
	// ------------------------------
	for (int i=0;i<4;i++)
	{
		b2PolygonShape wheelShape;
		wheelShape.SetAsBox(
			m_wheels_info[i].diameter*0.5, m_wheels_info[i].width*0.5,
			b2Vec2(m_wheels_info[i].x,m_wheels_info[i].y), 0 );

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


/** Must create a new object in the scene and/or update it according to the current state */
void DynamicsAckermann::gui_update( mrpt::opengl::COpenGLScene &scene)
{
	using namespace mrpt::math;

	VehicleBase::gui_update_sensors(scene); // Common part: update sensors

	// 1st time call?? -> Create objects
	// ----------------------------------
	if (!m_gl_chassis)
	{
		m_gl_chassis = mrpt::opengl::CSetOfObjects::Create();

		// Wheels shape:
		for (int i=0;i<4;i++)
		{
			m_gl_wheels[i]= mrpt::opengl::CSetOfObjects::Create();
			m_wheels_info[i].getAs3DObject(*m_gl_wheels[i]);
			m_gl_chassis->insert(m_gl_wheels[i]);
		}
		// Robot shape:
		//m_gl_chassis->insert( mrpt::opengl::stock_objects::RobotPioneer() );
		mrpt::opengl::CPolyhedronPtr gl_poly = mrpt::opengl::CPolyhedron::CreateCustomPrism( m_chassis_poly, m_chassis_z_max-m_chassis_z_min);
		gl_poly->setLocation(0,0, m_chassis_z_min);
		gl_poly->setColor( mrpt::utils::TColorf(m_chassis_color) );
		m_gl_chassis->insert(gl_poly);

		scene.insert(m_gl_chassis);
	}


	// Update them:
	// ----------------------------------
	m_gl_chassis->setPose( mrpt::math::TPose3D( m_q.vals[0], m_q.vals[1], 0.01, m_q.vals[2], 0.0, 0.0) );

	for (int i=0;i<4;i++)
	{
		const Wheel & w = m_wheels_info[i];
		m_gl_wheels[i]->setPose( mrpt::math::TPose3D( w.x,w.y, 0.5*w.diameter, w.yaw, w.getPhi(), 0.0) );
	}

}

// See docs in base class:
void DynamicsAckermann::apply_motor_forces(const TSimulContext &context, std::vector<double> &out_torque_per_wheel)
{
	// Longitudinal forces at each wheel:
	out_torque_per_wheel.assign(4, 0.0);

	if (m_controller)
	{
		// Invoke controller:
		TControllerInput ci;
		ci.context = context;
		TControllerOutput co;
		m_controller->control_step(ci,co);
		// Take its output:
		out_torque_per_wheel[0] = co.rl_torque;
		out_torque_per_wheel[1] = co.rr_torque;
		out_torque_per_wheel[2] = co.fl_torque;
		out_torque_per_wheel[3] = co.fr_torque;

		// Kinematically-driven steering wheels: 
		// Ackermann formulas for inner&outer weels turning angles wrt the equivalent (central) one:
		{
			// EQ1: cot(d)+0.5*w/l = cot(do)
			// EQ2: cot(di)=cot(do)-w/l
			const double w = m_wheels_info[2].y - m_wheels_info[3].y;
			const double l = m_wheels_info[2].x - m_wheels_info[0].x;
			ASSERT_(l>0)
			const double w_l=w/l;
			const double delta= std::abs(co.steer_ang);
			const bool delta_neg = (co.steer_ang<0);
			ASSERT_BELOW_(delta,0.5*M_PI-0.01)
			const double cot_do = 1.0/tan(delta)+ 0.5*w_l;
			const double cot_di = cot_do - w_l;
			// delta>0: do->right, di->left wheel
			// delta<0: do->left , di->right wheel
			m_wheels_info[delta_neg ? 3:2].yaw = atan(1.0/cot_di);
			m_wheels_info[delta_neg ? 2:3].yaw = atan(1.0/cot_do);
		}

	}

}

void DynamicsAckermann::updateMaxRadiusFromPoly()
{
	using namespace mrpt::math;

	m_max_radius=0.001f;
	for (TPolygon2D::const_iterator it=m_chassis_poly.begin();it!=m_chassis_poly.end();++it)
	{
		const float n=it->norm();
		mrpt::utils::keep_max(m_max_radius,n);
	}	
}
