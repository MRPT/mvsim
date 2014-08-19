/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleDynamics/VehicleDifferential.h>
#include <mv2dsim/World.h>

#include "xml_utils.h"

#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <rapidxml.hpp>

using namespace mv2dsim;
using namespace std;

// Ctor:
DynamicsDifferential::DynamicsDifferential(World *parent) :
	VehicleBase(parent),
	m_chassis_mass(15.0),
	m_chassis_z_min(0.05),
	m_chassis_z_max(0.6)
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

	m_fixture_chassis = NULL;
	for (int i=0;i<2;i++) m_fixture_wheels[i]=NULL;
}

/** The derived-class part of load_params_from_xml() */
void DynamicsDifferential::dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node)
{
	// <chassis ...> </chassis>
	const rapidxml::xml_node<char> * xml_chassis = xml_node->first_node("chassis");
	if (xml_chassis)
	{
		TXMLAttribs attribs[] = {
			{ "mass","%lf", &this->m_chassis_mass },
			{ "zmin","%lf", &this->m_chassis_z_min },
			{ "zmax","%lf", &this->m_chassis_z_max }
		};
		parse_xmlnode_attribs(*xml_chassis, attribs, sizeof(attribs)/sizeof(attribs[0]),"[DynamicsDifferential::dynamics_load_params_from_xml]" );

		MRPT_TODO("XML->Shape")
	}


     // <l_wheel ...>, <r_wheel ...>
     {
     	const rapidxml::xml_node<char> * xml_wheel_l = xml_node->first_node("l_wheel");
		if (xml_wheel_l)
			m_wheels_info[0].loadFromXML(xml_wheel_l);
		else
		{
			m_wheels_info[0] = VehicleBase::TInfoPerWheel();
			m_wheels_info[0].y = 0.5;
		}
     }
     {
     	const rapidxml::xml_node<char> * xml_wheel_r = xml_node->first_node("r_wheel");
		if (xml_wheel_r)
			m_wheels_info[1].loadFromXML(xml_wheel_r);
		else
		{
			m_wheels_info[1] = VehicleBase::TInfoPerWheel();
			m_wheels_info[1].y = -0.5;
		}
     }

	// Vehicle controller:
	// -------------------------------------------------
	MRPT_TODO("Load controller from XML")

}

/** Create bodies, fixtures, etc. for the dynamical simulation */
void DynamicsDifferential::create_multibody_system(b2World* world)
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

		// Define the dynamic body fixture.
		b2FixtureDef fixtureDef;
		fixtureDef.shape = &chassisPoly;
		fixtureDef.restitution = 0.05;

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
	for (int i=0;i<2;i++)
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
void DynamicsDifferential::gui_update( mrpt::opengl::COpenGLScene &scene)
{
	using namespace mrpt::math;

	// 1st time call?? -> Create objects
	// ----------------------------------
	if (!m_gl_chassis)
	{
		m_gl_chassis = mrpt::opengl::CSetOfObjects::Create();

		// Wheels shape:
		for (int i=0;i<2;i++)
		{
			m_gl_wheels[i]= mrpt::opengl::CSetOfObjects::Create();
			m_wheels_info[i].getAs3DObject(*m_gl_wheels[i]);
			m_gl_chassis->insert(m_gl_wheels[i]);
		}
		// Robot shape:
		//m_gl_chassis->insert( mrpt::opengl::stock_objects::RobotPioneer() );
		mrpt::opengl::CPolyhedronPtr gl_poly = mrpt::opengl::CPolyhedron::CreateCustomPrism( m_chassis_poly, m_chassis_z_max-m_chassis_z_min);
		gl_poly->setLocation(0,0, m_chassis_z_min);
		gl_poly->setColor(1.0,0.0,0.0,1.0);
		m_gl_chassis->insert(gl_poly);

		scene.insert(m_gl_chassis);
	}


	// Update them:
	// ----------------------------------
	m_gl_chassis->setPose( mrpt::math::TPose3D( m_q.vals[0], m_q.vals[1], 0.01, m_q.vals[2], 0.0, 0.0) );

	for (int i=0;i<2;i++)
	{
		const VehicleBase::TInfoPerWheel & w = m_wheels_info[i];
		m_gl_wheels[i]->setPose( mrpt::math::TPose3D( w.x,w.y, 0.5*w.diameter, w.yaw, 0.0, 0.0) );
	}

}

// See docs in base class:
void DynamicsDifferential::apply_motor_forces(const TSimulContext &context)
{
	// Apply one force on each wheel:
	b2Vec2 net_wheels_forces[2]; // In local (x,y) coordinates (Newtons)

	net_wheels_forces[0]= b2Vec2_zero; // ( context.simul_time<3.0 ? 25.0 : 7.0,0.0);
	net_wheels_forces[1]= b2Vec2_zero; // ( context.simul_time<3.0 ? 5.0 : 6.0 ,0.0);

	for (int wheel=0;wheel<2;wheel++)
	{
		m_b2d_vehicle_body->ApplyForce(
			m_b2d_vehicle_body->GetWorldVector(net_wheels_forces[wheel]), /* force */
			m_b2d_vehicle_body->GetWorldPoint( b2Vec2( m_wheels_info[wheel].x,m_wheels_info[wheel].y) ), /* point */
			true /* wake up */
			);
	}
}

void DynamicsDifferential::updateMaxRadiusFromPoly()
{
	using namespace mrpt::math;

	m_max_radius=0.001f;
	for (TPolygon2D::const_iterator it=m_chassis_poly.begin();it!=m_chassis_poly.end();++it)
	{
		const float n=it->norm();
		mrpt::utils::keep_max(m_max_radius,n);
	}	
}
