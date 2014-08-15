/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleDynamics/VehicleDifferential.h>
#include <mrpt/opengl.h>

#include <rapidxml.hpp>

using namespace mv2dsim;
using namespace std;

// Ctor:
DynamicsDifferential::DynamicsDifferential(World *parent) :
	VehicleBase(parent)
{
}

/** The derived-class part of load_params_from_xml() */
void DynamicsDifferential::dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node)
{
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


}

/** Create bodies, fixtures, etc. for the dynamical simulation */
void DynamicsDifferential::create_multibody_system(b2World* world)
{
	// Define the dynamic body. We set its position and call the body factory.
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;

	m_b2d_vehicle_body = world->CreateBody(&bodyDef);

	// Define another box shape for our dynamic body.
	b2PolygonShape dynamicBox;
	dynamicBox.SetAsBox(0.1f, 0.1f);

	// Define the dynamic body fixture.
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &dynamicBox;
	fixtureDef.restitution = 0.05;

	// Set the box density to be non-zero, so it will be dynamic.
	fixtureDef.density = 1.0f;

	// Override the default friction.
	fixtureDef.friction = 0.3f;

	// Add the shape to the body.
	m_b2d_vehicle_body->CreateFixture(&fixtureDef);
}


/** Must create a new object in the scene and/or update it according to the current state */
void DynamicsDifferential::gui_update( mrpt::opengl::COpenGLScene &scene)
{
	// 1st time call?? -> Create objects
	// ----------------------------------
	if (!m_gl_chassis)
	{
		m_gl_chassis = mrpt::opengl::CSetOfObjects::Create();
		m_gl_chassis->insert( mrpt::opengl::stock_objects::RobotPioneer() );

		for (int i=0;i<2;i++)
		{
			m_gl_wheels[i]= mrpt::opengl::CSetOfObjects::Create();
			m_wheels_info[i].getAs3DObject(*m_gl_wheels[i]);
			m_gl_chassis->insert(m_gl_wheels[i]);
		}
		scene.insert(m_gl_chassis);
	}


	// Update them:
	// ----------------------------------
	m_gl_chassis->setPose( mrpt::math::TPose3D( m_q.vals[0], m_q.vals[1], 0.01, m_q.vals[2], 0.0, 0.0) );

	for (int i=0;i<2;i++)
	{
		const VehicleBase::TInfoPerWheel & w = m_wheels_info[i];
		m_gl_wheels[i]->setPose( mrpt::math::TPose3D( w.x,w.y, 0.5*w.length, w.yaw, 0.0, 0.0) );
	}

}

