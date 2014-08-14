/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/VehicleDynamics/VehicleDifferential.h>
#include <mrpt/opengl.h>

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
	fixtureDef.restitution = 0.1;

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
		
		scene.insert(m_gl_chassis);
	}


	// Update them:
	// ----------------------------------
	m_gl_chassis->setPose( mrpt::math::TPose3D( m_q.vals[0], m_q.vals[1], 0.01, m_q.vals[2], 0.0, 0.0) );
}

