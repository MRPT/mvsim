/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#include <mv2dsim/World.h>
#include <mv2dsim/VehicleBase.h>
#include <mv2dsim/VehicleDynamics/VehicleDifferential.h>
#include <mv2dsim/FrictionModels/LinearFriction.h>

#include "JointXMLnode.h"
#include "VehicleClassesRegistry.h"
#include "xml_utils.h"

#include <rapidxml.hpp>
#include <rapidxml_utils.hpp>
#include <rapidxml_print.hpp>
#include <mrpt/utils/utils_defs.h>  // mrpt::format()
#include <mrpt/poses/CPose2D.h>
#include <mrpt/opengl/CCylinder.h>

#include <sstream>      // std::stringstream
#include <map>
#include <string>

using namespace mv2dsim;
using namespace std;


VehicleClassesRegistry veh_classes_registry;


// Protected ctor:
VehicleBase::VehicleBase(World *parent) :
	VisualObject(parent),
	m_b2d_vehicle_body(NULL),
	m_q(0,0,0),
	m_dq(0,0,0)
{
}

void VehicleBase::simul_post_timestep_common(const TSimulContext &context)
{
	if (m_b2d_vehicle_body)
	{
		// Pos:
		const b2Vec2 &pos = m_b2d_vehicle_body->GetPosition();
		const float32 angle = m_b2d_vehicle_body->GetAngle();
		m_q.vals[0]=pos(0);
		m_q.vals[1]=pos(1);
		m_q.vals[2]=angle;

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
	VehicleBase *veh = NULL;
	const xml_node<> *dyn_node = veh_root_node.first_node("dynamics");
	if (!dyn_node) throw runtime_error("[VehicleBase::factory] Missing XML node <dynamics>");

	const xml_attribute<> *dyn_class = dyn_node->first_attribute("class");
	if (!dyn_class || !dyn_class->value() ) throw runtime_error("[VehicleBase::factory] Missing mandatory attribute 'class' in node <dynamics>");

	if (!strcmp("differential",dyn_class->value()))
	{
		veh = new DynamicsDifferential(parent);
	}
	else
	{
		throw runtime_error(mrpt::format("[VehicleBase::factory] Unknown vehicle dynamics class '%s'",dyn_class->value()));
	}

	// Initialize here all common params shared by any polymorphic class:
	// -------------------------------------------------
	// attrib: name

	// (Mandatory) initial pose:
	{
		const xml_node<> *node = veh_root_node.first_node("init_pose");
		if (!node) throw runtime_error("[VehicleBase::factory] Missing XML node <init_pose>");

		if (3!= ::sscanf(node->value(),"%lf %lf %lf",&veh->m_q.vals[0],&veh->m_q.vals[1],&veh->m_q.vals[2]))
			throw runtime_error("[VehicleBase::factory] Error parsing <init_pose>...</init_pose>");
		veh->m_q.vals[2] *= M_PI/180.0;
	}

	// (Optional) initial vel:
	{
		const xml_node<> *node = veh_root_node.first_node("init_vel");
		if (node)
		{
			if (3!= ::sscanf(node->value(),"%lf %lf %lf",&veh->m_dq.vals[0],&veh->m_dq.vals[1],&veh->m_dq.vals[2]))
				throw runtime_error("[VehicleBase::factory] Error parsing <init_vel>...</init_vel>");
			veh->m_dq.vals[2] *= M_PI/180.0;

			// Convert twist (velocity) from local -> global coords:
			const mrpt::poses::CPose2D pose(0,0,veh->m_q.vals[2]); // Only the rotation
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
		veh->m_b2d_vehicle_body->SetTransform( b2Vec2( veh->m_q.vals[0], veh->m_q.vals[1] ),  veh->m_q.vals[2] );
		// Init vel:
		veh->m_b2d_vehicle_body->SetLinearVelocity( b2Vec2(veh->m_dq.vals[0], veh->m_dq.vals[1] ) );
		veh->m_b2d_vehicle_body->SetAngularVelocity(veh->m_dq.vals[2] );
	}

	// Vehicle controller:
	// -------------------------------------------------

	// Friction model:
	// -------------------------------------------------
	veh->m_friction = stlplus::smart_ptr<FrictionBase>( new LinearFriction() );
	veh->m_friction->init(parent,veh);

	// Sensors:
	// -------------------------------------------------


	return veh;
}

VehicleBase* VehicleBase::factory(World* parent, const std::string &xml_text)
{
	// Parse the string as if it was an XML file:
	std::stringstream s;
	s.str( xml_text );

	rapidxml::file<> fil(s);
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

/** Loads vehicle params from input XML node of type "<vehicle>...</vehicle>".
	* See derived classes & documentation for a list of accepted params.
	*/
void VehicleBase::load_params_from_xml(const rapidxml::xml_node<char> *xml_node)
{
}

/// \overload
void VehicleBase::load_params_from_xml(const std::string &xml_text)
{
}

VehicleBase::TInfoPerWheel::TInfoPerWheel() :
	x(.0),y(-.5),yaw(.0),
	diameter(.4),width(.2),
	mass(2.0),
	color_r(0.2),color_g(0.2),color_b(0.2),color_a(1.0)
{
}

void VehicleBase::TInfoPerWheel::getAs3DObject(mrpt::opengl::CSetOfObjects &obj)
{
	obj.clear();

	mrpt::opengl::CCylinderPtr gl_wheel = mrpt::opengl::CCylinder::Create( 0.5*diameter,0.5*diameter,this->width, 15, 1);
	gl_wheel->setColor(color_r,color_g,color_b,color_a);
	gl_wheel->setPose(mrpt::poses::CPose3D(0,0.5*width,0,  0,0,mrpt::utils::DEG2RAD(90) ));

	mrpt::opengl::CSetOfObjectsPtr gl_wheel_frame = mrpt::opengl::CSetOfObjects::Create();
	gl_wheel_frame->insert(gl_wheel);
	//gl_wheel_frame->insert( mrpt::opengl::stock_objects::CornerXYZSimple() );

	obj.setPose( mrpt::math::TPose3D( x,y, 0.5*diameter, yaw, 0.0, 0.0) );

     obj.insert(gl_wheel_frame);
}

void VehicleBase::TInfoPerWheel::loadFromXML(const rapidxml::xml_node<char> *xml_node)
{
	ASSERT_(xml_node)
	// Parse attributes:
	// <l_wheel pos="0.0 -0.5" mass="2.0" width="0.10" diameter="0.30" />
	// pos:
     {
     	const rapidxml::xml_attribute<char> * attr = xml_node->first_attribute("pos");
		if (attr && attr->value())
		{
			const std::string sAttr = attr->value();
			mrpt::math::TPose2D p(0,0,0);
			::sscanf(sAttr.c_str(),"%lf %lf %lf",&p.x,&p.y,&p.phi);
               this->x = p.x;
               this->y = p.y;
               this->yaw = p.phi;
		}
     }

     TXMLAttribs attribs[] = {
     	{ "mass","%lf", &this->mass },
     	{ "width","%lf", &this->width },
     	{ "diameter","%lf", &this->diameter },
     	{ "color_r","%lf", &this->color_r },
     	{ "color_g","%lf", &this->color_g },
     	{ "color_b","%lf", &this->color_b },
     	{ "color_a","%lf", &this->color_a }
	};

	parse_xmlnode_attribs(*xml_node, attribs, sizeof(attribs)/sizeof(attribs[0]),"[VehicleBase::TInfoPerWheel]" );

}


void VehicleBase::simul_pre_timestep(const TSimulContext &context)
{
	// Apply motor forces/torques:
	this->apply_motor_forces(context);

	// Apply friction model at each wheel:
	m_friction->update_step(context);

}

/** Last time-step velocity (of the ref. point, in local coords) */
vec3 VehicleBase::getVelocityLocal() const
{
	vec3 local_vel;
	local_vel.vals[2] = m_dq.vals[2]; // omega remains the same.

	const mrpt::poses::CPose2D p(0,0, m_q.vals[2]); // "-" means inverse pose
	p.composePoint( 
		m_dq.vals[0],m_dq.vals[1],
		local_vel.vals[0],local_vel.vals[1]);
	return local_vel;
}

mrpt::poses::CPose2D VehicleBase::getCPose2D() const 
{
	return mrpt::poses::CPose2D(m_q.vals[0],m_q.vals[1],m_q.vals[2]);
}