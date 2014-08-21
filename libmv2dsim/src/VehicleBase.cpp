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
#include <mv2dsim/FrictionModels/FrictionBase.h>
#include <mv2dsim/FrictionModels/LinearFriction.h> // For use as default model

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

TClassFactory_vehicleDynamics mv2dsim::classFactory_vehicleDynamics;


// Explicit registration calls seem to be one (the unique?) way to assure registration takes place:
void register_all_veh_dynamics()
{
	static bool done = false;
	if (done) return; else done=true;

	REGISTER_VEHICLE_DYNAMICS("differential",DynamicsDifferential)
}

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


	// Friction model: 
	// Parse <friction> node, or assume default linear model:
	// -----------------------------------------------------------
	{
		const xml_node<> *frict_node = veh_root_node.first_node("friction");
		if (!frict_node)
		{
			// Default:
			veh->m_friction = stlplus::smart_ptr<FrictionBase>( new LinearFriction(*veh, NULL /*default params*/) );
		}
		else
		{
			// Parse:
			veh->m_friction = stlplus::smart_ptr<FrictionBase>(  FrictionBase::factory(*veh,frict_node) );
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
			veh->m_sensors.push_back( SensorBasePtr(se));
		}
	}

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
	color(0xff323232)
{
}

void VehicleBase::TInfoPerWheel::getAs3DObject(mrpt::opengl::CSetOfObjects &obj)
{
	obj.clear();

	mrpt::opengl::CCylinderPtr gl_wheel = mrpt::opengl::CCylinder::Create( 0.5*diameter,0.5*diameter,this->width, 15, 1);
	gl_wheel->setColor(mrpt::utils::TColorf(color));
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
	// <l_wheel pos="0.0 -0.5 [OPTIONAL_ANG]" mass="2.0" width="0.10" diameter="0.30" />
	// pos:
     {
     	const rapidxml::xml_attribute<char> * attr = xml_node->first_attribute("pos");
		if (attr && attr->value())
		{
			const std::string sAttr = attr->value();
			vec3 v = parseXYPHI(sAttr, true);
			this->x =v.vals[0];
			this->y =v.vals[1];
			this->yaw =v.vals[2];
		}
     }

	std::map<std::string,TParamEntry> attribs;
	attribs["mass"] = TParamEntry("%lf", &this->mass);
	attribs["width"] = TParamEntry("%lf", &this->width);
	attribs["diameter"] = TParamEntry("%lf", &this->diameter);
	attribs["color"] = TParamEntry("%color", &this->color);
	
	parse_xmlnode_attribs(*xml_node, attribs,"[VehicleBase::TInfoPerWheel]" );

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

/** To be called at derived classes' gui_update() */
void VehicleBase::gui_update_sensors( mrpt::opengl::COpenGLScene &scene)
{
	for (TListSensors::iterator it=m_sensors.begin();it!=m_sensors.end();++it)
		(*it)->gui_update(scene);
}
