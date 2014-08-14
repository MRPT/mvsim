/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#include <mv2dsim/World.h>

#include <mrpt/utils/utils_defs.h>  // mrpt::format()
#include <mrpt/opengl.h>

#include <iostream> // for debugging
#include <algorithm> // count()
#include <stdexcept>
#include <map>

// XML parsing:
#include <rapidxml.hpp>
#include <rapidxml_print.hpp>

using namespace mv2dsim;
using namespace std;

// Default ctor: inits empty world.
World::World() : 
	m_simul_time(0.0),
	m_simul_timestep(0.010),
	m_b2d_vel_iters(6), 
	m_b2d_pos_iters(3),
	m_box2d_world( NULL )
{
	this->clear_all();
}

// Dtor.
World::~World()
{
	this->clear_all();
	delete m_box2d_world; m_box2d_world=NULL;
}

// Resets the entire simulation environment to an empty world.
void World::clear_all(bool acquire_mt_lock)
{
	try
	{
		if (acquire_mt_lock) m_world_cs.enter();

		// Reset params:
		m_simul_time = 0.0;
		
		// (B2D) World contents:
		// ---------------------------------------------
		delete m_box2d_world;
		m_box2d_world = new b2World( b2Vec2_zero );

		// Define the ground body.
		b2BodyDef groundBodyDef;
		groundBodyDef.position.Set(5.0f, 0.0f);
		m_b2_ground_body = m_box2d_world->CreateBody(&groundBodyDef);

		// Clear m_vehicles & other lists of objs:
		// ---------------------------------------------
		for(std::list<VehicleBase*>::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it) delete *it;
		m_vehicles.clear();

		for(std::list<WorldElementBase*>::iterator it=m_world_elements.begin();it!=m_world_elements.end();++it) delete *it;
		m_world_elements.clear();
		

		if (acquire_mt_lock) m_world_cs.leave();
	}
	catch (std::exception &)
	{
		if (acquire_mt_lock) m_world_cs.leave();
		throw; // re-throw
	}
}

/** Load an entire world description into this object from a specification in XML format.
	* \exception std::exception On any error, with what() giving a descriptive error message
	*/
void World::load_from_XML(const std::string &xml_text)
{
	using namespace std;
	using namespace rapidxml;

	mrpt::synch::CCriticalSectionLocker csl( &m_world_cs ); // Protect multithread access

	// Clear the existing world.
	this->clear_all(false /* critical section is already acquired */); 
	
	// Parse the XML input:
	rapidxml::xml_document<> xml;
	char* input_str = const_cast<char*>(xml_text.c_str());
	try {
		xml.parse<0>(input_str);
	}
	catch (rapidxml::parse_error &e) {
		unsigned int line = static_cast<long>(std::count(input_str, e.where<char>(), '\n') + 1);
		throw std::runtime_error( mrpt::format("XML parse error (Line %u): %s", static_cast<unsigned>(line), line, e.what() ) );
	}

	// Sanity checks:
	const xml_node<> *root = xml.first_node();
	if (!root) throw runtime_error("XML parse error: No root node found (empty file?)");
	if (0!=strcmp(root->name(),"mv2dsim_world")) throw runtime_error(mrpt::format("XML root element is '%s' ('mv2dsim_world' expected)",root->name()));
	
	// Optional: format version attrib:
	const xml_attribute<> *attrb_version = root->first_attribute("version");
	int version_major = 1, version_min = 0;
	if (attrb_version)
		sscanf(attrb_version->value(),"%i.%i",&version_major, &version_min);

	// load general parameters:
	// ------------------------------------------------
	struct TParamEntry { const char* frmt; void *val; TParamEntry() : frmt(NULL),val(NULL) {} TParamEntry(const char* frmt_,void *val_) : frmt(frmt_),val(val_) {} };
	std::map<std::string,TParamEntry> other_world_params;
	other_world_params["simul_timestep"] = TParamEntry("%lf", &this->m_simul_timestep);
	other_world_params["b2d_vel_iters"] = TParamEntry("%i", &this->m_b2d_vel_iters);
	other_world_params["b2d_pos_iters"] = TParamEntry("%i", &this->m_b2d_pos_iters);
	
	MRPT_TODO("Export this list of params to ROS dynamic reconfigure")

	// Process all nodes:
	// ------------------------------------------------
	xml_node<> *node = root->first_node();
	while (node)
	{
		if (!strcmp(node->name(),"world:gridmap")) 
		{
			// TODO!!!
			WorldElementBase *element_gridmap = new WorldElementBase(this);

			this->m_world_elements.push_back(element_gridmap);
		} 
		else if (!strcmp(node->name(),"vehicle")) 
		{
			VehicleBase* veh = VehicleBase::factory(this,node);
			this->m_vehicles.push_back( veh );
		} 
		else
		{
			// Default: Check if it's a parameter: 
			std::map<std::string,TParamEntry>::const_iterator it_param = other_world_params.find(node->name());
			
			if (it_param != other_world_params.end() )
			{
				// parse parameter:
				if (1 != ::sscanf(node->value(),it_param->second.frmt, it_param->second.val ) )
				{
					throw std::runtime_error( 
						mrpt::format(
							"Error parsing entry '%s' with expected format '%s' and content '%s'", 
							node->name(), it_param->second.frmt, node->value()
							) );
				}
			}
			else
			{
				// Unknown element!!
				std::cerr << "[World::load_from_XML] *Warning* Ignoring unknown XML node type '"<< node->name() <<"'\n";
			}
		}

		// Move on to next node:
		node = node->next_sibling(NULL);
	}
		
}

/** Runs the simulation for a given time interval (in seconds) */
void World::run_simulation(double dt)
{
	// sanity checks:
	ASSERT_(dt>0)
	ASSERT_(m_simul_timestep>0)

	// Run in time steps:
	const double end_time = m_simul_time+dt;
	const double timetol = 1e-6; // tolerance for rounding errors summing time steps
	while (m_simul_time<(end_time-timetol))
	{
		const double step = std::min(end_time-m_simul_time, this->m_simul_timestep);
		internal_one_timestep(step);
	}
}

/** Runs one individual time step */
void World::internal_one_timestep(double dt)
{
	// 1) Pre-step
	TSimulContext context;
	context.b2_world   = m_box2d_world;
	context.simul_time = m_simul_time;

	for(std::list<VehicleBase*>::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it) 
		(*it)->simul_pre_timestep(context);
	
	// 2) Run dynamics
	m_box2d_world->Step(dt, m_b2d_vel_iters, m_b2d_pos_iters);
	
	m_simul_time+= dt;  // Avance time

	// 3) Simulate sensors
	
	// 4) Save dynamical state into vehicles classes
	context.simul_time = m_simul_time;
	for(std::list<VehicleBase*>::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it) 
	{
		(*it)->simul_post_timestep_common(context);
		(*it)->simul_post_timestep(context);
	}
	
}

/** Updates (or sets-up upon first call) the GUI visualization of the scene. 
	* \note This method is prepared to be called concurrently with the simulation, and doing so is recommended to assure a smooth multi-threading simulation.
	*/
size_t ID_GLTEXT_CLOCK = 0;

void World::update_GUI()
{
	// First call?
	// -----------------------
	if (!m_gui_win)
	{
		m_gui_win = mrpt::gui::CDisplayWindow3D::Create("mv2dsim",800,600);
		mrpt::opengl::COpenGLScenePtr gl_scene = m_gui_win->get3DSceneAndLock();

		gl_scene->insert( mrpt::opengl::CGridPlaneXY::Create() );

		m_gui_win->unlockAccess3DScene();
	}

	mrpt::opengl::COpenGLScenePtr gl_scene = m_gui_win->get3DSceneAndLock(); // ** LOCK **

	// Update view of map elements
	// -----------------------------

	// Update view of vehicles
	// -----------------------------
	for(std::list<VehicleBase*>::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it) 
		(*it)->gui_update(*gl_scene);

	// Update view of sensors
	// -----------------------------

	// Other messages
	// -----------------------------
	m_gui_win->addTextMessage(2,2, mrpt::format("Time: %s", mrpt::system::formatTimeInterval(this->m_simul_time).c_str()), mrpt::utils::TColorf(1,1,1,0.5), "serif", 10, mrpt::opengl::NICE, ID_GLTEXT_CLOCK );
	
	// Force refresh view
	// -----------------------
	m_gui_win->unlockAccess3DScene(); // ** UNLOCK **
	m_gui_win->repaint();
}
