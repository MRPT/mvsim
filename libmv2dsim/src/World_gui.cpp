/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#include <mv2dsim/World.h>

#include <mrpt/utils/utils_defs.h>  // mrpt::format()
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>

#include <rapidxml.hpp>
#include "xml_utils.h"

using namespace mv2dsim;
using namespace std;


// Default ctor: inits empty world.
World::TGUI_Options::TGUI_Options() :
	ortho(false),
	camera_distance(80),
	fov_deg(60)
{
}

void World::TGUI_Options::parse_from(const rapidxml::xml_node<char> &node)
{
	std::map<std::string,TParamEntry> gui_params;
	gui_params["ortho"] = TParamEntry("%bool", &ortho);
	gui_params["cam_distance"]  = TParamEntry("%lf",&camera_distance);
	gui_params["fov_deg"] = TParamEntry("%lf",&fov_deg); 

	parse_xmlnode_children_as_param(node,gui_params,"[World::TGUI_Options]");
}


// Text labels unique IDs:
size_t ID_GLTEXT_CLOCK = 0;

/** Updates (or sets-up upon first call) the GUI visualization of the scene.
	* \note This method is prepared to be called concurrently with the simulation, and doing so is recommended to assure a smooth multi-threading simulation.
	*/
void World::update_GUI( TGUIKeyEvent *out_keyevent )
{
	// First call?
	// -----------------------
	if (!m_gui_win)
	{
		m_timlogger.enter("update_GUI_init");

		m_gui_win = mrpt::gui::CDisplayWindow3D::Create("mv2dsim",800,600);
		m_gui_win->setCameraZoom(m_gui_options.camera_distance);
		m_gui_win->setCameraProjective(!m_gui_options.ortho);
		m_gui_win->setFOV(m_gui_options.fov_deg);
		mrpt::opengl::COpenGLScenePtr gl_scene = m_gui_win->get3DSceneAndLock();

		gl_scene->insert( mrpt::opengl::CGridPlaneXY::Create() );

		m_gui_win->unlockAccess3DScene();
		m_timlogger.leave("update_GUI_init");
	}

	m_timlogger.enter("update_GUI"); // Don't count initialization, since that is a total outlier and lacks interest!

	m_timlogger.enter("update_GUI.1.get-lock");
	mrpt::opengl::COpenGLScenePtr gl_scene = m_gui_win->get3DSceneAndLock(); // ** LOCK **
	m_timlogger.leave("update_GUI.1.get-lock");

	// Update view of map elements
	// -----------------------------
	m_timlogger.enter("update_GUI.2.map-elements");

	for(std::list<WorldElementBase*>::iterator it=m_world_elements.begin();it!=m_world_elements.end();++it)
		(*it)->gui_update(*gl_scene);

	m_timlogger.leave("update_GUI.2.map-elements");

	// Update view of vehicles
	// -----------------------------
	m_timlogger.enter("update_GUI.3.vehicles");

	for(TListVehicles::iterator it=m_vehicles.begin();it!=m_vehicles.end();++it)
		(*it)->gui_update(*gl_scene);

	m_timlogger.leave("update_GUI.3.vehicles");

	// Update view of sensors
	// -----------------------------
	m_timlogger.enter("update_GUI.4.sensors");
	m_timlogger.leave("update_GUI.4.sensors");

	// Other messages
	// -----------------------------
	m_timlogger.enter("update_GUI.5.text-msgs");

	m_gui_win->addTextMessage(2,2, mrpt::format("Time: %s", mrpt::system::formatTimeInterval(this->m_simul_time).c_str()), mrpt::utils::TColorf(1,1,1,0.5), "serif", 10, mrpt::opengl::NICE, ID_GLTEXT_CLOCK );

	m_timlogger.leave("update_GUI.5.text-msgs");

	// Force refresh view
	// -----------------------
	m_gui_win->unlockAccess3DScene(); // ** UNLOCK **
	m_gui_win->repaint();

	m_timlogger.leave("update_GUI");

	// Key-strokes:
	// -----------------------
	if (out_keyevent && m_gui_win->keyHit())
		out_keyevent->keycode = m_gui_win->getPushedKey( &out_keyevent->key_modifier );		

}

