/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */
#include <mvsim/World.h>

#include <mrpt/utils/utils_defs.h>  // mrpt::format()
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>

#include <rapidxml.hpp>
#include "xml_utils.h"

using namespace mvsim;
using namespace std;

// Default ctor: inits empty world.
World::TGUI_Options::TGUI_Options()
	: win_w(800),
	  win_h(600),
	  ortho(false),
	  show_forces(false),
	  force_scale(0.01),
	  camera_distance(80),
	  fov_deg(60)
{
}

void World::TGUI_Options::parse_from(const rapidxml::xml_node<char>& node)
{
	std::map<std::string, TParamEntry> gui_params;
	gui_params["win_w"] = TParamEntry("%u", &win_w);
	gui_params["win_h"] = TParamEntry("%u", &win_h);
	gui_params["ortho"] = TParamEntry("%bool", &ortho);
	gui_params["show_forces"] = TParamEntry("%bool", &show_forces);
	gui_params["force_scale"] = TParamEntry("%lf", &force_scale);
	gui_params["cam_distance"] = TParamEntry("%lf", &camera_distance);
	gui_params["fov_deg"] = TParamEntry("%lf", &fov_deg);
	gui_params["follow_vehicle"] = TParamEntry("%s", &follow_vehicle);

	parse_xmlnode_children_as_param(node, gui_params, "[World::TGUI_Options]");
}

World::TUpdateGUIParams::TUpdateGUIParams() {}
// Text labels unique IDs:
size_t ID_GLTEXT_CLOCK = 0;

//!< Return true if the GUI window is open, after a previous call to
//!update_GUI()
bool World::is_GUI_open() const { return !!m_gui_win; }
//!< Forces closing the GUI window, if any.
void World::close_GUI() { m_gui_win.reset(); }
/** Updates (or sets-up upon first call) the GUI visualization of the scene.
* \note This method is prepared to be called concurrently with the simulation,
* and doing so is recommended to assure a smooth multi-threading simulation.
*/
void World::update_GUI(TUpdateGUIParams* guiparams)
{
	// First call?
	// -----------------------
	if (!m_gui_win)
	{
		m_timlogger.enter("update_GUI_init");

		m_gui_win = mrpt::gui::CDisplayWindow3D::Create(
			"mvsim", m_gui_options.win_w, m_gui_options.win_h);
		m_gui_win->setCameraZoom(m_gui_options.camera_distance);
		m_gui_win->setCameraProjective(!m_gui_options.ortho);
		m_gui_win->setFOV(m_gui_options.fov_deg);

		// Only if the world is empty: at least introduce a ground grid:
		if (m_world_elements.empty())
		{
			WorldElementBase* we =
				WorldElementBase::factory(this, NULL, "groundgrid");
			this->m_world_elements.push_back(we);
		}

		m_timlogger.leave("update_GUI_init");
	}

	m_timlogger.enter("update_GUI");  // Don't count initialization, since that
									  // is a total outlier and lacks interest!

	m_timlogger.enter("update_GUI.1.get-lock");
	mrpt::opengl::COpenGLScene::Ptr gl_scene =
		m_gui_win->get3DSceneAndLock();  // ** LOCK **
	m_timlogger.leave("update_GUI.1.get-lock");

	// 1st time only:
	// Build different "stacks" or "z-order levels" for
	// rendering with transparencies to work nicely
	// --------------------------------------------------------
	if (!gl_scene->getByName("level_0"))
	{
		for (unsigned int i = 0; i < 5; i++)
		{
			auto gl_obj = mrpt::opengl::CSetOfObjects::Create();
			gl_obj->setName(mrpt::format("level_%u", i));
			gl_scene->insert(gl_obj);
		}
	}

	// Update view of map elements
	// -----------------------------
	m_timlogger.enter("update_GUI.2.map-elements");

	for (std::list<WorldElementBase*>::iterator it = m_world_elements.begin();
		 it != m_world_elements.end(); ++it)
		(*it)->gui_update(*gl_scene);

	m_timlogger.leave("update_GUI.2.map-elements");

	// Update view of vehicles
	// -----------------------------
	m_timlogger.enter("update_GUI.3.vehicles");

	for (TListVehicles::iterator it = m_vehicles.begin();
		 it != m_vehicles.end(); ++it)
		it->second->gui_update(*gl_scene);

	m_timlogger.leave("update_GUI.3.vehicles");

	// Update view of blocks
	// -----------------------------
	m_timlogger.enter("update_GUI.4.blocks");

	for (TListBlocks::iterator it = m_blocks.begin(); it != m_blocks.end();
		 ++it)
		it->second->gui_update(*gl_scene);

	m_timlogger.leave("update_GUI.4.blocks");

	// Other messages
	// -----------------------------
	m_timlogger.enter("update_GUI.5.text-msgs");
	{
		const int txt_h = 12, space_h = 2;  // font height
		int txt_y = 4;

		// 1st line: time
		m_gui_win->addTextMessage(
			2, 2,
			mrpt::format(
				"Time: %s",
				mrpt::system::formatTimeInterval(this->m_simul_time).c_str()),
			mrpt::utils::TColorf(1, 1, 1, 0.5), "serif", txt_h,
			mrpt::opengl::NICE, ID_GLTEXT_CLOCK);
		txt_y += txt_h + space_h;

		// User supplied-lines:
		if (guiparams)
		{
			const size_t nLines = std::count(
				guiparams->msg_lines.begin(), guiparams->msg_lines.end(), '\n');
			txt_y += nLines * (txt_h + space_h);
			m_gui_win->addTextMessage(
				2, txt_y, guiparams->msg_lines,
				mrpt::utils::TColorf(1, 1, 1, 0.5), "serif", txt_h,
				mrpt::opengl::NICE, ID_GLTEXT_CLOCK + 1);
		}
	}

	m_timlogger.leave("update_GUI.5.text-msgs");

	// Camera follow modes:
	// -----------------------
	if (!m_gui_options.follow_vehicle.empty())
	{
		TListVehicles::const_iterator it =
			m_vehicles.find(m_gui_options.follow_vehicle);
		if (it == m_vehicles.end())
		{
			static bool warn1st = true;
			if (warn1st)
			{
				std::cerr << mrpt::format(
					"GUI: Camera set to follow vehicle named '%s' which can't "
					"be found!\n",
					m_gui_options.follow_vehicle.c_str());
				warn1st = true;
			}
		}
		else
		{
			const mrpt::poses::CPose2D pose = it->second->getCPose2D();
			m_gui_win->setCameraPointingToPoint(pose.x(), pose.y(), 0.0f);
		}
	}

	// Force refresh view
	// -----------------------
	m_gui_win->unlockAccess3DScene();  // ** UNLOCK **
	m_gui_win->repaint();

	m_timlogger.leave("update_GUI");

	// Key-strokes:
	// -----------------------
	if (guiparams && m_gui_win->keyHit())
		guiparams->keyevent.keycode =
			m_gui_win->getPushedKey(&guiparams->keyevent.key_modifier);
}
