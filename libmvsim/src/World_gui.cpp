/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mvsim/World.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

void World::TGUI_Options::parse_from(const rapidxml::xml_node<char>& node)
{
	TParameterDefinitions gui_params;
	gui_params["win_w"] = TParamEntry("%u", &win_w);
	gui_params["win_h"] = TParamEntry("%u", &win_h);
	gui_params["ortho"] = TParamEntry("%bool", &ortho);
	gui_params["show_forces"] = TParamEntry("%bool", &show_forces);
	gui_params["force_scale"] = TParamEntry("%lf", &force_scale);
	gui_params["cam_distance"] = TParamEntry("%lf", &camera_distance);
	gui_params["fov_deg"] = TParamEntry("%lf", &fov_deg);
	gui_params["follow_vehicle"] = TParamEntry("%s", &follow_vehicle);
	gui_params["start_maximized"] = TParamEntry("%bool", &start_maximized);
	gui_params["refresh_fps"] = TParamEntry("%i", &refresh_fps);

	parse_xmlnode_children_as_param(node, gui_params, "[World::TGUI_Options]");
}

// Text labels unique IDs:
size_t ID_GLTEXT_CLOCK = 0;

//!< Return true if the GUI window is open, after a previous call to
//! update_GUI()
bool World::is_GUI_open() const { return !!m_gui_win; }
//!< Forces closing the GUI window, if any.
void World::close_GUI() { m_gui_win.reset(); }

void World::internal_GUI_thread()
{
	try
	{
		std::cout << "[World::internal_GUI_thread] Init.\n";
		nanogui::init();

		mrpt::gui::CDisplayWindowGUI_Params cp;
		cp.maximized = m_gui_options.start_maximized;

		m_gui_win = mrpt::gui::CDisplayWindowGUI::Create(
			"mvsim", m_gui_options.win_w, m_gui_options.win_h, cp);

		// Add a background scene:
		auto scene = mrpt::opengl::COpenGLScene::Create();
		{
			std::lock_guard<std::mutex> lck(m_gui_win->background_scene_mtx);
			m_gui_win->background_scene = std::move(scene);
		}

		// Only if the world is empty: at least introduce a ground grid:
		if (m_world_elements.empty())
		{
			WorldElementBase* we =
				WorldElementBase::factory(this, nullptr, "groundgrid");
			this->m_world_elements.push_back(we);
		}

		// Add controls:

		m_gui_win->performLayout();
		auto& cam = m_gui_win->camera();

		cam.setCameraPointing(0.0f, .0f, .0f);
		cam.setCameraProjective(!m_gui_options.ortho);
		cam.setZoomDistance(m_gui_options.camera_distance);

		// Main GUI loop
		// ---------------------
		m_gui_win->drawAll();
		m_gui_win->setVisible(true);

		// Listen for keyboard events:
		m_gui_win->setKeyboardCallback([&](int key, int /*scancode*/,
										   int action, int modifiers) {
			if (action != GLFW_PRESS && action != GLFW_REPEAT) return false;

			auto lck = mrpt::lockHelper(m_lastKeyEvent_mtx);

			m_lastKeyEvent.keycode = key;
			m_lastKeyEvent.modifierShift = (modifiers & GLFW_MOD_SHIFT) != 0;
			m_lastKeyEvent.modifierCtrl = (modifiers & GLFW_MOD_CONTROL) != 0;
			m_lastKeyEvent.modifierSuper = (modifiers & GLFW_MOD_SUPER) != 0;
			m_lastKeyEvent.modifierAlt = (modifiers & GLFW_MOD_ALT) != 0;

			m_lastKeyEventValid = true;

			return false;
		});

		m_gui_thread_running = true;

		// The GUI must be closed from this same thread. Use a shared atomic
		// bool:
		m_gui_win->setLoopCallback([&]() {
			if (m_gui_thread_must_close) nanogui::leave();

			// Update all GUI elements:
			ASSERT_(m_gui_win->background_scene);

			internalUpdate3DSceneObjects(m_gui_win->background_scene);
		});

		nanogui::mainloop(m_gui_options.refresh_fps);

		std::cout << "[World::internal_GUI_thread] Mainloop ended.\n";

		m_gui_win.reset();

		nanogui::shutdown();
	}
	catch (const std::exception& e)
	{
		std::cerr << "[internal_GUI_init] Exception: "
				  << mrpt::exception_to_str(e) << std::endl;
	}
	m_gui_thread_running = false;
}

void World::internalUpdate3DSceneObjects(
	mrpt::opengl::COpenGLScene::Ptr& gl_scene)
{
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
		const int txt_h = 12, space_h = 2;	// font height
		int txt_y = 4;

		// 1st line: time
		gl_scene->getViewport()->addTextMessage(
			2, 2,
			mrpt::format(
				"Time: %s",
				mrpt::system::formatTimeInterval(this->m_simul_time).c_str()),
			ID_GLTEXT_CLOCK);
		txt_y += txt_h + space_h;

		// User supplied-lines:
		m_gui_msg_lines_mtx.lock();
		const std::string msg_lines = m_gui_msg_lines;
		m_gui_msg_lines_mtx.unlock();

		if (!msg_lines.empty())
		{
			const size_t nLines =
				std::count(msg_lines.begin(), msg_lines.end(), '\n');
			txt_y += nLines * (txt_h + space_h);
			gl_scene->getViewport()->addTextMessage(
				2, txt_y, msg_lines, ID_GLTEXT_CLOCK + 1);
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
			m_gui_win->camera().setCameraPointing(pose.x(), pose.y(), 0.0f);
		}
	}
}

void World::update_GUI(TUpdateGUIParams* guiparams)
{
	// First call?
	// -----------------------
	{
		auto lock = mrpt::lockHelper(m_gui_thread_start_mtx);
		if (!m_gui_thread_running && !m_gui_thread.joinable())
		{
			std::cout << "[update_GUI] Launching GUI thread...\n";

			m_gui_thread = std::thread(&World::internal_GUI_thread, this);
			for (int timeout = 0; timeout < 300; timeout++)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				if (m_gui_thread_running) break;
			}

			if (!m_gui_thread_running)
			{
				THROW_EXCEPTION("Timeout waiting for GUI to open!");
			}
			else
			{
				std::cout << "[update_GUI] GUI thread started.\n";
			}
		}
	}

	if (!m_gui_win)
	{
		std::cerr << "[World::update_GUI] Ignoring call since GUI window has "
					 "been closed."
				  << std::endl;
		return;
	}

	m_timlogger.enter("update_GUI");  // Don't count initialization, since that
									  // is a total outlier and lacks interest!

	m_gui_msg_lines_mtx.lock();
	m_gui_msg_lines = guiparams->msg_lines;
	m_gui_msg_lines_mtx.unlock();

	m_timlogger.leave("update_GUI");

	// Key-strokes:
	// -----------------------
	if (guiparams && m_lastKeyEventValid)
	{
		auto lck = mrpt::lockHelper(m_lastKeyEvent_mtx);

		guiparams->keyevent = std::move(m_lastKeyEvent);
		m_lastKeyEventValid = false;
	}
}
