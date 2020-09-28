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

#include <mrpt/version.h>
#if MRPT_VERSION >= 0x204
#include <mrpt/system/thread_name.h>
#endif

using namespace mvsim;
using namespace std;

void World::TGUI_Options::parse_from(const rapidxml::xml_node<char>& node)
{
	parse_xmlnode_children_as_param(node, params, {}, "[World::TGUI_Options]");
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
		MRPT_LOG_DEBUG("[World::internal_GUI_thread] Started.");

		struct InfoPerObject
		{
			nanogui::CheckBox* cb = nullptr;
			Simulable* simulable = nullptr;
			VisualObject* visual = nullptr;
		};
		std::vector<InfoPerObject> gui_cbObjects;

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
			auto we = WorldElementBase::factory(this, nullptr, "groundgrid");
			m_world_elements.push_back(we);
		}

		// Add top menu subwindow:
		// -----------------------------
		{
			auto winMenu = new nanogui::Window(m_gui_win.get(), "");
			winMenu->setPosition(nanogui::Vector2i(0, 0));
			winMenu->setLayout(new nanogui::BoxLayout(
				nanogui::Orientation::Horizontal, nanogui::Alignment::Middle,
				5));
			nanogui::Theme* modTheme =
				new nanogui::Theme(m_gui_win->screen()->nvgContext());
			modTheme->mWindowHeaderHeight = 1;
			winMenu->setTheme(modTheme);

			winMenu->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_BOLD_LEFT)
				->setCallback([this]() {
					m_gui_win->setVisible(false);
					nanogui::leave();
				});

			winMenu->add<nanogui::Label>("      ");  // separator

			winMenu
				->add<nanogui::CheckBox>(
					"Orthogonal view",
					[&](bool b) {
						m_gui_win->camera().setCameraProjective(!b);
					})
				->setChecked(m_gui_options.ortho);

			std::vector<std::string> lstVehicles;
			lstVehicles.reserve(m_vehicles.size() + 1);

			lstVehicles.push_back("[none]");  // None
			for (const auto& v : m_vehicles) lstVehicles.push_back(v.first);

			winMenu->add<nanogui::Label>("Camera follows:");
			auto cbFollowVeh = winMenu->add<nanogui::ComboBox>(lstVehicles);
			cbFollowVeh->setSelectedIndex(0);
			cbFollowVeh->setCallback([this, lstVehicles](int idx) {
				if (idx == 0)
					m_gui_options.follow_vehicle.clear();
				else if (idx <= static_cast<int>(m_vehicles.size()))
					m_gui_options.follow_vehicle = lstVehicles[idx];
			});
		}

		// Add Status window
		// -----------------------------
		MRPT_TODO("Add bottom windows bar to restore minimized windows");
		nanogui::Window* winStatus = nullptr;
		{
			nanogui::Window* w = new nanogui::Window(m_gui_win.get(), "Status");
			winStatus = w;

			w->setPosition(nanogui::Vector2i(10, 45));
			w->setLayout(new nanogui::BoxLayout(
				nanogui::Orientation::Vertical, nanogui::Alignment::Fill));
			w->setFixedWidth(250);

			w->buttonPanel()
				->add<nanogui::Button>("", ENTYPO_ICON_CROSS)
				->setCallback([w]() { w->setVisible(false); });

			m_lbCpuUsage = w->add<nanogui::Label>(" ");
			m_lbStatuses.resize(5);
			for (size_t i = 0; i < m_lbStatuses.size(); i++)
				m_lbStatuses[i] = w->add<nanogui::Label>(" ");
		}

		// Add editor window
		// -----------------------------
		nanogui::Window* winEditor = nullptr;
		{
			nanogui::Window* w = new nanogui::Window(m_gui_win.get(), "Editor");
			winEditor = w;

			w->setPosition(nanogui::Vector2i(10, 300));
			w->setLayout(new nanogui::BoxLayout(
				nanogui::Orientation::Vertical, nanogui::Alignment::Minimum));
			w->setFixedWidth(300);

			w->buttonPanel()
				->add<nanogui::Button>("", ENTYPO_ICON_CROSS)
				->setCallback([w]() { w->setVisible(false); });

			w->add<nanogui::Label>("Selected object", "sans-bold");

			if (!m_simulableObjects.empty())
			{
				const int pnWidth = 300, pnHeight = 200,
						  listWidth = pnWidth / 3;

				auto pn = w->add<nanogui::Widget>();
				pn->setFixedSize({pnWidth, pnHeight});
				pn->setLayout(new nanogui::GridLayout(
					nanogui::Orientation::Horizontal, 3 /*columns */,
					nanogui::Alignment::Minimum));

				nanogui::VScrollPanel* vscrolls[3] = {
					pn->add<nanogui::VScrollPanel>(),
					pn->add<nanogui::VScrollPanel>(),
					pn->add<nanogui::VScrollPanel>()};

				for (auto vs : vscrolls)
					vs->setFixedSize({listWidth, pnHeight});

				// vscroll should only have *ONE* child. this is what `wrapper`
				// is for
				nanogui::Widget* wrappers[3];
				std::array<const char*, 3> wrapTitles = {"Vehicles", "Blocks",
														 "World elements"};
				for (int i = 0; i < 3; i++)
				{
					wrappers[i] = vscrolls[i]->add<nanogui::Widget>();
					wrappers[i]->add<nanogui::Label>(wrapTitles[i]);
					wrappers[i]->setFixedSize({listWidth, pnHeight});
					wrappers[i]->setLayout(new nanogui::GridLayout(
						nanogui::Orientation::Horizontal, 1 /*columns */,
						nanogui::Alignment::Minimum));
				}

				for (const auto& o : m_simulableObjects)
				{
					InfoPerObject ipo;

					const auto& name = o.first;
					bool isVehicle = false;
					if (auto v = dynamic_cast<VehicleBase*>(o.second.get()); v)
					{
						isVehicle = true;
						ipo.visual = dynamic_cast<VisualObject*>(v);
					}
					bool isBlock = false;
					if (auto v = dynamic_cast<Block*>(o.second.get()); v)
					{
						isBlock = true;
						ipo.visual = dynamic_cast<VisualObject*>(v);
					}
					bool isWorldElement = false;
					if (auto v =
							dynamic_cast<WorldElementBase*>(o.second.get());
						v)
					{
						isWorldElement = true;
						ipo.visual = dynamic_cast<VisualObject*>(v);
					}
					auto wrapper = isVehicle
									   ? wrappers[0]
									   : (isBlock ? wrappers[1] : wrappers[2]);

					std::string label = name;
					if (label.empty()) label = "(unnamed)";

					auto cb = wrapper->add<nanogui::CheckBox>(label);
					ipo.cb = cb;
					ipo.simulable = o.second.get();
					gui_cbObjects.emplace_back(ipo);

					cb->setChecked(false);
					cb->setCallback([cb, &gui_cbObjects, ipo](bool check) {
						// Only mark 1 at once:
						if (check)
						{
							for (auto& c : gui_cbObjects)
							{
								c.cb->setChecked(false);
								c.visual->showBoundingBox(false);
							}
						}
						cb->setChecked(check);

						// If checked, show bounding box:
						if (ipo.visual && check)
							ipo.visual->showBoundingBox(true);
					});
				}
			}

			auto btnMove = w->add<nanogui::Button>("Move...");
			btnMove->setFlags(nanogui::Button::ToggleButton);
			btnMove->setCallback([btnMove]() {
				//
			});
		}

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

		MRPT_LOG_DEBUG("[World::internal_GUI_thread] Mainloop ended.");

		m_gui_win.reset();

		nanogui::shutdown();
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"[internal_GUI_init] Exception: " << mrpt::exception_to_str(e));
	}
	m_gui_thread_running = false;
}

void World::internalUpdate3DSceneObjects(
	mrpt::opengl::COpenGLScene::Ptr& gl_scene)
{
	// Update view of map elements
	// -----------------------------
	m_timlogger.enter("update_GUI.2.map-elements");

	for (auto& e : m_world_elements) e->guiUpdate(*gl_scene);

	m_timlogger.leave("update_GUI.2.map-elements");

	// Update view of vehicles
	// -----------------------------
	m_timlogger.enter("update_GUI.3.vehicles");

	for (auto& v : m_vehicles) v.second->guiUpdate(*gl_scene);

	m_timlogger.leave("update_GUI.3.vehicles");

	// Update view of blocks
	// -----------------------------
	m_timlogger.enter("update_GUI.4.blocks");

	for (auto& v : m_blocks) v.second->guiUpdate(*gl_scene);

	m_timlogger.leave("update_GUI.4.blocks");

	// Other messages
	// -----------------------------
	m_timlogger.enter("update_GUI.5.text-msgs");
	{
		// 1st line: time
		double cpu_usage_ratio =
			std::max(1e-10, m_timlogger.getMeanTime("run_simulation.cpu_dt")) /
			std::max(1e-10, m_timlogger.getMeanTime("run_simulation.dt"));

		if (m_lbCpuUsage)
			m_lbCpuUsage->setCaption(mrpt::format(
				"Time: %s (CPU usage: %.03f%%)",
				mrpt::system::formatTimeInterval(this->m_simul_time).c_str(),
				cpu_usage_ratio * 100.0));

		// User supplied-lines:
		m_gui_msg_lines_mtx.lock();
		const std::string msg_lines = m_gui_msg_lines;
		m_gui_msg_lines_mtx.unlock();

		if (!msg_lines.empty())
		{
			MRPT_TODO("Split lines?");
			m_lbStatuses[0]->setCaption(msg_lines);
		}
	}

	m_timlogger.leave("update_GUI.5.text-msgs");

	// Camera follow modes:
	// -----------------------
	if (!m_gui_options.follow_vehicle.empty())
	{
		auto it = m_vehicles.find(m_gui_options.follow_vehicle);
		if (it == m_vehicles.end())
		{
			static bool warn1st = true;
			if (warn1st)
			{
				MRPT_LOG_ERROR_FMT(
					"GUI: Camera set to follow vehicle named '%s' which can't "
					"be found!",
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
			MRPT_LOG_DEBUG("[update_GUI] Launching GUI thread...");

			m_gui_thread = std::thread(&World::internal_GUI_thread, this);
#if MRPT_VERSION >= 0x204
			mrpt::system::thread_name("guiThread", m_gui_thread);
#endif
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
				MRPT_LOG_DEBUG("[update_GUI] GUI thread started.");
			}
		}
	}

	if (!m_gui_win)
	{
		MRPT_LOG_THROTTLE_WARN(
			2.0,
			"[World::update_GUI] Ignoring call since GUI window has "
			"been closed.");
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
