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
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
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
bool World::is_GUI_open() const { return !!m_gui.gui_win; }
//!< Forces closing the GUI window, if any.
void World::close_GUI() { m_gui.gui_win.reset(); }

// Add top menu subwindow:
void World::GUI::prepare_top_menu()
{
#if MRPT_VERSION >= 0x211
	nanogui::Window* winMenu = gui_win->createManagedSubWindow("Control");
#else
	nanogui::Window* winMenu = new nanogui::Window(gui_win.get(), "Control");
#endif

	winMenu->setPosition(nanogui::Vector2i(5, 80));
	winMenu->setLayout(new nanogui::BoxLayout(
		nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 5));

	winMenu->add<nanogui::Button>("Quit", ENTYPO_ICON_ARROW_BOLD_LEFT)
		->setCallback([this]() {
			gui_win->setVisible(false);
			nanogui::leave();
		});

	winMenu->add<nanogui::Label>("      ");  // separator

	winMenu
		->add<nanogui::CheckBox>(
			"Orthogonal view",
			[&](bool b) { gui_win->camera().setCameraProjective(!b); })
		->setChecked(m_parent.m_gui_options.ortho);

	std::vector<std::string> lstVehicles;
	lstVehicles.reserve(m_parent.m_vehicles.size() + 1);

	lstVehicles.push_back("[none]");  // None
	for (const auto& v : m_parent.m_vehicles) lstVehicles.push_back(v.first);

	winMenu->add<nanogui::Label>("Camera follows:");
	auto cbFollowVeh = winMenu->add<nanogui::ComboBox>(lstVehicles);
	cbFollowVeh->setSelectedIndex(0);
	cbFollowVeh->setCallback([this, lstVehicles](int idx) {
		if (idx == 0)
			m_parent.m_gui_options.follow_vehicle.clear();
		else if (idx <= static_cast<int>(m_parent.m_vehicles.size()))
			m_parent.m_gui_options.follow_vehicle = lstVehicles[idx];
	});
}

// Add Status window
void World::GUI::prepare_status_window()
{
#if MRPT_VERSION >= 0x211
	nanogui::Window* w = gui_win->createManagedSubWindow("Status");
#else
	nanogui::Window* w = new nanogui::Window(gui_win.get(), "Status");
#endif

	w->setPosition(nanogui::Vector2i(5, 170));
	w->setLayout(new nanogui::BoxLayout(
		nanogui::Orientation::Vertical, nanogui::Alignment::Fill));
	w->setFixedWidth(250);

#if MRPT_VERSION < 0x211
	w->buttonPanel()
		->add<nanogui::Button>("", ENTYPO_ICON_CROSS)
		->setCallback([w]() { w->setVisible(false); });
#endif

	lbCpuUsage = w->add<nanogui::Label>(" ");
	lbStatuses.resize(5);
	for (size_t i = 0; i < lbStatuses.size(); i++)
		lbStatuses[i] = w->add<nanogui::Label>(" ");
}

// Add editor window
void World::GUI::prepare_editor_window()
{
#if MRPT_VERSION >= 0x211
	nanogui::Window* w = gui_win->createManagedSubWindow("Editor");
#else
	nanogui::Window* w = new nanogui::Window(gui_win.get(), "Editor");
#endif

	w->setPosition(nanogui::Vector2i(5, 300));
	w->setLayout(new nanogui::BoxLayout(
		nanogui::Orientation::Vertical, nanogui::Alignment::Minimum, 3, 3));
	w->setFixedWidth(300);

#if MRPT_VERSION < 0x211
	w->buttonPanel()
		->add<nanogui::Button>("", ENTYPO_ICON_CROSS)
		->setCallback([w]() { w->setVisible(false); });
#endif

	w->add<nanogui::Label>("Selected object", "sans-bold");

	if (!m_parent.m_simulableObjects.empty())
	{
		const int pnWidth = 300, pnHeight = 200, listWidth = pnWidth / 3;

		auto pn = w->add<nanogui::Widget>();
		pn->setFixedSize({pnWidth, pnHeight});
		pn->setLayout(new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 3 /*columns */,
			nanogui::Alignment::Minimum));

		nanogui::VScrollPanel* vscrolls[3] = {pn->add<nanogui::VScrollPanel>(),
											  pn->add<nanogui::VScrollPanel>(),
											  pn->add<nanogui::VScrollPanel>()};

		for (auto vs : vscrolls) vs->setFixedSize({listWidth, pnHeight});

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

		for (const auto& o : m_parent.m_simulableObjects)
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
			if (auto v = dynamic_cast<WorldElementBase*>(o.second.get()); v)
			{
				isWorldElement = true;
				ipo.visual = dynamic_cast<VisualObject*>(v);
			}
			auto wrapper =
				isVehicle ? wrappers[0] : (isBlock ? wrappers[1] : wrappers[2]);

			std::string label = name;
			if (label.empty()) label = "(unnamed)";

			auto cb = wrapper->add<nanogui::CheckBox>(label);
			ipo.cb = cb;
			ipo.simulable = o.second;
			gui_cbObjects.emplace_back(ipo);

			cb->setChecked(false);
			cb->setCallback([cb, ipo, this](bool check) {
				// deselect former one:
				if (gui_selectedObject.visual)
					gui_selectedObject.visual->showBoundingBox(false);
				if (gui_selectedObject.cb)
					gui_selectedObject.cb->setChecked(false);
				gui_selectedObject = InfoPerObject();

				cb->setChecked(check);

				// If checked, show bounding box:
				if (ipo.visual && check)
				{
					gui_selectedObject = ipo;
					ipo.visual->showBoundingBox(true);
				}

				const bool btnsEnabled = !!gui_selectedObject.simulable;
				for (auto b : btns_selectedOps) b->setEnabled(btnsEnabled);
			});
		}
	}

	w->add<nanogui::Label>(" ");

	btnReplaceObject = w->add<nanogui::Button>("Click to replace...");
	btnReplaceObject->setFlags(nanogui::Button::Flags::ToggleButton);
	btns_selectedOps.push_back(btnReplaceObject);

	{
		auto pn = w->add<nanogui::Widget>();
		pn->setLayout(new nanogui::BoxLayout(
			nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 2, 2));
		pn->add<nanogui::Label>("Reorient:");
		auto slAngle = pn->add<nanogui::Slider>();
		slAngle->setRange({-M_PI, M_PI});
		slAngle->setCallback([this](float v) {
			if (!gui_selectedObject.simulable) return;
			auto p = gui_selectedObject.simulable->getPose();
			p.yaw = v;
			gui_selectedObject.simulable->setPose(p);
		});
		slAngle->setFixedWidth(150);
		btns_selectedOps.push_back(slAngle);
	}

	auto btnPlaceCoords = w->add<nanogui::Button>("Replace by coordinates...");
	btns_selectedOps.push_back(btnPlaceCoords);
	btnPlaceCoords->setCallback([this]() {
		//
		if (!gui_selectedObject.simulable) return;

		auto* formPose = new nanogui::Window(gui_win.get(), "Enter new pose");
		formPose->setLayout(new nanogui::GridLayout(
			nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 5));

		nanogui::TextBox* lbs[3];

		formPose->add<nanogui::Label>("x coordinate:");
		lbs[0] = formPose->add<nanogui::TextBox>();
		formPose->add<nanogui::Label>("y coordinate:");
		lbs[1] = formPose->add<nanogui::TextBox>();
		formPose->add<nanogui::Label>("Orientation:");
		lbs[2] = formPose->add<nanogui::TextBox>();

		for (int i = 0; i < 3; i++)
		{
			lbs[i]->setEditable(true);
			lbs[i]->setFixedSize({100, 20});
			lbs[i]->setValue("0.0");
			lbs[i]->setUnits(i == 2 ? "[deg]" : "[m]");
			lbs[i]->setDefaultValue("0.0");
			lbs[i]->setFontSize(16);
			lbs[i]->setFormat("[-]?[0-9]*\\.?[0-9]+");
		}

		const auto pos = gui_selectedObject.simulable->getPose();
		lbs[0]->setValue(std::to_string(pos.x));
		lbs[1]->setValue(std::to_string(pos.y));
		lbs[2]->setValue(std::to_string(mrpt::RAD2DEG(pos.yaw)));

		formPose->add<nanogui::Label>("");
		formPose->add<nanogui::Label>("");

		formPose->add<nanogui::Button>("Cancel")->setCallback(
			[formPose]() { formPose->dispose(); });

		formPose->add<nanogui::Button>("Accept")->setCallback(
			[formPose, this, lbs]() {
				gui_selectedObject.simulable->setPose(
					{// X:
					 std::stod(lbs[0]->value()),
					 // Y:
					 std::stod(lbs[1]->value()),
					 // Z:
					 .0,
					 // Yaw
					 mrpt::DEG2RAD(std::stod(lbs[2]->value())),
					 // Pitch
					 0.0,
					 // Roll:
					 0.0});
				formPose->dispose();
			});

		formPose->setModal(true);
		formPose->center();
		formPose->setVisible(true);
	});

	for (auto b : btns_selectedOps) b->setEnabled(false);

}  // end "editor" window

void World::internal_GUI_thread()
{
	try
	{
		MRPT_LOG_DEBUG("[World::internal_GUI_thread] Started.");

		// Start GUI:
		nanogui::init();

		mrpt::gui::CDisplayWindowGUI_Params cp;
		cp.maximized = m_gui_options.start_maximized;

		m_gui.gui_win = mrpt::gui::CDisplayWindowGUI::Create(
			"mvsim", m_gui_options.win_w, m_gui_options.win_h, cp);

		// Add a background scene:
		auto scene = mrpt::opengl::COpenGLScene::Create();
		{
			std::lock_guard<std::mutex> lck(
				m_gui.gui_win->background_scene_mtx);
			m_gui.gui_win->background_scene = std::move(scene);
		}

		// Only if the world is empty: at least introduce a ground grid:
		if (m_world_elements.empty())
		{
			auto we = WorldElementBase::factory(this, nullptr, "groundgrid");
			m_world_elements.push_back(we);
		}

		// Windows:
		m_gui.prepare_top_menu();
		m_gui.prepare_status_window();
		m_gui.prepare_editor_window();

		// Finish GUI setup:
		m_gui.gui_win->performLayout();
		auto& cam = m_gui.gui_win->camera();

		cam.setCameraPointing(0.0f, .0f, .0f);
		cam.setCameraProjective(!m_gui_options.ortho);
		cam.setZoomDistance(m_gui_options.camera_distance);

		// Main GUI loop
		// ---------------------
		m_gui.gui_win->drawAll();
		m_gui.gui_win->setVisible(true);

		// Listen for keyboard events:
		m_gui.gui_win->setKeyboardCallback([&](int key, int /*scancode*/,
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
		m_gui.gui_win->setLoopCallback([&]() {
			if (m_gui_thread_must_close) nanogui::leave();

			// Update all GUI elements:
			ASSERT_(m_gui.gui_win->background_scene);
			internalUpdate3DSceneObjects(m_gui.gui_win->background_scene);

			// handle mouse operations:
			m_gui.handle_mouse_operations();
		});

		nanogui::mainloop(m_gui_options.refresh_fps);

		MRPT_LOG_DEBUG("[World::internal_GUI_thread] Mainloop ended.");

		m_gui.gui_win.reset();

		nanogui::shutdown();
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"[internal_GUI_init] Exception: " << mrpt::exception_to_str(e));
	}
	m_gui_thread_running = false;
}

void World::GUI::handle_mouse_operations()
{
	MRPT_START
	if (!gui_win) return;

	mrpt::opengl::COpenGLViewport::Ptr vp;
	{
		auto lck = mrpt::lockHelper(gui_win->background_scene_mtx);
		vp = gui_win->background_scene->getViewport();
	}
	ASSERT_(vp);

	const auto mousePt = gui_win->mousePos();
	mrpt::math::TLine3D ray;
	vp->get3DRayForPixelCoord(mousePt.x(), mousePt.y(), ray);

	// Create a 3D plane, e.g. Z=0
	const auto ground_plane =
		mrpt::math::TPlane::From3Points({0, 0, 0}, {1, 0, 0}, {0, 1, 0});

	// Intersection of the line with the plane:
	mrpt::math::TObject3D inters;
	mrpt::math::intersect(ray, ground_plane, inters);

	// Interpret the intersection as a point, if there is an
	// intersection:
	if (inters.getPoint(clickedPt))
	{
		// Move object to the position picked by the user:
		// vp->getByClass<CDisk>(0)->setLocation(clickedPt);
	}

#if MRPT_VERSION >= 0x211
	const auto screen = gui_win->screen();
	const bool leftClick = screen->mouseState() == 0x01;

	// Replace object?
	if (btnReplaceObject && btnReplaceObject->pushed())
	{
		static bool isReplacing = false;

		// Start of replace? When the button push is released:
		if (!isReplacing && !leftClick)
		{
			isReplacing = true;
		}
		if (gui_selectedObject.simulable)
		{
			// btnReplaceObject->screen()->setCursor()

			mrpt::math::TPose3D p = gui_selectedObject.simulable->getPose();
			p.x = clickedPt.x;
			p.y = clickedPt.y;

			gui_selectedObject.simulable->setPose(p);
		}
		if (isReplacing && leftClick)
		{
			isReplacing = false;
			btnReplaceObject->setPushed(false);
		}
	}
#endif

	MRPT_END
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

		if (m_gui.lbCpuUsage)
			m_gui.lbCpuUsage->setCaption(mrpt::format(
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
			m_gui.lbStatuses[0]->setCaption(msg_lines);
		}
		m_gui.lbStatuses[1]->setCaption(
			std::string("Mouse: ") + m_gui.clickedPt.asString());
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
					"GUI: Camera set to follow vehicle named '%s' which "
					"can't "
					"be found!",
					m_gui_options.follow_vehicle.c_str());
				warn1st = true;
			}
		}
		else
		{
			const mrpt::poses::CPose2D pose = it->second->getCPose2D();
			m_gui.gui_win->camera().setCameraPointing(pose.x(), pose.y(), 0.0f);
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

	if (!m_gui.gui_win)
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
