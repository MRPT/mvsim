/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/format.h>
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/core/round.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/thread_name.h>
#include <mrpt/version.h>
#include <mrpt/viz/Scene.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/assets/mvsim_icon_64x64.h>

#include <Eigen/Dense>	// asEigen()

// clang-format off
#include <imgui.h>
#include <imgui_internal.h>  // DockBuilder*
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <IconsMaterialSymbols.h>
#include <mrpt/imgui/CImGuiSceneView.h>
#include <mrpt/imgui_vendor/icon_font.h>
#include <GLFW/glfw3.h>
// clang-format on

#include <algorithm>
#include <cctype>  // isspace()
#include <cmath>  // cos(), sin()
#include <cstdlib>	// getenv()
#include <filesystem>
#include <iostream>
#include <rapidxml.hpp>
#include <type_traits>

#include "xml_utils.h"

using namespace mvsim;
using namespace std;

void World::TGUI_Options::parse_from(
	const rapidxml::xml_node<char>& node, mrpt::system::COutputLogger& logger)
{
	parse_xmlnode_children_as_param(node, params, {}, "[World::TGUI_Options]", &logger);
}

void World::LightOptions::parse_from(
	const rapidxml::xml_node<char>& node, mrpt::system::COutputLogger& logger)
{
	// Parse scalar parameters, skipping point_light/spot_light child nodes:
	for (auto* child = node.first_node(); child; child = child->next_sibling(nullptr))
	{
		const std::string name(child->name(), child->name_size());
		if (name == "point_light" || name == "spot_light") continue;

		if (!parse_xmlnode_as_param(*child, params, {}, "[World::LightOptions]"))
		{
			logger.logFmt(
				mrpt::system::LVL_WARN, "Unrecognized tag '<%s>' in [World::LightOptions]",
				name.c_str());
		}
	}
	shadow_cascades = std::clamp(shadow_cascades, 1, 4);
	sensor_shadow_cascades = std::clamp(sensor_shadow_cascades, 1, 4);

	// Parse <point_light> and <spot_light> children:
	for (auto* n = node.first_node(); n; n = n->next_sibling(nullptr))
	{
		const std::string name(n->name(), n->name_size());
		if (name != "point_light" && name != "spot_light")
		{
			continue;
		}
		const auto l = parse_light_xml_node(*n);
		extra_lights.push_back(l);

		logger.logFmt(
			mrpt::system::LVL_INFO, "[LightOptions] Parsed %s at (%.1f, %.1f, %.1f)", name.c_str(),
			l.position.x, l.position.y, l.position.z);
	}
}

namespace
{
// Dear ImGui needs an OpenGL 3.3 context; mrpt rendering too.
constexpr int GL_MAJOR = 3;
constexpr int GL_MINOR = 3;
constexpr float FONT_SIZE = 15.0f;

// Frame rate while the user interacts with the window, and for how long
// after the last input event:
constexpr double INTERACTIVE_FPS = 60.0;
constexpr double INTERACTIVE_HOLD_TIME = 1.0;  // [s]

// Frame scheduling, so GUI frames do not delay OpenGL sensors:
// - A frame is postponed until after the next sensors if it would not finish
//   before them (plus this margin), but never by more than this number of
//   frame periods:
constexpr double SENSOR_MARGIN = 0.005;	 // [s]
constexpr double MAX_POSTPONE_PERIODS = 2.0;
// - While the simulation thread is busier than OVERLOAD_BUSY, the frame rate
//   decreases (down to MIN_OVERLOAD_FPS), and it recovers below RELAXED_BUSY:
constexpr double OVERLOAD_BUSY = 0.9;
constexpr double RELAXED_BUSY = 0.75;
constexpr double MIN_OVERLOAD_FPS = 5.0;
constexpr double OVERLOAD_STEP = 1.25;

/** Path of the autosaved ImGui settings (window layout), shared by all
 * worlds, or empty if no config directory can be found or created. */
std::string imgui_ini_path()
{
	namespace fs = std::filesystem;
	fs::path base;
#ifdef _WIN32
	if (const char* appData = std::getenv("APPDATA"); appData && *appData)
	{
		base = fs::path(appData) / "mvsim";
	}
#else
	if (const char* xdg = std::getenv("XDG_CONFIG_HOME"); xdg && *xdg)
	{
		base = fs::path(xdg) / "mvsim";
	}
	else if (const char* home = std::getenv("HOME"); home && *home)
	{
		base = fs::path(home) / ".config" / "mvsim";
	}
#endif
	if (base.empty())
	{
		return {};
	}
	std::error_code ec;
	fs::create_directories(base, ec);
	if (ec)
	{
		return {};
	}
	return (base / "imgui.ini").string();
}

void set_window_icon(GLFWwindow* win)
{
	// GIMP header image file format (RGB), with 0xff as transparent color:
	constexpr uint8_t TRANSPARENT = 0xff;
	std::vector<uint8_t> rgba(mvsim_icon_width * mvsim_icon_height * 4);
	const char* in = mvsim_icon_data;
	uint8_t* out = rgba.data();
	for (unsigned int i = 0; i < mvsim_icon_width * mvsim_icon_height; i++)
	{
		MVSIM_HEADER_PIXEL(in, out);
		out[3] =
			(out[0] == TRANSPARENT && out[1] == TRANSPARENT && out[2] == TRANSPARENT) ? 0x00 : 0xff;
		out += 4;
	}
	GLFWimage img;
	img.width = static_cast<int>(mvsim_icon_width);
	img.height = static_cast<int>(mvsim_icon_height);
	img.pixels = rgba.data();
	glfwSetWindowIcon(win, 1, &img);
}

World* world_from(GLFWwindow* w) { return static_cast<World*>(glfwGetWindowUserPointer(w)); }

// GLFW input callbacks. ImGui chains to them, since they are installed
// before initializing its GLFW backend.
void on_glfw_key(GLFWwindow* w, int key, int /*scancode*/, int action, int mods)
{
	World* world = world_from(w);
	world->internal_on_gui_key(key, action, mods);
}
void on_glfw_cursor(GLFWwindow* w, double /*x*/, double /*y*/)
{
	world_from(w)->internal_on_gui_input_event();
}
void on_glfw_mouse_button(GLFWwindow* w, int /*button*/, int /*action*/, int /*mods*/)
{
	world_from(w)->internal_on_gui_input_event();
}
void on_glfw_scroll(GLFWwindow* w, double /*dx*/, double /*dy*/)
{
	world_from(w)->internal_on_gui_input_event();
}
void on_glfw_focus(GLFWwindow* w, int focused)
{
	world_from(w)->internal_on_gui_focus(focused == GLFW_TRUE);
}

void setup_imgui_fonts_and_style(GLFWwindow* win)
{
	ImGuiIO& io = ImGui::GetIO();

	ImFontConfig textCfg;
	textCfg.SizePixels = FONT_SIZE;
	io.Fonts->AddFontDefaultVector(&textCfg);

	// Merge the icons into the text font:
	ImFontConfig iconCfg;
	iconCfg.MergeMode = true;
	iconCfg.FontDataOwnedByAtlas = false;  // static array
	iconCfg.GlyphMinAdvanceX = FONT_SIZE;  // monospaced icons
	iconCfg.GlyphOffset.y = 3.0f;
	static const ImWchar iconRanges[] = {ICON_MIN_MS, ICON_MAX_16_MS, 0};
	io.Fonts->AddFontFromMemoryTTF(
		const_cast<void*>(mrpt::imgui_vendor::iconFontData()),
		static_cast<int>(mrpt::imgui_vendor::iconFontDataSize()), FONT_SIZE, &iconCfg, iconRanges);

	ImGui::StyleColorsDark();
	ImGuiStyle& style = ImGui::GetStyle();
	style.WindowRounding = 4.0f;
	style.FrameRounding = 3.0f;
	style.TabRounding = 3.0f;
	style.Colors[ImGuiCol_WindowBg].w = 0.92f;

	// HiDPI monitors:
	float xScale = 1.0f;
	float yScale = 1.0f;
	glfwGetWindowContentScale(win, &xScale, &yScale);
	if (xScale > 1.0f)
	{
		style.ScaleAllSizes(xScale);
		style.FontScaleDpi = xScale;
	}
}

}  // namespace

//!< Return true if the GUI window is open, after a previous call to
//! update_GUI()
bool World::is_GUI_open() const { return gui_.window != nullptr && !gui_.hideRequested; }

//!< Hides the GUI window, if any. The simulation keeps running.
void World::close_GUI()
{
	gui_.hideRequested = true;
	internal_wake_up_gui_thread();
}

void World::internal_wake_up_gui_thread()
{
	std::lock_guard<std::mutex> lck(gui_.windowMtx);
	if (gui_.windowReady)
	{
		// Thread-safe in GLFW:
		glfwPostEmptyEvent();
	}
}

void World::internal_on_gui_input_event()
{
	// Mouse events on a window in the background (e.g. under another one) do
	// not raise the frame rate:
	if (gui_.windowFocused)
	{
		gui_.gotInputEvents = true;
	}
}

void World::internal_on_gui_focus(bool focused) { gui_.windowFocused = focused; }

void World::internal_on_gui_key(int key, int action, int mods)
{
	gui_.gotInputEvents = true;

	if (action != GLFW_PRESS && action != GLFW_REPEAT)
	{
		return;
	}
	// Keys typed into a GUI text box are not for the user application:
	if (ImGui::GetCurrentContext() && ImGui::GetIO().WantCaptureKeyboard)
	{
		return;
	}

	auto lck = mrpt::lockHelper(lastKeyEventMtx_);

	lastKeyEvent_.keycode = key;
	lastKeyEvent_.modifierShift = (mods & GLFW_MOD_SHIFT) != 0;
	lastKeyEvent_.modifierCtrl = (mods & GLFW_MOD_CONTROL) != 0;
	lastKeyEvent_.modifierSuper = (mods & GLFW_MOD_SUPER) != 0;
	lastKeyEvent_.modifierAlt = (mods & GLFW_MOD_ALT) != 0;

	lastKeyEventValid_ = true;
}

void World::internal_apply_initial_camera()
{
	auto& cam = gui_.sceneView->cameraController;

	cam.setProjectiveModel(!guiOptions_.ortho);
	cam.setZoomDistance(static_cast<float>(guiOptions_.camera_distance));
	cam.setAzimuthDegrees(static_cast<float>(guiOptions_.camera_azimuth_deg));
	cam.setElevationDegrees(static_cast<float>(guiOptions_.camera_elevation_deg));
	cam.setFOVdeg(static_cast<float>(guiOptions_.fov_deg));

	const auto p = this->worldRenderOffset() + guiOptions_.camera_point_to;
	cam.setCameraPointing(p);
}

void World::internal_GUI_thread()
{
	// Must outlive the ImGui context, which keeps a pointer to it:
	const std::string iniPath = imgui_ini_path();

	bool imguiReady = false;
	// False if the GUI is closed but the simulation must go on (headless):
	bool closeSimulator = true;

	try
	{
		MRPT_LOG_DEBUG("[World::internal_GUI_thread] Started.");

		if (glfwInit() == GLFW_FALSE)
		{
			THROW_EXCEPTION("glfwInit() failed");
		}

		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, GL_MAJOR);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, GL_MINOR);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
		// Multisampling anti-aliasing: off by default, since it delays the
		// rendering of camera and lidar sensors, which share this OpenGL context.
		// The window exists before the world file is read, hence an environment
		// variable:
		glfwWindowHint(GLFW_SAMPLES, std::max(0, mrpt::get_env<int>("MVSIM_MSAA_SAMPLES", 0)));
		glfwWindowHint(GLFW_DEPTH_BITS, 24);
		// So the scene gamma correction is applied:
		glfwWindowHint(GLFW_SRGB_CAPABLE, GLFW_TRUE);
		glfwWindowHint(GLFW_MAXIMIZED, guiOptions_.start_maximized ? GLFW_TRUE : GLFW_FALSE);

		GLFWwindow* win = glfwCreateWindow(
			static_cast<int>(guiOptions_.win_w), static_cast<int>(guiOptions_.win_h), "mvsim",
			nullptr, nullptr);
		if (!win)
		{
			glfwTerminate();
			THROW_EXCEPTION("glfwCreateWindow() failed");
		}
		{
			std::lock_guard<std::mutex> lck(gui_.windowMtx);
			gui_.window = win;
			gui_.windowReady = true;
		}

		set_window_icon(win);
		if (guiOptions_.start_maximized)
		{
			// Some window managers ignore the GLFW_MAXIMIZED hint:
			glfwMaximizeWindow(win);
		}
		glfwMakeContextCurrent(win);
		// No vsync: this thread paces the frames itself, and must not block
		// while the simulation waits for OpenGL sensors.
		glfwSwapInterval(0);

		glfwSetWindowUserPointer(win, this);
		glfwSetKeyCallback(win, &on_glfw_key);
		glfwSetCursorPosCallback(win, &on_glfw_cursor);
		glfwSetMouseButtonCallback(win, &on_glfw_mouse_button);
		glfwSetScrollCallback(win, &on_glfw_scroll);
		glfwSetWindowFocusCallback(win, &on_glfw_focus);
		gui_.windowFocused = glfwGetWindowAttrib(win, GLFW_FOCUSED) == GLFW_TRUE;

		// Dear ImGui:
		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO();
		io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
		io.IniFilename = iniPath.empty() ? nullptr : iniPath.c_str();

		setup_imgui_fonts_and_style(win);

		ImGui_ImplGlfw_InitForOpenGL(win, true /*install callbacks*/);
		ImGui_ImplOpenGL3_Init("#version 330");
		imguiReady = true;

		gui_.sceneView = std::make_unique<mrpt::imgui::CImGuiSceneView>();

		gui_thread_running_ = true;

		// Show a message until the world is loaded and its first frame rendered:
		while (guiWaitsForWorldLoad_ && !simulator_must_close())
		{
			glfwWaitEventsTimeout(0.05);
			if (glfwWindowShouldClose(win))
			{
				simulator_must_close(true);
			}
			gui_.draw_loading_frame();
		}

		// Closed or failed while loading, or the world file asks for headless mode:
		if (simulator_must_close() || headless())
		{
			closeSimulator = !headless() || simulator_must_close();
			THROW_EXCEPTION("");  // just to clean up
		}

		// Window options from the world file, if they were not known yet:
		if (!guiOptions_.start_maximized && glfwGetWindowAttrib(win, GLFW_MAXIMIZED))
		{
			glfwRestoreWindow(win);
			glfwSetWindowSize(
				win, static_cast<int>(guiOptions_.win_w), static_cast<int>(guiOptions_.win_h));
		}

		// zmin / zmax of opengl viewport:
		worldVisual_->getViewport()->setViewportClipDistances(
			guiOptions_.clip_plane_min, guiOptions_.clip_plane_max);

		// add the placeholders for user-provided objects, both for pure
		// visualization only, and physical objects:
		worldVisual_->insert(glUserObjsViz_);
		worldPhysical_.insert(glUserObjsPhysical_);

		gui_.sceneView->setScene(worldVisual_);

		// Only if the world is empty: at least introduce a ground grid:
		if (worldElements_.empty())
		{
			auto we = WorldElementBase::factory(this, nullptr, "ground_grid");
			worldElements_.push_back(we);
			invalidateElevationIndex();
		}

		// Optionally, start with only the 3D view (and sensor previews), e.g. to record videos:
		if (!guiOptions_.show_gui_panels)
		{
			gui_.showWorld = false;
			gui_.showInspector = false;
			gui_.showLighting = false;
			gui_.showMessages = false;
		}

		internal_apply_initial_camera();

		// The first frame is the slowest one (textures, shaders, shadow
		// maps...): render it while the loading message is still shown.
		internalGraphicsLoopTasksForSimulation();
		gui_.draw_loading_frame();
		gui_.draw_frame();
		worldReadyWallclock_ = mrpt::Clock::nowDouble();

		// Register observation callback:
		const auto lambdaOnObservation =
			[this](const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs)
		{
			this->enqueue_task_to_run_in_gui_thread([this, obs, &veh]()
													{ internal_gui_on_observation(veh, obs); });
		};

		this->registerCallbackOnObservation(lambdaOnObservation);

		// ============= Mainloop =============
		// Frames are drawn at "refresh_fps", or faster while the user
		// interacts with the window. In between, the thread sleeps until a
		// sensor needs OpenGL rendering (see mark_as_pending_running_sensors_on_3D_scene()),
		// and sensors have priority over frames (see the constants above).
		const double idlePeriod = 1.0 / std::max(1, guiOptions_.refresh_fps);
		const double interactivePeriod = std::min(idlePeriod, 1.0 / INTERACTIVE_FPS);
		const double maxOverloadFactor = std::max(1.0, 1.0 / (MIN_OVERLOAD_FPS * idlePeriod));

		MRPT_LOG_DEBUG_FMT(
			"[World::internal_GUI_thread] Using GUI FPS=%i", guiOptions_.refresh_fps);

		double nextFrameTime = 0;
		double lastFrameTime = 0;
		double lastInputTime = mrpt::Clock::nowDouble();
		double overloadFactor = 1.0;
		bool framePostponed = false;

		while (!simulator_must_close())
		{
			const double tWait = nextFrameTime - mrpt::Clock::nowDouble();
			if (tWait > 0)
			{
				glfwWaitEventsTimeout(tWait);
			}
			else
			{
				glfwPollEvents();
			}

			if (glfwWindowShouldClose(win))
			{
				break;
			}

			const double now = mrpt::Clock::nowDouble();
			if (gui_.gotInputEvents.exchange(false))
			{
				lastInputTime = now;
			}
			const bool sensorsPending = pending_running_sensors_on_3D_scene();
			// A frame postponed for the sensors goes right after them:
			const bool frameDue = now >= nextFrameTime || (framePostponed && sensorsPending);

			// Update the 3D scene from the simulation and run sensors that
			// are waiting for OpenGL:
			if (frameDue || sensorsPending)
			{
				internalGraphicsLoopTasksForSimulation();
			}
			if (!frameDue)
			{
				continue;
			}

			const bool interactive = (now - lastInputTime) < INTERACTIVE_HOLD_TIME;
			const double basePeriod = interactive ? interactivePeriod : idlePeriod;
			const bool overloaded = simulation_busy_fraction() > OVERLOAD_BUSY;

			// Postpone the frame if it would delay the next sensors. Right
			// after running sensors, the next ones are far away.
			if (!sensorsPending && now - lastFrameTime < MAX_POSTPONE_PERIODS * basePeriod)
			{
				// Wall-clock time until the next OpenGL sensor, if known:
				std::optional<double> tSensor;
				if (const auto tNext = next_opengl_sensor_time(); tNext.has_value())
				{
					const double rtf = get_realtime_factor_achieved();
					if (rtf > 0.01)
					{
						tSensor = std::max(0.0, (*tNext - get_simul_time()) / rtf);
					}
				}
				// When overloaded, the simulation runs ahead of the real-time
				// factor between sensors, so just wait for them:
				if (tSensor.has_value() &&
					(overloaded || *tSensor < gui_.frameCost + SENSOR_MARGIN))
				{
					framePostponed = true;
					// Retry by the deadline, if the sensors do not come first:
					nextFrameTime = lastFrameTime + MAX_POSTPONE_PERIODS * basePeriod;
					if (!overloaded)
					{
						nextFrameTime = std::min(nextFrameTime, now + *tSensor + SENSOR_MARGIN);
					}
					continue;
				}
			}
			framePostponed = false;

			internal_process_pending_gui_user_tasks();

			// While the simulation can not keep up, give it more time by
			// lowering the frame rate, unless the user interacts:
			if (overloaded)
			{
				overloadFactor = std::min(overloadFactor * OVERLOAD_STEP, maxOverloadFactor);
			}
			else if (simulation_busy_fraction() < RELAXED_BUSY)
			{
				overloadFactor = std::max(1.0, overloadFactor / OVERLOAD_STEP);
			}
			nextFrameTime = now + basePeriod * (interactive ? 1.0 : overloadFactor);
			lastFrameTime = now;

			if (gui_.hideRequested && !gui_.hidden)
			{
				glfwHideWindow(win);
				gui_.hidden = true;
			}
			if (gui_.hidden || glfwGetWindowAttrib(win, GLFW_ICONIFIED))
			{
				continue;
			}

			gui_.draw_frame();
		}

		MRPT_LOG_DEBUG("[World::internal_GUI_thread] Mainloop ended.");
	}
	catch (const std::exception& e)
	{
		if (const auto msg = mrpt::exception_to_str(e);
			!msg.empty() && closeSimulator && !simulator_must_close())
		{
			MRPT_LOG_ERROR_STREAM("[internal_GUI_thread] Exception: " << msg);
		}
	}

	// to let other threads know that we are closing:
	if (closeSimulator)
	{
		simulator_must_close(true);
	}

	// OpenGL resources must be freed from this thread, with its context
	// still alive:
	try
	{
		gui_.free_preview_textures();
		if (gui_.gpuQueries[0][0] != 0)
		{
			glDeleteQueries(4, &gui_.gpuQueries[0][0]);
		}
		gui_.sceneView.reset();

		internalFreeOpenGLResourcesForSimulation();
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM("[internal_GUI_thread] Exception freeing resources: " << e.what());
	}

	if (imguiReady)
	{
		// This also saves the window layout:
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
	}

	GLFWwindow* win = nullptr;
	{
		std::lock_guard<std::mutex> lck(gui_.windowMtx);
		win = gui_.window;
		gui_.window = nullptr;
		gui_.windowReady = false;
	}
	if (win)
	{
		glfwDestroyWindow(win);
		glfwTerminate();
	}

	gui_thread_running_ = false;
}

bool World::GUI::scene_hovered() const
{
#if defined(MRPT_IMGUI_HAS_BACKGROUND_SCENE_VIEW)
	return sceneView && sceneView->isHovered();
#else
	return legacySceneHovered;
#endif
}

std::optional<mrpt::math::TLine3D> World::GUI::scene_mouse_ray() const
{
	if (!sceneView)
	{
		return std::nullopt;
	}
#if defined(MRPT_IMGUI_HAS_BACKGROUND_SCENE_VIEW)
	return sceneView->mouseRay();
#else
	// The FBO image of render() has one pixel per ImGui unit:
	auto scene = sceneView->scene();
	if (!legacySceneHovered || !scene || !scene->getViewport())
	{
		return std::nullopt;
	}
	const ImVec2 m = ImGui::GetMousePos();
	return scene->getViewport()->get3DRayForPixelCoord(
		{static_cast<int>(m.x - legacySceneX), static_cast<int>(m.y - legacySceneY)});
#endif
}

void World::GUI::handle_mouse_operations()
{
	MRPT_START
	if (!sceneView)
	{
		return;
	}

	if (const auto ray = scene_mouse_ray(); ray.has_value())
	{
		// Create a 3D plane, i.e. Z=0
		const auto ground_plane = mrpt::math::TPlane::From3Points({0, 0, 0}, {1, 0, 0}, {0, 1, 0});

		// Intersection of the line with the plane:
		mrpt::math::TObject3D inters;
		mrpt::math::intersect(*ray, ground_plane, inters);

		// Interpret the intersection as a point, if there is an intersection:
		if (inters.getPoint(clickedPt))
		{
			// Apply world offset:
			// P_GL = P_REAL + Off
			// P_REAL = P_GL - Off
			const auto dp = parent_.worldRenderOffset();
			clickedPt.x -= dp.x;
			clickedPt.y -= dp.y;
			clickedPt.z -= dp.z;

			// Find out the "z": get first elevation if many exist.
			const auto zs =
				parent_.getElevationsAt(mrpt::math::TPoint2Df(clickedPt.x, clickedPt.y));
			if (!zs.empty())
			{
				clickedPt.z = *zs.begin();
			}
		}
	}

	// Place the selected object with the mouse, until a click:
	if (placingWithMouse && selected && scene_hovered())
	{
		mrpt::math::TPose3D p = selected->getPose();
		p.x = clickedPt.x;
		p.y = clickedPt.y;
		selected->setPose(p);

		if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
		{
			placingWithMouse = false;
		}
	}

	if (mouseCallback)
	{
		gui::MouseState ms;
		ms.pt = clickedPt;
		ms.over_scene = scene_hovered();
		ms.left_down = ms.over_scene && ImGui::IsMouseDown(ImGuiMouseButton_Left);
		ms.right_down = ms.over_scene && ImGui::IsMouseDown(ImGuiMouseButton_Right);
		try
		{
			mouseCallback(ms);
		}
		catch (const std::exception& e)
		{
			std::cerr << "[mvsim gui] Exception in the mouse callback:\n" << e.what() << std::endl;
		}
	}

	MRPT_END
}

void World::internal_process_pending_gui_user_tasks()
{
	auto tle = mrpt::system::CTimeLoggerEntry(timlogger_, "gui.tasks");

	std::vector<std::function<void(void)>> tasks;
	{
		std::lock_guard<std::mutex> lck(guiUserPendingTasksMtx_);
		tasks = std::move(guiUserPendingTasks_);
		guiUserPendingTasks_.clear();
	}

	// Execute tasks outside the mutex to avoid holding it during callbacks:
	for (const auto& task : tasks) task();
}

void World::internalRunSensorsOn3DScene(mrpt::viz::Scene& physicalObjects)
{
	auto tle = mrpt::system::CTimeLoggerEntry(timlogger_, "internalRunSensorsOn3DScene");

	{
		// User objects are shared with the application, which may modify them:
		const auto lck = mrpt::lockHelper(guiUserObjectsMtx_);
		for (auto& v : vehicles_)
		{
			for (auto& sensor : v.second->getSensors())
			{
				if (sensor)
				{
					sensor->simulateOn3DScene(physicalObjects);
				}
			}
		}
	}

	// clear the flag of pending 3D simulation required:
	clear_pending_running_sensors_on_3D_scene();
}

void World::internalUpdate3DSceneObjects(mrpt::viz::Scene& viz, mrpt::viz::Scene& physical)
{
	// Update view of map elements
	// -----------------------------
	auto tle = mrpt::system::CTimeLoggerEntry(timlogger_, "update_GUI.2.map-elements");

	for (auto& e : worldElements_) e->guiUpdate(viz, physical);

	tle.stop();

	// Update view of vehicles
	// -----------------------------
	timlogger_.enter("update_GUI.3.vehicles");

	for (auto& v : vehicles_) v.second->guiUpdate(viz, physical);

	timlogger_.leave("update_GUI.3.vehicles");

	// Update view of blocks
	// -----------------------------
	timlogger_.enter("update_GUI.4.blocks");

	for (auto& v : blocks_) v.second->guiUpdate(viz, physical);

	timlogger_.leave("update_GUI.4.blocks");

	// Update view of actors
	// -----------------------------
	timlogger_.enter("update_GUI.4b.actors");

	for (auto& a : actors_)
	{
		a.second->guiUpdate(viz, physical);
	}

	timlogger_.leave("update_GUI.4b.actors");

	runtimeObjects_.guiUpdate(viz, physical);

	// Update view of joints
	// -----------------------------
	{
		static const std::string kJointsGlName = "__joint_lines";
		auto glJoints =
			std::dynamic_pointer_cast<mrpt::viz::CSetOfLines>(viz.getByName(kJointsGlName));
		if (!glJoints)
		{
			glJoints = mrpt::viz::CSetOfLines::Create();
			glJoints->setName(kJointsGlName);
			glJoints->setLineWidth(2.0f);
			glJoints->setColor_u8(0xff, 0xcc, 0x00, 0xcc);
			viz.insert(glJoints);
		}
		glJoints->clear();

		for (const auto& jd : joints_)
		{
			if (!jd.b2joint)
			{
				continue;
			}

			const b2Vec2 wA = jd.b2joint->GetAnchorA();
			const b2Vec2 wB = jd.b2joint->GetAnchorB();
			const auto oA = worldRenderOffset();
			const double zDraw = 0.5;

			switch (jd.type)
			{
				case WorldJoint::Type::Distance:
				{
					glJoints->appendLine(
						static_cast<double>(wA.x) + oA.x, static_cast<double>(wA.y) + oA.y, zDraw,
						static_cast<double>(wB.x) + oA.x, static_cast<double>(wB.y) + oA.y, zDraw);
					break;
				}
				case WorldJoint::Type::Revolute:
				{
					glJoints->appendLine(
						static_cast<double>(wA.x) + oA.x, static_cast<double>(wA.y) + oA.y, zDraw,
						static_cast<double>(wB.x) + oA.x, static_cast<double>(wB.y) + oA.y, zDraw);

					// Small cross at midpoint
					const double mx =
						0.5 * (static_cast<double>(wA.x) + static_cast<double>(wB.x)) + oA.x;
					const double my =
						0.5 * (static_cast<double>(wA.y) + static_cast<double>(wB.y)) + oA.y;
					const double cs = 0.15;
					glJoints->appendLine(mx - cs, my, zDraw, mx + cs, my, zDraw);
					glJoints->appendLine(mx, my - cs, zDraw, mx, my + cs, zDraw);
					break;
				}
			}
		}
	}

	// Camera follow modes:
	// -----------------------
	if (gui_.sceneView && !guiOptions_.follow_vehicle.empty())
	{
		if (auto it = vehicles_.find(guiOptions_.follow_vehicle); it != vehicles_.end())
		{
			const auto pose = it->second->getCPose3D();
			const auto p = applyWorldRenderOffset(pose);
			gui_.sceneView->cameraController.setCameraPointing(
				static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
		}
		else
		{
			MRPT_LOG_THROTTLE_ERROR_FMT(
				5.0,
				"GUI: Camera set to follow vehicle named '%s' which can't be "
				"found!",
				guiOptions_.follow_vehicle.c_str());
		}
	}
}

void World::open_GUI_while_loading()
{
	if (headless())
	{
		return;
	}
	auto lock = mrpt::lockHelper(gui_thread_start_mtx_);
	if (gui_thread_.joinable())
	{
		return;
	}
	guiWaitsForWorldLoad_ = true;
	gui_thread_ = std::thread(&World::internal_GUI_thread, this);
	mrpt::system::thread_name("guiThread", gui_thread_);
}

void World::update_GUI(TUpdateGUIParams* guiparams)
{
	// First call?
	// -----------------------
	{
		auto lock = mrpt::lockHelper(gui_thread_start_mtx_);
		if (!gui_thread_running_ && !gui_thread_.joinable())
		{
			MRPT_LOG_DEBUG("[update_GUI] Launching GUI thread...");

			gui_thread_ = std::thread(&World::internal_GUI_thread, this);
			mrpt::system::thread_name("guiThread", gui_thread_);

			const int MVSIM_OPEN_GUI_TIMEOUT_MS =
				mrpt::get_env<int>("MVSIM_OPEN_GUI_TIMEOUT_MS", 3000);

			for (int timeout = 0; timeout < MVSIM_OPEN_GUI_TIMEOUT_MS / 10; timeout++)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				if (gui_thread_running_) break;
			}

			if (!gui_thread_running_)
			{
				THROW_EXCEPTION("Timeout waiting for GUI to open!");
			}
			else
			{
				MRPT_LOG_DEBUG("[update_GUI] GUI thread started.");
			}
		}
	}

	if (!is_GUI_open())
	{
		MRPT_LOG_THROTTLE_WARN(
			5.0,
			"[World::update_GUI] GUI window has been closed, but note that "
			"simulation keeps running.");
		return;
	}

	timlogger_.enter("update_GUI");	 // Don't count initialization, since that
									 // is a total outlier and lacks interest!

	// guiparams is optional (defaults to nullptr): only copy the caller's
	// message lines when they actually passed a params struct.
	if (guiparams)
	{
		guiMsgLinesMtx_.lock();
		guiMsgLines_ = guiparams->msg_lines;
		guiMsgLinesMtx_.unlock();
	}

	timlogger_.leave("update_GUI");

	// Key-strokes:
	// -----------------------
	if (guiparams && lastKeyEventValid_)
	{
		auto lck = mrpt::lockHelper(lastKeyEventMtx_);

		guiparams->keyevent = std::move(lastKeyEvent_);
		lastKeyEventValid_ = false;
	}
}

// This method is ensured to be run in the GUI thread
void World::internal_gui_on_observation(
	const Simulable& veh, const mrpt::obs::CObservation::Ptr& obs)
{
	if (!obs || !guiOptions_.show_sensor_previews)
	{
		return;
	}
	if (auto obs3D = std::dynamic_pointer_cast<mrpt::obs::CObservation3DRangeScan>(obs); obs3D)
	{
		internal_gui_on_observation_3Dscan(veh, obs3D);
	}
	else if (auto obsIm = std::dynamic_pointer_cast<mrpt::obs::CObservationImage>(obs); obsIm)
	{
		internal_gui_on_observation_image(veh, obsIm);
	}
}

const SensorBase* World::internal_gui_find_sensor(
	const Simulable& veh, const std::string& sensorLabel)
{
	const auto* vehPtr = dynamic_cast<const VehicleBase*>(&veh);
	if (!vehPtr)
	{
		return nullptr;
	}
	for (const auto& s : vehPtr->getSensors())
	{
		if (s && s->getName() == sensorLabel)
		{
			return s.get();
		}
	}
	return nullptr;
}

void World::internal_gui_on_observation_3Dscan(
	const Simulable& veh, const std::shared_ptr<mrpt::obs::CObservation3DRangeScan>& obs)
{
	using namespace std::string_literals;

	if (!obs)
	{
		return;
	}
	const auto* sensor = internal_gui_find_sensor(veh, obs->sensorLabel);
	const bool startVisible = !sensor || sensor->previewWinVisible();
	const auto name = veh.getName() + "/"s + obs->sensorLabel;

	if (obs->hasIntensityImage && gui_.preview_needs_update(name, 0))
	{
		gui_.update_preview_texture(name, 0, obs->intensityImage, startVisible);
	}
	if (obs->hasRangeImage && (!sensor || sensor->previewDepth()) &&
		gui_.preview_needs_update(name, 1))
	{
		mrpt::math::CMatrixFloat d;
		d = obs->rangeImage.asEigen().cast<float>() * (obs->rangeUnits / obs->maxRange);

		mrpt::img::CImage imDepth;
		imDepth.setFromMatrix(d, true /* in range [0,1] */);

		gui_.update_preview_texture(name, 1, imDepth, startVisible);
	}
}

void World::internal_gui_on_observation_image(
	const Simulable& veh, const std::shared_ptr<mrpt::obs::CObservationImage>& obs)
{
	using namespace std::string_literals;

	if (!obs || obs->image.isEmpty())
	{
		return;
	}
	const auto* sensor = internal_gui_find_sensor(veh, obs->sensorLabel);
	const bool startVisible = !sensor || sensor->previewWinVisible();
	const auto name = veh.getName() + "/"s + obs->sensorLabel;

	if (gui_.preview_needs_update(name, 0))
	{
		gui_.update_preview_texture(name, 0, obs->image, startVisible);
	}
}

void World::internalFreeOpenGLResourcesForSimulation()
{
	auto lckListObjs = mrpt::lockHelper(getListOfSimulableObjectsMtx());
	for (auto& obj : getListOfSimulableObjects())
	{
		obj.second->freeOpenGLResources();
	}
}

void World::internalGraphicsLoopTasksForSimulation()
{
	try
	{
		// Update all GUI elements:
		ASSERT_(worldVisual_);

		auto lckPhys = mrpt::lockHelper(physical_objects_mtx());

		internalProcessRemovedEntitiesInGui();

		internalUpdate3DSceneObjects(*worldVisual_, worldPhysical_);

		internalRunSensorsOn3DScene(worldPhysical_);

		lckPhys.unlock();

		if (headless() && worldReadyWallclock_ == 0)
		{
			worldReadyWallclock_ = mrpt::Clock::nowDouble();
		}

		// handle user custom 3D visual objects:
		{
			const auto lck = mrpt::lockHelper(guiUserObjectsMtx_);
			// replace list of smart pointers (fast):
			if (guiUserObjectsPhysical_) *glUserObjsPhysical_ = *guiUserObjectsPhysical_;
			if (guiUserObjectsViz_) *glUserObjsViz_ = *guiUserObjectsViz_;
		}
	}
	catch (const std::exception& e)
	{
		// In case of an exception in the functions above,
		// abort. Otherwise, the error may repeat over and over forever
		// and the main thread will never know about it.
		MRPT_LOG_ERROR(e.what());
		// Clear this flag so the simulation thread's busy-wait can exit:
		clear_pending_running_sensors_on_3D_scene();
		simulator_must_close(true);
	}
}

void World::applyLightOptions()
{
	const auto& lo = lightOptions_;

	setLightDirectionFromAzimuthElevation(lo.light_azimuth, lo.light_elevation);

	auto vv = worldVisual_->getViewport();
	auto vp = worldPhysical_.getViewport();

	const auto extraLights = pointAndSpotLightsForRendering();

	auto lambdaSetLightParams = [&lo, &extraLights](const mrpt::viz::Viewport::Ptr& v)
	{
		// enable shadows and set the shadow map texture size:
		const int sms = lo.shadow_map_size;
		v->enableShadowCasting(lo.enable_shadows, sms, sms);

		// light color and intensities:
		const auto colf = mrpt::img::TColorf(lo.light_color);

		auto& vlp = v->lightParameters();

		if (!vlp.lights.empty())
		{
			vlp.lights[0].color = colf;
			vlp.lights[0].diffuse = lo.light_diffuse;
			vlp.lights[0].specular = lo.light_specular;
		}

		// Hemisphere ambient lighting (replaces fill light):
		vlp.ambient = lo.light_ambient;
		vlp.ambientSkyColor = mrpt::img::TColorf(lo.ambient_sky_color);
		vlp.ambientGroundColor = mrpt::img::TColorf(lo.ambient_ground_color);

		// Add extra lights (point and spot) from XML:
		for (const auto& el : extraLights)
		{
			vlp.lights.push_back(el);
		}

		vlp.eyeDistance2lightShadowExtension = lo.eye_distance_to_shadow_map_extension;

		vlp.minimum_shadow_map_extension_ratio = lo.minimum_shadow_map_extension_ratio;
		vlp.shadow_cascades = static_cast<uint8_t>(lo.shadow_cascades);
		// light view frustrum near/far planes:
		v->setLightShadowClipDistances(lo.light_clip_plane_min, lo.light_clip_plane_max);

		// Shadow bias should be proportional to clip range:
		vlp.shadow_bias = lo.shadow_bias;
		vlp.shadow_bias_cam2frag = lo.shadow_bias_cam2frag;
		vlp.shadow_bias_normal = lo.shadow_bias_normal;
	};

	lambdaSetLightParams(vv);
	lambdaSetLightParams(vp);
}

std::vector<mrpt::viz::TLight> World::pointAndSpotLightsForRendering() const
{
	// In rendering coordinates, like everything else sent to OpenGL:
	const auto renderOffset = worldRenderOffset();

	auto lights = lightOptions_.extra_lights;
	for (auto& l : lights)
	{
		l.position.x += static_cast<float>(renderOffset.x);
		l.position.y += static_cast<float>(renderOffset.y);
		l.position.z += static_cast<float>(renderOffset.z);
	}
	return lights;
}

void World::setPointAndSpotLightsEnabled(const bool enabled)
{
	ASSERT_(worldVisual_);

	auto lckPhys = mrpt::lockHelper(physical_objects_mtx());

	pointAndSpotLightsEnabled_ = enabled;
	const auto extraLights = pointAndSpotLightsForRendering();

	for (const auto& v : {worldVisual_->getViewport(), worldPhysical_.getViewport()})
	{
		// Keep the directional light only:
		auto& lights = v->lightParameters().lights;
		lights.resize(std::min<size_t>(lights.size(), 1));
		if (enabled)
		{
			lights.insert(lights.end(), extraLights.begin(), extraLights.end());
		}
	}
}

void World::setLightAmbient(const float ambient)
{
	ASSERT_(worldVisual_);

	auto lckPhys = mrpt::lockHelper(physical_objects_mtx());

	lightOptions_.light_ambient = ambient;
	worldVisual_->getViewport()->lightParameters().ambient = ambient;
	worldPhysical_.getViewport()->lightParameters().ambient = ambient;
}

void World::setLightIntensityFactor(const float factor)
{
	ASSERT_(worldVisual_);

	auto lckPhys = mrpt::lockHelper(physical_objects_mtx());

	for (const auto& v : {worldVisual_->getViewport(), worldPhysical_.getViewport()})
	{
		auto& lights = v->lightParameters().lights;
		if (!lights.empty())
		{
			lights[0].diffuse = factor * lightOptions_.light_diffuse;
			lights[0].specular = factor * lightOptions_.light_specular;
		}
	}
}

void World::setLightDirectionFromAzimuthElevation(const float azimuth, const float elevation)
{
	const mrpt::math::TPoint3Df dir = {
		-cos(azimuth) * cos(elevation), -sin(azimuth) * cos(elevation), -sin(elevation)};

	ASSERT_(worldVisual_);

	auto lckPhys = mrpt::lockHelper(physical_objects_mtx());

	auto vv = worldVisual_->getViewport();
	auto vp = worldPhysical_.getViewport();

	if (!vv->lightParameters().lights.empty()) vv->lightParameters().lights[0].direction = dir;
	if (!vp->lightParameters().lights.empty()) vp->lightParameters().lights[0].direction = dir;
}
