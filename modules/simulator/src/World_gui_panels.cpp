/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Dear ImGui panels of the mvsim GUI. The window layout (docking, sizes) is
// autosaved by ImGui, see imgui_ini_path() in World_gui.cpp.

#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/opengl_api.h>
#include <mrpt/system/datetime.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#include <mvsim/mvsim_version.h>

// clang-format off
#include <imgui.h>
#include <imgui_internal.h>  // DockBuilder*, BeginViewportSideBar()
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <misc/cpp/imgui_stdlib.h>
#include <IconsMaterialSymbols.h>
#include <portable-file-dialogs.h>
#include <mrpt/imgui/CImGuiSceneView.h>
#include <GLFW/glfw3.h>
// clang-format on

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <iostream>
#include <map>
#include <variant>

using namespace mvsim;

namespace
{
// Window titles. The text after "###" is the ImGui ID, which must not change
// so the saved layout keeps applying.
constexpr const char* WIN_WORLD = ICON_MS_LAYERS " World###World";
constexpr const char* WIN_INSPECTOR = ICON_MS_TUNE " Inspector###Inspector";
constexpr const char* WIN_LIGHTING = ICON_MS_BLUR_ON " Lighting###Lighting";
constexpr const char* WIN_MESSAGES = ICON_MS_TERMINAL " Messages###Messages";
constexpr const char* PREVIEW_ID_PREFIX = "###preview:";

std::string preview_window_title(const std::string& name)
{
	return std::string(ICON_MS_PHOTO_CAMERA " ") + name + PREVIEW_ID_PREFIX + name;
}

bool contains_case_insensitive(const std::string& haystack, const std::string& needle)
{
	if (needle.empty())
	{
		return true;
	}
	const auto it = std::search(
		haystack.begin(), haystack.end(), needle.begin(), needle.end(),
		[](char a, char b)
		{
			return std::tolower(static_cast<unsigned char>(a)) ==
				   std::tolower(static_cast<unsigned char>(b));
		});
	return it != haystack.end();
}

ImTextureRef as_imgui_texture(unsigned int glTexture)
{
	return ImTextureRef(static_cast<ImTextureID>(glTexture));
}

void call_user_callback(const std::function<void()>& f)
{
	try
	{
		f();
	}
	catch (const std::exception& e)
	{
		std::cerr << "[mvsim gui] Exception in a custom panel callback:\n" << e.what() << std::endl;
	}
}

// Draws one widget of a custom panel. `id` must be unique within the window.
void draw_user_widget(
	const gui::AnyWidget& any, const std::string& id, std::map<std::string, bool>& checkStates)
{
	std::visit(
		[&](const auto& w)
		{
			using T = std::decay_t<decltype(w)>;
			if constexpr (std::is_same_v<T, gui::Row>)
			{
				for (size_t i = 0; i < w.widgets.size(); i++)
				{
					if (i > 0)
					{
						ImGui::SameLine();
					}
					// Row members are leaf widgets, a subset of AnyWidget:
					std::visit(
						[&](const auto& leaf)
						{ draw_user_widget(leaf, id + "." + std::to_string(i), checkStates); },
						w.widgets[i]);
				}
			}
			else if constexpr (std::is_same_v<T, gui::Label>)
			{
				if (w.text)
				{
					w.text->poll_into_display();
					ImGui::TextUnformatted(w.text->display.c_str());
				}
			}
			else if constexpr (std::is_same_v<T, gui::Separator>)
			{
				ImGui::Separator();
			}
			else if constexpr (std::is_same_v<T, gui::CheckBox>)
			{
				auto [it, isNew] = checkStates.try_emplace(id, w.initial_value);
				if (ImGui::Checkbox((w.label + "##" + id).c_str(), &it->second) && w.on_change)
				{
					call_user_callback([&]() { w.on_change(it->second); });
				}
			}
			else if constexpr (std::is_same_v<T, gui::Button>)
			{
				if (ImGui::Button((w.label + "##" + id).c_str()) && w.on_click)
				{
					call_user_callback(w.on_click);
				}
			}
			else if constexpr (std::is_same_v<T, gui::TextBox>)
			{
				if (!w.live_value)
				{
					return;
				}
				if (!w.label.empty())
				{
					ImGui::TextUnformatted(w.label.c_str());
				}
				w.live_value->poll_into_display();
				ImGui::SetNextItemWidth(-FLT_MIN);
				if (ImGui::InputText(("##" + id).c_str(), &w.live_value->display) && w.on_change)
				{
					call_user_callback([&]() { w.on_change(w.live_value->display); });
				}
			}
		},
		any);
}

}  // namespace

World::GUI::GUI(World& parent) : parent_(parent) {}

World::GUI::~GUI() = default;

void World::add_gui_panel(const gui::WindowDescription& panel)
{
	// The list of panels is only accessed from the GUI thread:
	enqueue_task_to_run_in_gui_thread(
		[this, panel]()
		{
			GUI::UserPanel p;
			p.desc = panel;
			p.id = panel.title;
			const auto nSameTitle = std::count_if(
				gui_.userPanels.begin(), gui_.userPanels.end(),
				[&](const GUI::UserPanel& o) { return o.desc.title == panel.title; });
			if (nSameTitle > 0)
			{
				p.id += "#" + std::to_string(nSameTitle);
			}
			gui_.userPanels.push_back(std::move(p));
		});
}

void World::set_gui_mouse_callback(const std::function<void(const gui::MouseState&)>& callback)
{
	enqueue_task_to_run_in_gui_thread([this, callback]() { gui_.mouseCallback = callback; });
}

void World::GUI::dock_new_window_right(const std::string& title)
{
	// New windows, without saved settings, go to the right column:
	if (dockRightId_ == 0 && !ImGui::FindWindowSettingsByID(ImHashStr(title.c_str())))
	{
		if (const ImGuiDockNode* central = ImGui::DockBuilderGetCentralNode(dockspaceId_); central)
		{
			ImGuiID centralId = central->ID;
			dockRightId_ =
				ImGui::DockBuilderSplitNode(centralId, ImGuiDir_Right, 0.28f, nullptr, &centralId);
			ImGui::DockBuilderFinish(dockspaceId_);
		}
	}
	if (dockRightId_ != 0)
	{
		ImGui::SetNextWindowDockID(dockRightId_, ImGuiCond_FirstUseEver);
	}
}

void World::GUI::draw_user_panels()
{
	for (auto& p : userPanels)
	{
		if (!p.open)
		{
			continue;
		}
		const auto& d = p.desc;

		ImGui::SetNextWindowSize(
			ImVec2(static_cast<float>(d.size[0]), static_cast<float>(d.size[1])),
			ImGuiCond_FirstUseEver);
		const std::string winTitle = d.title + "###user:" + p.id;
		dock_new_window_right(winTitle);
		if (!ImGui::Begin(winTitle.c_str(), &p.open))
		{
			ImGui::End();
			continue;
		}

		const auto drawTab = [&](const gui::Tab& tab, size_t tabIdx)
		{
			for (size_t i = 0; i < tab.widgets.size(); i++)
			{
				draw_user_widget(
					tab.widgets[i], p.id + "/" + std::to_string(tabIdx) + "/" + std::to_string(i),
					p.checkStates);
			}
		};

		if (d.tabs.size() == 1)
		{
			drawTab(d.tabs.front(), 0);
		}
		else if (ImGui::BeginTabBar("##tabs"))
		{
			for (size_t t = 0; t < d.tabs.size(); t++)
			{
				if (ImGui::BeginTabItem(d.tabs[t].title.c_str()))
				{
					drawTab(d.tabs[t], t);
					ImGui::EndTabItem();
				}
			}
			ImGui::EndTabBar();
		}
		ImGui::End();
	}
}

void World::GUI::select(const std::string& name, const Simulable::Ptr& obj)
{
	// Deselect the former one:
	if (selectedVisual && !showCollisionShapes)
	{
		selectedVisual->showCollisionShape(false);
	}
	selected = obj;
	selectedName = name;
	selectedVisual = dynamic_cast<CVisualObject*>(obj.get());
	placingWithMouse = false;

	// Highlight the selected one:
	if (selectedVisual)
	{
		selectedVisual->showCollisionShape(true);
	}
}

void World::GUI::refresh_objects_snapshot()
{
	std::unique_lock<std::mutex> lck(parent_.getListOfSimulableObjectsMtx(), std::try_to_lock);
	if (!lck.owns_lock())
	{
		return;	 // keep the former snapshot
	}

	ObjectsSnapshot snap;
	for (const auto& o : parent_.getListOfSimulableObjects())
	{
		auto* ptr = o.second.get();
		if (dynamic_cast<VehicleBase*>(ptr))
		{
			snap.vehicles.emplace_back(o);
		}
		else if (dynamic_cast<Block*>(ptr))
		{
			snap.blocks.emplace_back(o);
		}
		else if (dynamic_cast<HumanActor*>(ptr))
		{
			snap.actors.emplace_back(o);
		}
		else if (dynamic_cast<WorldElementBase*>(ptr))
		{
			snap.elements.emplace_back(o);
		}

		if (auto* visual = dynamic_cast<CVisualObject*>(ptr); visual)
		{
			for (const auto& group : visual->lightGroupNames())
			{
				snap.lightGroups.push_back({o.first + ": " + group, o.second, visual, group});
			}
		}
	}
	objects = std::move(snap);
}

void World::GUI::draw_loading_frame()
{
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	const ImGuiViewport* vp = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos(vp->GetCenter(), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
	ImGui::Begin(
		"##loading", nullptr,
		ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
			ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoMove);
	ImGui::SetWindowFontScale(1.6f);
	ImGui::TextUnformatted("Loading the world...");
	ImGui::End();

	ImGui::Render();
	int w = 0;
	int h = 0;
	glfwGetFramebufferSize(window, &w, &h);
	glViewport(0, 0, w, h);
	glClearColor(0.15f, 0.15f, 0.17f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	glfwSwapBuffers(window);
}

void World::GUI::draw_frame()
{
	auto tle = mrpt::system::CTimeLoggerEntry(parent_.timlogger_, "gui.frame");
	const double tStart = mrpt::Clock::nowDouble();

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

	refresh_objects_snapshot();

	// Order matters: the menu and status bars reduce the work area used by
	// the dockspace.
	draw_menu_bar();
	draw_status_bar();
	draw_dockspace_and_background();

	if (showWorld)
	{
		draw_world_panel();
	}
	if (showInspector)
	{
		draw_inspector_panel();
	}
	if (showLighting)
	{
		draw_lighting_panel();
	}
	if (showMessages)
	{
		draw_messages_panel();
	}
	draw_sensor_previews();
	draw_user_panels();

	handle_mouse_operations();

	ImGui::Render();

	int w = 0;
	int h = 0;
	glfwGetFramebufferSize(window, &w, &h);
	glViewport(0, 0, w, h);
	glClearColor(0.15f, 0.15f, 0.17f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Measure the GPU time of frames, with two alternating pairs of time
	// stamp queries. Results are read when available, never waited for.
	if (gpuQueries[0][0] == 0)
	{
		glGenQueries(4, &gpuQueries[0][0]);
	}
	gpuQueryIdx = 1 - gpuQueryIdx;
	const auto& q = gpuQueries[gpuQueryIdx];
	bool canQuery = true;
	if (gpuQueriesIssued[gpuQueryIdx])
	{
		GLint available = 0;
		glGetQueryObjectiv(q[1], GL_QUERY_RESULT_AVAILABLE, &available);
		canQuery = available != 0;
		if (canQuery)
		{
			GLuint64 tBegin = 0;
			GLuint64 tEnd = 0;
			glGetQueryObjectui64v(q[0], GL_QUERY_RESULT, &tBegin);
			glGetQueryObjectui64v(q[1], GL_QUERY_RESULT, &tEnd);
			lastGpuFrameTime = 1e-9 * static_cast<double>(tEnd - tBegin);
		}
	}
	if (canQuery)
	{
		glQueryCounter(q[0], GL_TIMESTAMP);
	}

	// The 3D scene is rendered from within here, behind all windows. User
	// objects are shared with the application, which may modify them:
	{
		const auto lck = mrpt::lockHelper(parent_.guiUserObjectsMtx_);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}

	if (canQuery)
	{
		glQueryCounter(q[1], GL_TIMESTAMP);
		gpuQueriesIssued[gpuQueryIdx] = true;
	}

	glfwSwapBuffers(window);

	// Conservative: CPU and GPU work only partially overlap.
	constexpr double alpha = 0.2;
	const double cpuTime = mrpt::Clock::nowDouble() - tStart;
	frameCost = (1.0 - alpha) * frameCost + alpha * (cpuTime + lastGpuFrameTime);
}

void World::GUI::draw_menu_bar()
{
	if (!ImGui::BeginMainMenuBar())
	{
		return;
	}

	if (ImGui::BeginMenu("File"))
	{
		if (ImGui::MenuItem(ICON_MS_SAVE " Save 3D scene..."))
		{
			try
			{
				const std::string outFile = pfd::save_file(
												"Save 3D scene", "world.3Dscene",
												{"MRPT 3D scene files (*.3Dscene)", "*.3Dscene"})
												.result();
				if (!outFile.empty())
				{
					auto lck = mrpt::lockHelper(parent_.physical_objects_mtx());
					parent_.worldPhysical_.saveToFile(outFile);
					std::cout << "[mvsim gui] Saved world scene to: " << outFile << std::endl;
				}
			}
			catch (const std::exception& e)
			{
				std::cerr << "[mvsim gui] Exception while saving 3D scene:\n"
						  << e.what() << std::endl;
			}
		}
		ImGui::Separator();
		if (ImGui::MenuItem(ICON_MS_CLOSE " Quit"))
		{
			parent_.simulator_must_close(true);
		}
		ImGui::EndMenu();
	}

	if (ImGui::BeginMenu("View"))
	{
		auto& opts = parent_.guiOptions_;

		if (ImGui::BeginMenu(ICON_MS_VIDEOCAM " Camera follows"))
		{
			if (ImGui::MenuItem("(none)", nullptr, opts.follow_vehicle.empty()))
			{
				opts.follow_vehicle.clear();
			}
			for (const auto& v : parent_.vehicles_)
			{
				if (ImGui::MenuItem(v.first.c_str(), nullptr, opts.follow_vehicle == v.first))
				{
					opts.follow_vehicle = v.first;
				}
			}
			ImGui::EndMenu();
		}
		if (ImGui::MenuItem(ICON_MS_CENTER_FOCUS_STRONG " Reset camera"))
		{
			parent_.internal_apply_initial_camera();
		}
		if (ImGui::MenuItem("Orthogonal view", nullptr, &opts.ortho))
		{
			sceneView->cameraController.setProjectiveModel(!opts.ortho);
		}

		ImGui::Separator();
		ImGui::MenuItem("Forces", nullptr, &opts.show_forces);
		ImGui::MenuItem("Trajectories", nullptr, &opts.show_trajectories);
		if (ImGui::MenuItem("Sensor point clouds", nullptr, &opts.show_sensor_points))
		{
			if (auto glVizSensors = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(
					parent_.worldVisual_->getByName("group_sensors_viz"));
				glVizSensors)
			{
				glVizSensors->setVisibility(opts.show_sensor_points);
			}
		}
		if (ImGui::MenuItem("Sensor poses", nullptr, &showSensorPoses))
		{
			for (const auto& o : *SensorBase::GetAllSensorsOriginViz())
			{
				o->setVisibility(showSensorPoses);
			}
		}
		if (ImGui::MenuItem("Sensor FOVs", nullptr, &showSensorFOVs))
		{
			for (const auto& o : *SensorBase::GetAllSensorsFOVViz())
			{
				o->setVisibility(showSensorFOVs);
			}
		}
		if (ImGui::MenuItem("Collision shapes", nullptr, &showCollisionShapes))
		{
			auto lck = mrpt::lockHelper(parent_.simulableObjectsMtx_);
			for (auto& s : parent_.simulableObjects_)
			{
				if (auto* vis = dynamic_cast<CVisualObject*>(s.second.get()); vis)
				{
					vis->showCollisionShape(showCollisionShapes || vis == selectedVisual);
				}
			}
		}
		ImGui::EndMenu();
	}

	if (ImGui::BeginMenu("Window"))
	{
		ImGui::MenuItem(WIN_WORLD, nullptr, &showWorld);
		ImGui::MenuItem(WIN_INSPECTOR, nullptr, &showInspector);
		ImGui::MenuItem(WIN_LIGHTING, nullptr, &showLighting);
		ImGui::MenuItem(WIN_MESSAGES, nullptr, &showMessages);
		for (auto& p : userPanels)
		{
			ImGui::MenuItem((p.desc.title + "###menu:" + p.id).c_str(), nullptr, &p.open);
		}
		if (ImGui::BeginMenu(ICON_MS_PHOTO_CAMERA " Sensor previews", !sensorPreviews.empty()))
		{
			for (auto& [name, p] : sensorPreviews)
			{
				ImGui::MenuItem(name.c_str(), nullptr, &p.open);
			}
			ImGui::EndMenu();
		}
		ImGui::Separator();
		if (ImGui::MenuItem(ICON_MS_DASHBOARD " Reset layout"))
		{
			resetLayoutRequested = true;
		}
		ImGui::EndMenu();
	}

	if (ImGui::BeginMenu("Help"))
	{
		ImGui::TextDisabled("MVSim " MVSIM_VERSION);
		ImGui::TextDisabled("Mouse: left=orbit, right/middle=pan, wheel=zoom");
		ImGui::TextDisabled("Ctrl+click on a numeric field to type a value.");
		ImGui::EndMenu();
	}

	ImGui::EndMainMenuBar();
}

void World::GUI::draw_status_bar()
{
	ImGuiViewport* vp = ImGui::GetMainViewport();
	const float height = ImGui::GetFrameHeight();
	constexpr ImGuiWindowFlags flags =
		ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar;

	if (ImGui::BeginViewportSideBar("##mvsim_status_bar", vp, ImGuiDir_Down, height, flags))
	{
		if (ImGui::BeginMenuBar())
		{
			ImGui::Text(
				ICON_MS_ACCESS_TIME " %s",
				mrpt::system::formatTimeInterval(parent_.get_simul_time()).c_str());
			ImGui::Separator();

			const double cpu = parent_.cpu_usage();
			if (cpu > 1.0)
			{
				ImGui::TextColored(
					ImVec4(1.0f, 0.4f, 0.4f, 1.0f), ICON_MS_MEMORY " CPU %.01f%%", cpu * 100.0);
			}
			else
			{
				ImGui::Text(ICON_MS_MEMORY " CPU %.01f%%", cpu * 100.0);
			}
			ImGui::Separator();
			ImGui::Text(ICON_MS_SPEED " %.03fx real time", parent_.get_realtime_factor_achieved());
			ImGui::Separator();
			ImGui::Text(
				ICON_MS_MOUSE " (%.02f, %.02f, %.02f)", clickedPt.x, clickedPt.y, clickedPt.z);
			if (placingWithMouse)
			{
				ImGui::Separator();
				ImGui::TextColored(
					ImVec4(1.0f, 0.85f, 0.3f, 1.0f), ICON_MS_ADS_CLICK " Click to place '%s'",
					selectedName.c_str());
			}
			ImGui::EndMenuBar();
		}
	}
	ImGui::End();
}

void World::GUI::build_default_layout()
{
	const ImGuiViewport* vp = ImGui::GetMainViewport();

	ImGui::DockBuilderRemoveNode(dockspaceId_);
	ImGui::DockBuilderAddNode(
		dockspaceId_, ImGuiDockNodeFlags_DockSpace | ImGuiDockNodeFlags_PassthruCentralNode);
	ImGui::DockBuilderSetNodeSize(dockspaceId_, vp->WorkSize);

	// Left column: world tree on top, inspector/lighting tabs, messages at
	// the bottom. Right column: sensor previews, as tabs.
	ImGuiID center = dockspaceId_;
	ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.22f, nullptr, &center);
	dockRightId_ = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.28f, nullptr, &center);
	dockLeftBottomId_ = ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.22f, nullptr, &left);
	dockLeftMiddleId_ = ImGui::DockBuilderSplitNode(left, ImGuiDir_Down, 0.50f, nullptr, &left);
	dockLeftTopId_ = left;

	ImGui::DockBuilderDockWindow(WIN_WORLD, dockLeftTopId_);
	ImGui::DockBuilderDockWindow(WIN_INSPECTOR, dockLeftMiddleId_);
	ImGui::DockBuilderDockWindow(WIN_LIGHTING, dockLeftMiddleId_);
	ImGui::DockBuilderDockWindow(WIN_MESSAGES, dockLeftBottomId_);
	for (const auto& p : sensorPreviews)
	{
		ImGui::DockBuilderDockWindow(preview_window_title(p.first).c_str(), dockRightId_);
	}
	ImGui::DockBuilderFinish(dockspaceId_);
}

void World::GUI::draw_dockspace_and_background()
{
	const ImGuiViewport* vp = ImGui::GetMainViewport();

	dockspaceId_ = ImGui::GetID("mvsim_dockspace");
	// No saved layout yet (first run), or the user asked for the default one:
	if (resetLayoutRequested || !ImGui::DockBuilderGetNode(dockspaceId_))
	{
		resetLayoutRequested = false;
		build_default_layout();
	}
	ImGui::DockSpaceOverViewport(dockspaceId_, vp, ImGuiDockNodeFlags_PassthruCentralNode);

	// The 3D view fills the central (empty) dock area only: no pixels are
	// rendered under docked panels, and the camera centers on the visible
	// region.
	ImVec2 pos = vp->WorkPos;
	ImVec2 size = vp->WorkSize;
	if (const ImGuiDockNode* central = ImGui::DockBuilderGetCentralNode(dockspaceId_); central)
	{
		pos = central->Pos;
		size = central->Size;
	}
	ImGui::SetNextWindowPos(pos);
	ImGui::SetNextWindowSize(size);
	ImGui::SetNextWindowBgAlpha(0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	constexpr ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
									   ImGuiWindowFlags_NoBringToFrontOnFocus |
									   ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoDocking |
									   ImGuiWindowFlags_NoSavedSettings |
									   ImGuiWindowFlags_NoScrollWithMouse;
	ImGui::Begin("##mvsim_3d_view", nullptr, flags);
	ImGui::PopStyleVar(2);
#if defined(MRPT_IMGUI_HAS_BACKGROUND_SCENE_VIEW)
	sceneView->renderAsBackground();
#else
	// Older MRPT: rendered into an FBO and shown as an image (an extra copy,
	// and no multisampling). Its last item is the input-capturing button:
	sceneView->render();
	legacySceneHovered = ImGui::IsItemHovered();
	legacySceneX = ImGui::GetItemRectMin().x;
	legacySceneY = ImGui::GetItemRectMin().y;
#endif
	ImGui::End();
}

void World::GUI::draw_world_panel()
{
	if (!ImGui::Begin(WIN_WORLD, &showWorld))
	{
		ImGui::End();
		return;
	}

	ImGui::SetNextItemWidth(-FLT_MIN);
	ImGui::InputTextWithHint("##filter", ICON_MS_SEARCH " Filter by name", &worldFilter);

	const auto& vehicles = objects.vehicles;
	const auto& blocks = objects.blocks;
	const auto& actors = objects.actors;
	const auto& elements = objects.elements;

	const auto lambdaLeaf =
		[this](const std::string& name, const std::string& label, const Simulable::Ptr& obj)
	{
		ImGuiTreeNodeFlags fl = ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen |
								ImGuiTreeNodeFlags_SpanAvailWidth;
		if (selected == obj)
		{
			fl |= ImGuiTreeNodeFlags_Selected;
		}
		// Names may be empty or repeated, so use the object as ImGui ID:
		ImGui::TreeNodeEx(static_cast<const void*>(obj.get()), fl, "%s", label.c_str());
		if (ImGui::IsItemClicked())
		{
			select(name, obj);
		}
	};

	const auto lambdaGroup =
		[&](const char* title, const std::vector<std::pair<std::string, Simulable::Ptr>>& objs)
	{
		if (objs.empty())
		{
			return;
		}
		const std::string header = mrpt::format("%s (%zu)###%s", title, objs.size(), title);
		if (!ImGui::TreeNodeEx(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
		{
			return;
		}
		for (const auto& [name, obj] : objs)
		{
			if (contains_case_insensitive(name, worldFilter))
			{
				lambdaLeaf(name, name.empty() ? "(unnamed)" : name, obj);
			}
		}
		ImGui::TreePop();
	};

	// Vehicles, with their sensors as children:
	if (!vehicles.empty())
	{
		const std::string header = mrpt::format("Vehicles (%zu)###Vehicles", vehicles.size());
		if (ImGui::TreeNodeEx(header.c_str(), ImGuiTreeNodeFlags_DefaultOpen))
		{
			for (const auto& [name, obj] : vehicles)
			{
				auto* veh = dynamic_cast<VehicleBase*>(obj.get());
				const auto& sensors = veh->getSensors();

				// Show a vehicle if its name or any sensor matches the filter:
				bool anySensorMatches = false;
				for (const auto& s : sensors)
				{
					anySensorMatches |=
						s && contains_case_insensitive(name + "." + s->getName(), worldFilter);
				}
				if (!anySensorMatches && !contains_case_insensitive(name, worldFilter))
				{
					continue;
				}

				ImGuiTreeNodeFlags fl = ImGuiTreeNodeFlags_OpenOnArrow |
										ImGuiTreeNodeFlags_OpenOnDoubleClick |
										ImGuiTreeNodeFlags_SpanAvailWidth;
				if (sensors.empty())
				{
					fl |= ImGuiTreeNodeFlags_Leaf;
				}
				if (selected == obj)
				{
					fl |= ImGuiTreeNodeFlags_Selected;
				}
				if (!worldFilter.empty() && anySensorMatches)
				{
					ImGui::SetNextItemOpen(true);
				}
				const bool open = ImGui::TreeNodeEx(name.c_str(), fl, "%s", name.c_str());
				if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
				{
					select(name, obj);
				}
				if (!open)
				{
					continue;
				}
				for (const auto& s : sensors)
				{
					if (!s)
					{
						continue;
					}
					const auto fullName = name + "." + s->getName();
					if (contains_case_insensitive(fullName, worldFilter) ||
						contains_case_insensitive(name, worldFilter))
					{
						lambdaLeaf(fullName, ICON_MS_SENSORS " " + s->getName(), s);
					}
				}
				ImGui::TreePop();
			}
			ImGui::TreePop();
		}
	}

	lambdaGroup("Blocks", blocks);
	lambdaGroup("Actors", actors);
	lambdaGroup("World elements", elements);

	ImGui::End();
}

void World::GUI::draw_inspector_panel()
{
	if (!ImGui::Begin(WIN_INSPECTOR, &showInspector))
	{
		ImGui::End();
		return;
	}

	if (!selected)
	{
		ImGui::TextDisabled("Select an object in the World panel.");
		ImGui::End();
		return;
	}

	ImGui::SeparatorText(selectedName.c_str());

	// Pose (relative to its parent, for sensors):
	const auto pose = selected->getRelativePose();
	double xyz[3] = {pose.x, pose.y, pose.z};
	double ypr[3] = {mrpt::RAD2DEG(pose.yaw), mrpt::RAD2DEG(pose.pitch), mrpt::RAD2DEG(pose.roll)};

	// Labels above the fields, so they fit in narrow panels:
	bool changed = false;
	ImGui::TextUnformatted("x, y, z [m]");
	ImGui::SetNextItemWidth(-FLT_MIN);
	changed |=
		ImGui::DragScalarN("##xyz", ImGuiDataType_Double, xyz, 3, 0.01f, nullptr, nullptr, "%.3f");
	ImGui::TextUnformatted("yaw, pitch, roll [deg]");
	ImGui::SetNextItemWidth(-FLT_MIN);
	changed |=
		ImGui::DragScalarN("##ypr", ImGuiDataType_Double, ypr, 3, 0.25f, nullptr, nullptr, "%.2f");
	if (changed)
	{
		selected->setRelativePose(
			{xyz[0], xyz[1], xyz[2], mrpt::DEG2RAD(ypr[0]), mrpt::DEG2RAD(ypr[1]),
			 mrpt::DEG2RAD(ypr[2])});
	}

	const bool isSensor = dynamic_cast<SensorBase*>(selected.get()) != nullptr;
	const bool isElement = dynamic_cast<WorldElementBase*>(selected.get()) != nullptr;

	if (!isSensor)
	{
		if (placingWithMouse)
		{
			ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
		}
		if (ImGui::Button(ICON_MS_ADS_CLICK " Place with mouse"))
		{
			placingWithMouse = !placingWithMouse;
		}
		if (placingWithMouse)
		{
			ImGui::PopStyleColor();
		}
		ImGui::SetItemTooltip("The object follows the mouse until clicking on the 3D view.");
	}

	if (!isSensor && !isElement)
	{
		const auto vel = selected->getRefVelocityLocal();
		ImGui::SeparatorText("Velocity (local)");
		ImGui::Text("vx=%.03f vy=%.03f m/s", vel.vx, vel.vy);
		ImGui::Text("w=%.02f deg/s", mrpt::RAD2DEG(vel.omega));
	}

	// Light groups of this object:
	if (selectedVisual)
	{
		const auto groups = selectedVisual->lightGroupNames();
		if (!groups.empty())
		{
			ImGui::SeparatorText("Light groups");
		}
		for (const auto& g : groups)
		{
			bool on = selectedVisual->lightGroupState(g).value_or(false);
			if (ImGui::Checkbox(g.c_str(), &on))
			{
				selectedVisual->setLightGroupState(g, on);
			}
		}
	}

	ImGui::End();
}

void World::GUI::draw_lighting_panel()
{
	if (!ImGui::Begin(WIN_LIGHTING, &showLighting))
	{
		ImGui::End();
		return;
	}

	auto& lo = parent_.lightOptions_;

	if (ImGui::Checkbox("Shadows", &lo.enable_shadows))
	{
		parent_.worldVisual_->getViewport()->enableShadowCasting(lo.enable_shadows);
		parent_.worldPhysical_.getViewport()->enableShadowCasting(lo.enable_shadows);
	}

	bool pointAndSpot = parent_.pointAndSpotLightsEnabled_;
	if (ImGui::Checkbox("Point and spot lights", &pointAndSpot))
	{
		parent_.setPointAndSpotLightsEnabled(pointAndSpot);
	}

	ImGui::SeparatorText("Sun");
	float azimuth = static_cast<float>(lo.light_azimuth);
	float elevation = static_cast<float>(lo.light_elevation);
	bool dirChanged = ImGui::SliderAngle("Azimuth", &azimuth, -180.0f, 180.0f);
	dirChanged |= ImGui::SliderAngle("Elevation", &elevation, 0.0f, 90.0f);
	if (dirChanged)
	{
		lo.light_azimuth = azimuth;
		lo.light_elevation = elevation;
		parent_.setLightDirectionFromAzimuthElevation(azimuth, elevation);
	}
	if (ImGui::SliderFloat("Intensity", &sunIntensity, 0.0f, 2.0f))
	{
		parent_.setLightIntensityFactor(sunIntensity);
	}
	float ambient = lo.light_ambient;
	if (ImGui::SliderFloat("Ambient", &ambient, 0.0f, 1.0f))
	{
		parent_.setLightAmbient(ambient);
	}

	// All object light groups:
	ImGui::SeparatorText("Object light groups");
	for (const auto& lg : objects.lightGroups)
	{
		bool on = lg.visual->lightGroupState(lg.group).value_or(false);
		if (ImGui::Checkbox(lg.label.c_str(), &on))
		{
			lg.visual->setLightGroupState(lg.group, on);
		}
	}
	if (objects.lightGroups.empty())
	{
		ImGui::TextDisabled("(No object has <light_group> tags)");
	}

	ImGui::End();
}

void World::GUI::draw_messages_panel()
{
	if (!ImGui::Begin(WIN_MESSAGES, &showMessages))
	{
		ImGui::End();
		return;
	}

	std::string msgLines;
	{
		std::lock_guard<std::mutex> lck(parent_.guiMsgLinesMtx_);
		msgLines = parent_.guiMsgLines_;
	}
	if (msgLines.empty())
	{
		ImGui::TextDisabled("(No messages)");
	}
	else
	{
		ImGui::TextUnformatted(msgLines.c_str());
	}

	ImGui::End();
}

void World::GUI::draw_sensor_previews()
{
	for (auto& [name, p] : sensorPreviews)
	{
		p.visible = false;
		if (!p.open)
		{
			continue;
		}

		const std::string title = preview_window_title(name);

		dock_new_window_right(title);
		ImGui::SetNextWindowSize(ImVec2(400, 300), ImGuiCond_FirstUseEver);

		p.visible = ImGui::Begin(title.c_str(), &p.open);
		if (p.visible)
		{
			// Fit all images side by side, keeping their aspect ratios:
			const ImVec2 avail = ImGui::GetContentRegionAvail();
			float sumAspect = 0;
			for (int i = 0; i < 2; i++)
			{
				if (p.tex[i] != 0 && p.height[i] > 0)
				{
					sumAspect += static_cast<float>(p.width[i]) / static_cast<float>(p.height[i]);
				}
			}
			if (sumAspect > 0)
			{
				const float spacing = ImGui::GetStyle().ItemSpacing.x;
				const float h = std::max(1.0f, std::min(avail.y, (avail.x - spacing) / sumAspect));
				bool first = true;
				for (int i = 0; i < 2; i++)
				{
					if (p.tex[i] == 0 || p.height[i] <= 0)
					{
						continue;
					}
					if (!first)
					{
						ImGui::SameLine();
					}
					first = false;
					const float w =
						h * static_cast<float>(p.width[i]) / static_cast<float>(p.height[i]);
					ImGui::Image(as_imgui_texture(p.tex[i]), ImVec2(w, h));
					ImGui::SetItemTooltip(
						"%s: %ix%i", i == 0 ? "Image" : "Depth", p.width[i], p.height[i]);
				}
			}
			else
			{
				ImGui::TextDisabled("(Waiting for images)");
			}
		}
		ImGui::End();
	}
}

bool World::GUI::preview_needs_update(const std::string& previewName, int slot) const
{
	const auto it = sensorPreviews.find(previewName);
	// Always upload the first image, so the window knows its contents:
	return it == sensorPreviews.end() || it->second.visible || it->second.tex[slot] == 0;
}

void World::GUI::update_preview_texture(
	const std::string& previewName, int slot, const mrpt::img::CImage& im, bool startVisible)
{
	ASSERT_(slot == 0 || slot == 1);
	if (im.isEmpty())
	{
		return;
	}

	auto [it, isNew] = sensorPreviews.try_emplace(previewName);
	auto& p = it->second;
	if (isNew)
	{
		p.title = previewName;
		p.open = startVisible;
	}

	const int w = static_cast<int>(im.getWidth());
	const int h = static_cast<int>(im.getHeight());
	const int nCh = static_cast<int>(im.channels());

	GLenum format = GL_RGB;
	if (nCh == 1)
	{
		format = GL_RED;
	}
	else if (nCh == 4)
	{
		format = GL_RGBA;
	}

	auto& tex = p.tex[slot];
	const bool sizeChanged = tex == 0 || p.width[slot] != w || p.height[slot] != h;
	if (tex == 0)
	{
		glGenTextures(1, &tex);
	}
	glBindTexture(GL_TEXTURE_2D, tex);

	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, static_cast<GLint>(im.getRowStride() / nCh));

	if (sizeChanged)
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		// Show grayscale images as gray, not red:
		const GLint swizzle[4] = {
			GL_RED, nCh == 1 ? GL_RED : GL_GREEN, nCh == 1 ? GL_RED : GL_BLUE,
			nCh == 4 ? GL_ALPHA : GL_ONE};
		glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzle);

		const GLint internalFormat = nCh == 1 ? GL_R8 : (nCh == 4 ? GL_RGBA8 : GL_RGB8);
		glTexImage2D(
			GL_TEXTURE_2D, 0, internalFormat, w, h, 0, format, GL_UNSIGNED_BYTE,
			im.ptrLine<uint8_t>(0));
		p.width[slot] = w;
		p.height[slot] = h;
	}
	else
	{
		glTexSubImage2D(
			GL_TEXTURE_2D, 0, 0, 0, w, h, format, GL_UNSIGNED_BYTE, im.ptrLine<uint8_t>(0));
	}

	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glBindTexture(GL_TEXTURE_2D, 0);
}

void World::GUI::free_preview_textures()
{
	for (auto& [name, p] : sensorPreviews)
	{
		for (auto& tex : p.tex)
		{
			if (tex != 0)
			{
				glDeleteTextures(1, &tex);
				tex = 0;
			}
		}
	}
}
