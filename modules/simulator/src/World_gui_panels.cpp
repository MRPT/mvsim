/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

/**
 * @file World_gui_dockpanels.cpp
 * @brief Implementation of the new dockable panel-based GUI for MVSIM.
 *
 * This file implements fixed side panels using nanogui::DockablePanel,
 * replacing the previous floating subwindows with a cleaner, IDE-like interface.
 *
 * Panel Layout:
 * - Top Bar: Mode selector, quick stats (time/FPS/CPU)
 * - Left Panel: Tabs for View, Navigate, Stats
 * - Right Panel: Tabs for Add, Edit (visible only in WorldEdit mode)
 * - Bottom Bar: Mouse coords, selected object, status messages
 */

#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TPoint3D.h>
#include <mvsim/World.h>

#include <cmath>

// Include nanogui with DockablePanel support
#include <nanogui/nanogui.h>

using namespace mvsim;
using namespace std;

// ============================================================================
// Constants
// ============================================================================
namespace
{
constexpr int TOP_BAR_HEIGHT = 35;
constexpr int BOTTOM_BAR_HEIGHT = 25;
constexpr int LEFT_PANEL_WIDTH = 280;
constexpr int RIGHT_PANEL_WIDTH = 290;
constexpr int PANEL_OFFSET = 40;  // Leave room for top bar
}  // namespace

// ============================================================================
// Top Bar - Mode selector and quick stats
// ============================================================================
void World::GUI::prepare_top_bar()
{
	using namespace nanogui;

	topBar.panel = new DockablePanel(gui_win.get(), "", DockPosition::Top);
	topBar.panel->setFixedHeight(TOP_BAR_HEIGHT);
	topBar.panel->setDockMargin(0);
	topBar.panel->setCollapsible(false);
	topBar.panel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 8, 15));

	// Mode selector
	topBar.panel->add<Label>("Mode:", "sans-bold");
	topBar.cbMode = topBar.panel->add<ComboBox>(vector<string>{"Simulation", "World Edit"});
	topBar.cbMode->setFixedWidth(120);
	topBar.cbMode->setSelectedIndex(static_cast<int>(currentMode));
	topBar.cbMode->setCallback([this](int index) { setMode(static_cast<GUIMode>(index)); });

	// Separator
	topBar.panel->add<Label>("  |  ");

	// Quick stats
	topBar.lbTime = topBar.panel->add<Label>("Time: 00:00:00.000");
	topBar.panel->add<Label>("  ");
	topBar.lbFps = topBar.panel->add<Label>("FPS: --");
	topBar.panel->add<Label>("  ");
	topBar.lbCpu = topBar.panel->add<Label>("CPU: --%");

	// Spacer to push remaining items to the right
	auto* spacer = topBar.panel->add<Widget>();
	spacer->setFixedWidth(300);

	// Panel control buttons
	auto* btnMinAll = topBar.panel->add<Button>("", ENTYPO_ICON_ALIGN_BOTTOM);
	btnMinAll->setTooltip("Minimize all panels");
	btnMinAll->setCallback(
		[this]()
		{
			if (leftPanel.panel)
			{
				leftPanel.panel->setCollapsed(true);
			}
			if (rightPanel.panel && rightPanel.panel->visible())
			{
				rightPanel.panel->setCollapsed(true);
			}
		});

	auto* btnMaxAll = topBar.panel->add<Button>("", ENTYPO_ICON_BROWSER);
	btnMaxAll->setTooltip("Restore all panels");
	btnMaxAll->setCallback(
		[this]()
		{
			if (leftPanel.panel)
			{
				leftPanel.panel->setCollapsed(false);
			}
			if (rightPanel.panel && rightPanel.panel->visible())
			{
				rightPanel.panel->setCollapsed(false);
			}
		});

	// Quit button
	auto* btnQuit = topBar.panel->add<Button>("Quit", ENTYPO_ICON_ARROW_BOLD_LEFT);
	btnQuit->setCallback(
		[this]()
		{
			parent_.simulator_must_close(true);
			gui_win->setVisible(false);
			nanogui::leave();
		});
}

void World::GUI::update_top_bar()
{
	if (!topBar.panel)
	{
		return;
	}

	// Update time
	const double simTime = parent_.get_simul_time();
	const int hours = static_cast<int>(simTime / 3600.0) % 24;
	const int mins = static_cast<int>(std::fmod(simTime, 3600.0) / 60.0);
	const int secs = static_cast<int>(std::fmod(simTime, 60.0));
	const int msecs = static_cast<int>(std::fmod(simTime, 1.0) * 1000.0);

	topBar.lbTime->setCaption(mrpt::format("Time: %02d:%02d:%02d.%03d", hours, mins, secs, msecs));

	// Update FPS (from render timing)
	if (lastRenderFps_ > 0)
	{
		topBar.lbFps->setCaption(mrpt::format("FPS: %.0f", lastRenderFps_));
	}

	// Update CPU usage
	topBar.lbCpu->setCaption(mrpt::format("CPU: %.1f%%", lastCpuUsage_ * 100.0));
}

// ============================================================================
// Left Panel - View, Navigate, Stats tabs
// ============================================================================
void World::GUI::prepare_left_panel()
{
	using namespace nanogui;

	leftPanel.panel = new DockablePanel(gui_win.get(), "Tools", DockPosition::Left);
	leftPanel.panel->setFixedWidth(LEFT_PANEL_WIDTH);
	leftPanel.panel->setDockOffset(PANEL_OFFSET);
	leftPanel.panel->setSemiTransparent(true);
	leftPanel.panel->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 5, 5));

	auto* tabs = leftPanel.panel->add<TabWidget>();

	// ========== View Tab ==========
	auto* viewTab = tabs->createTab("View");
	viewTab->setLayout(new GroupLayout(10, 4, 8, 10));
	prepare_view_tab(viewTab);

	// ========== Navigate Tab ==========
	auto* navTab = tabs->createTab("Navigate");
	navTab->setLayout(new GroupLayout(10, 4, 8, 10));
	prepare_navigate_tab(navTab);

	// ========== Stats Tab ==========
	auto* statsTab = tabs->createTab("Stats");
	statsTab->setLayout(new GroupLayout(10, 4, 8, 10));
	prepare_stats_tab(statsTab);

	tabs->setActiveTab(0);
}

void World::GUI::prepare_view_tab(nanogui::Widget* parent)
{
	using namespace nanogui;

	parent->add<Label>("Camera", "sans-bold");

	// Vehicle follow dropdown
	parent->add<Label>("Follow:");
	leftPanel.viewTab.cbFollowVeh = parent->add<ComboBox>();
	leftPanel.viewTab.cbFollowVeh->setFixedWidth(150);
	updateVehicleList();  // Populate combo

	leftPanel.viewTab.cbFollowVeh->setCallback(
		[this](int idx)
		{
			const auto& items = leftPanel.viewTab.cbFollowVeh->items();
			if (idx == 0 || idx >= static_cast<int>(items.size()))
			{
				parent_.guiOptions_.follow_vehicle.clear();
			}
			else
			{
				parent_.guiOptions_.follow_vehicle = items[idx];
			}
		});

	// Orthogonal view
	auto* orthoCheck = parent->add<CheckBox>(
		"Orthogonal view", [this](bool b) { gui_win->camera().setCameraProjective(!b); });
	orthoCheck->setChecked(parent_.guiOptions_.ortho);

	parent->add<Label>("Visualization", "sans-bold");

	// Enable shadows
	auto* shadowCheck = parent->add<CheckBox>(
		"Enable shadows",
		[this](bool b)
		{
			auto vv = parent_.worldVisual_->getViewport();
			auto vp = parent_.worldPhysical_.getViewport();
			vv->enableShadowCasting(b);
			vp->enableShadowCasting(b);
			parent_.lightOptions_.enable_shadows = b;
		});
	shadowCheck->setChecked(parent_.lightOptions_.enable_shadows);

	// Show forces
	auto* forcesCheck = parent->add<CheckBox>(
		"Show forces", [this](bool b) { parent_.guiOptions_.show_forces = b; });
	forcesCheck->setChecked(parent_.guiOptions_.show_forces);

	// Show sensor points
	auto* sensorPtsCheck = parent->add<CheckBox>(
		"Show sensor points",
		[this](bool b)
		{
			std::lock_guard<std::mutex> lck(gui_win->background_scene_mtx);
			auto glVizSensors = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(
				gui_win->background_scene->getByName("group_sensors_viz"));
			if (glVizSensors)
			{
				glVizSensors->setVisibility(b);
			}
		});
	sensorPtsCheck->setChecked(parent_.guiOptions_.show_sensor_points);

	// Show sensor poses
	parent->add<CheckBox>(
		"Show sensor poses",
		[](bool b)
		{
			const auto& objs = SensorBase::GetAllSensorsOriginViz();
			for (const auto& o : *objs)
			{
				o->setVisibility(b);
			}
		});

	// Show collision shapes
	parent->add<CheckBox>(
		"Show collision shapes",
		[this](bool b)
		{
			auto lck = mrpt::lockHelper(parent_.simulableObjectsMtx_);
			for (auto& s : parent_.simulableObjects_)
			{
				auto* vis = dynamic_cast<VisualObject*>(s.second.get());
				if (vis)
				{
					vis->showCollisionShape(b);
				}
			}
		});

	parent->add<Label>("Lighting", "sans-bold");

	parent->add<Label>("Azimuth:");
	auto* azimuthSlider = parent->add<Slider>();
	azimuthSlider->setRange({static_cast<float>(-M_PI), static_cast<float>(M_PI)});
	azimuthSlider->setValue(parent_.lightOptions_.light_azimuth);
	azimuthSlider->setCallback(
		[this](float v)
		{
			parent_.lightOptions_.light_azimuth = v;
			parent_.setLightDirectionFromAzimuthElevation(
				parent_.lightOptions_.light_azimuth, parent_.lightOptions_.light_elevation);
		});

	parent->add<Label>("Elevation:");
	auto* elevSlider = parent->add<Slider>();
	elevSlider->setRange({0.0f, static_cast<float>(M_PI * 0.5)});
	elevSlider->setValue(parent_.lightOptions_.light_elevation);
	elevSlider->setCallback(
		[this](float v)
		{
			parent_.lightOptions_.light_elevation = v;
			parent_.setLightDirectionFromAzimuthElevation(
				parent_.lightOptions_.light_azimuth, parent_.lightOptions_.light_elevation);
		});
}

void World::GUI::prepare_navigate_tab(nanogui::Widget* parent)
{
	using namespace nanogui;

	parent->add<Label>("Click-to-Navigate", "sans-bold");

	parent->add<Label>("Target Vehicle:");
	leftPanel.navTab.cbVehicle = parent->add<ComboBox>();
	leftPanel.navTab.cbVehicle->setFixedWidth(150);
	// Will be populated by updateVehicleList()

	parent->add<Label>("Click on ground to set navigation target", "sans");

	parent->add<Label>("Current Target", "sans-bold");

	leftPanel.navTab.lbTargetPos = parent->add<Label>("Position: (none)");
	leftPanel.navTab.lbDistance = parent->add<Label>("Distance: --");
	leftPanel.navTab.lbETA = parent->add<Label>("ETA: --");
	leftPanel.navTab.lbStatus = parent->add<Label>("Status: Inactive");

	parent->add<Label>("Speed Control", "sans-bold");

	parent->add<Label>("Max Speed:");
	leftPanel.navTab.slMaxSpeed = parent->add<Slider>();
	leftPanel.navTab.slMaxSpeed->setRange({0.1f, 3.0f});
	leftPanel.navTab.slMaxSpeed->setValue(1.0f);

	leftPanel.navTab.lbMaxSpeedValue = parent->add<Label>("1.0 m/s");

	leftPanel.navTab.slMaxSpeed->setCallback(
		[this](float v)
		{ leftPanel.navTab.lbMaxSpeedValue->setCaption(mrpt::format("%.1f m/s", v)); });

	auto* cbShowPath = parent->add<CheckBox>("Show path");
	cbShowPath->setChecked(true);

	auto* cbShowMarker = parent->add<CheckBox>("Show target marker");
	cbShowMarker->setChecked(true);
	cbShowMarker->setCallback(
		[this](bool b)
		{
			showNavigationMarker_ = b;
			updateNavigationTargetMarker();
		});

	leftPanel.navTab.btnClear = parent->add<Button>("Clear Target");
	leftPanel.navTab.btnClear->setCallback(
		[this]()
		{
			const int idx = leftPanel.navTab.cbVehicle->selectedIndex();
			if (idx > 0)
			{
				const auto& items = leftPanel.navTab.cbVehicle->items();
				if (idx < static_cast<int>(items.size()))
				{
					parent_.clearNavigationTarget(items[idx]);
					updateNavigationTargetMarker();
				}
			}
		});
}

void World::GUI::prepare_stats_tab(nanogui::Widget* parent)
{
	using namespace nanogui;

	parent->add<Label>("Simulation", "sans-bold");

	leftPanel.statsTab.lbSimTime = parent->add<Label>("Time: 00:00:00.000");
	leftPanel.statsTab.lbCpuUsage = parent->add<Label>("CPU Usage: --%");
	leftPanel.statsTab.lbSimRate = parent->add<Label>("Sim Rate: -- realtime");
	leftPanel.statsTab.lbPhysicsHz = parent->add<Label>("Physics: -- Hz");
	leftPanel.statsTab.lbRenderFps = parent->add<Label>("Render: -- FPS");

	parent->add<Label>("Selected Object", "sans-bold");

	leftPanel.statsTab.lbSelectedName = parent->add<Label>("(none)");
	leftPanel.statsTab.lbSelectedPos = parent->add<Label>("Position: --");
	leftPanel.statsTab.lbSelectedVel = parent->add<Label>("Velocity: --");

	parent->add<Label>("Sensor Rates", "sans-bold");

	// Scrollable sensor list
	leftPanel.statsTab.sensorScroll = parent->add<VScrollPanel>();
	leftPanel.statsTab.sensorScroll->setFixedHeight(120);

	leftPanel.statsTab.sensorListWidget = leftPanel.statsTab.sensorScroll->add<Widget>();
	leftPanel.statsTab.sensorListWidget->setLayout(
		new BoxLayout(Orientation::Vertical, Alignment::Fill, 2, 2));
}

void World::GUI::update_left_panel()
{
	if (!leftPanel.panel)
	{
		return;
	}

	// Update Stats tab
	const double simTime = parent_.get_simul_time();
	const int hours = static_cast<int>(simTime / 3600.0) % 24;
	const int mins = static_cast<int>(std::fmod(simTime, 3600.0) / 60.0);
	const int secs = static_cast<int>(std::fmod(simTime, 60.0));
	const int msecs = static_cast<int>(std::fmod(simTime, 1.0) * 1000.0);

	leftPanel.statsTab.lbSimTime->setCaption(
		mrpt::format("Time: %02d:%02d:%02d.%03d", hours, mins, secs, msecs));

	leftPanel.statsTab.lbCpuUsage->setCaption(
		mrpt::format("CPU Usage: %.1f%%", lastCpuUsage_ * 100.0));

	leftPanel.statsTab.lbSimRate->setCaption(
		mrpt::format("Sim Rate: %.2fx realtime", lastSimRate_));

	leftPanel.statsTab.lbPhysicsHz->setCaption(mrpt::format("Physics: %.0f Hz", lastPhysicsHz_));

	leftPanel.statsTab.lbRenderFps->setCaption(mrpt::format("Render: %.0f FPS", lastRenderFps_));

	// Update selected object info
	if (gui_selectedObject.simulable)
	{
		const auto& name = gui_selectedObject.simulable->getName();
		leftPanel.statsTab.lbSelectedName->setCaption(name.empty() ? "(unnamed)" : name);

		const auto pose = gui_selectedObject.simulable->getPose();
		leftPanel.statsTab.lbSelectedPos->setCaption(
			mrpt::format("Pos: (%.2f, %.2f, %.2f)", pose.x, pose.y, pose.z));

		const auto twist = gui_selectedObject.simulable->getTwist();
		leftPanel.statsTab.lbSelectedVel->setCaption(
			mrpt::format("Vel: (%.2f, %.2f) m/s", twist.vx, twist.vy));
	}
	else
	{
		leftPanel.statsTab.lbSelectedName->setCaption("(none)");
		leftPanel.statsTab.lbSelectedPos->setCaption("Position: --");
		leftPanel.statsTab.lbSelectedVel->setCaption("Velocity: --");
	}

	// Update navigation tab
	update_navigate_tab();
}

void World::GUI::update_navigate_tab()
{
	if (!leftPanel.navTab.cbVehicle)
	{
		return;
	}

	const int idx = leftPanel.navTab.cbVehicle->selectedIndex();
	if (idx <= 0)
	{
		leftPanel.navTab.lbTargetPos->setCaption("Position: (none)");
		leftPanel.navTab.lbDistance->setCaption("Distance: --");
		leftPanel.navTab.lbETA->setCaption("ETA: --");
		leftPanel.navTab.lbStatus->setCaption("Status: Select vehicle");
		return;
	}

	const auto& items = leftPanel.navTab.cbVehicle->items();
	if (idx >= static_cast<int>(items.size()))
	{
		return;
	}

	const std::string vehName = items[idx];
	const auto target = parent_.getNavigationTarget(vehName);

	if (!target || !target->active)
	{
		leftPanel.navTab.lbTargetPos->setCaption("Position: (none)");
		leftPanel.navTab.lbDistance->setCaption("Distance: --");
		leftPanel.navTab.lbETA->setCaption("ETA: --");
		leftPanel.navTab.lbStatus->setCaption("Status: Inactive");
		return;
	}

	// Get vehicle current position
	auto vehIt = parent_.vehicles_.find(vehName);
	if (vehIt == parent_.vehicles_.end())
	{
		return;
	}

	const auto vehPose = vehIt->second->getPose();
	const double dx = target->position.x - vehPose.x;
	const double dy = target->position.y - vehPose.y;
	const double distance = std::sqrt(dx * dx + dy * dy);

	leftPanel.navTab.lbTargetPos->setCaption(
		mrpt::format("Position: (%.2f, %.2f)", target->position.x, target->position.y));

	leftPanel.navTab.lbDistance->setCaption(mrpt::format("Distance: %.2f m", distance));

	// Estimate ETA based on max speed
	const double maxSpeed = leftPanel.navTab.slMaxSpeed->value();
	if (maxSpeed > 0.01)
	{
		const double eta = distance / maxSpeed;
		if (eta < 60.0)
		{
			leftPanel.navTab.lbETA->setCaption(mrpt::format("ETA: %.0f sec", eta));
		}
		else
		{
			leftPanel.navTab.lbETA->setCaption(mrpt::format("ETA: %.1f min", eta / 60.0));
		}
	}

	// Status
	std::string statusStr;
	switch (target->status)
	{
		case NavigationTarget::Status::Inactive:
			statusStr = "Inactive";
			break;
		case NavigationTarget::Status::Navigating:
			statusStr = "Navigating";
			break;
		case NavigationTarget::Status::Reached:
			statusStr = "Reached!";
			break;
		case NavigationTarget::Status::Stuck:
			statusStr = "STUCK";
			break;
	}
	leftPanel.navTab.lbStatus->setCaption("Status: " + statusStr);
}

// ============================================================================
// Right Panel - Add, Edit tabs (World Edit mode)
// ============================================================================
void World::GUI::prepare_right_panel()
{
	using namespace nanogui;

	rightPanel.panel = new DockablePanel(gui_win.get(), "Editor", DockPosition::Right);
	rightPanel.panel->setFixedWidth(RIGHT_PANEL_WIDTH);
	rightPanel.panel->setDockOffset(PANEL_OFFSET);
	rightPanel.panel->setSemiTransparent(true);
	rightPanel.panel->setVisible(false);  // Hidden initially
	rightPanel.panel->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 5, 5));

	auto* tabs = rightPanel.panel->add<TabWidget>();

	// ========== Add Tab ==========
	auto* addTab = tabs->createTab("Add");
	addTab->setLayout(new GroupLayout(10, 4, 8, 10));
	prepare_add_tab(addTab);

	// ========== Edit Tab ==========
	auto* editTab = tabs->createTab("Edit");
	editTab->setLayout(new GroupLayout(10, 4, 8, 10));
	prepare_edit_tab(editTab);

	// ========== Scene Tab ==========
	auto* sceneTab = tabs->createTab("Scene");
	sceneTab->setLayout(new GroupLayout(10, 4, 8, 10));
	prepare_scene_tab(sceneTab);

	tabs->setActiveTab(0);
}

void World::GUI::prepare_add_tab(nanogui::Widget* parent)
{
	using namespace nanogui;

	parent->add<Label>("Add Elements", "sans-bold");

	// Tool buttons
	auto* toolGrid = parent->add<Widget>();
	toolGrid->setLayout(new GridLayout(Orientation::Horizontal, 3, Alignment::Fill, 2, 2));

	auto createToolBtn = [this, toolGrid](
							 const std::string& label, int icon, WorldEditTool tool) -> Button*
	{
		auto* btn = toolGrid->add<Button>(label, icon);
		btn->setFlags(Button::RadioButton);
		btn->setFixedSize(nanogui::Vector2i(80, 30));
		btn->setChangeCallback(
			[this, tool](bool pushed)
			{
				if (pushed)
				{
					worldEditState.currentTool = tool;
				}
			});
		return btn;
	};

	auto* btnSelect = createToolBtn("Select", ENTYPO_ICON_MOUSE_POINTER, WorldEditTool::Select);
	btnSelect->setPushed(true);

	createToolBtn("Wall", ENTYPO_ICON_MINUS, WorldEditTool::AddWall);
	createToolBtn("Box", ENTYPO_ICON_BOX, WorldEditTool::AddBox);
	createToolBtn("Cylinder", ENTYPO_ICON_CIRCLE, WorldEditTool::AddCylinder);
	createToolBtn("Sphere", ENTYPO_ICON_RECORD, WorldEditTool::AddSphere);
	createToolBtn("Ramp", ENTYPO_ICON_TRIANGLE_UP, WorldEditTool::AddRamp);

	parent->add<Label>("Wall Properties", "sans-bold");

	auto* wallProps = parent->add<Widget>();
	wallProps->setLayout(new GridLayout(Orientation::Horizontal, 2, Alignment::Fill, 2, 2));

	wallProps->add<Label>("Height:");
	rightPanel.addTab.tbWallHeight = wallProps->add<TextBox>("2.5");
	rightPanel.addTab.tbWallHeight->setEditable(true);
	rightPanel.addTab.tbWallHeight->setUnits("m");
	rightPanel.addTab.tbWallHeight->setFixedWidth(80);
	rightPanel.addTab.tbWallHeight->setFormat("[0-9]*\\.?[0-9]+");

	wallProps->add<Label>("Thickness:");
	rightPanel.addTab.tbWallThickness = wallProps->add<TextBox>("0.15");
	rightPanel.addTab.tbWallThickness->setEditable(true);
	rightPanel.addTab.tbWallThickness->setUnits("m");
	rightPanel.addTab.tbWallThickness->setFixedWidth(80);
	rightPanel.addTab.tbWallThickness->setFormat("[0-9]*\\.?[0-9]+");

	parent->add<Label>("Block Properties", "sans-bold");

	auto* blockProps = parent->add<Widget>();
	blockProps->setLayout(new GridLayout(Orientation::Horizontal, 2, Alignment::Fill, 2, 2));

	blockProps->add<Label>("Size:");
	rightPanel.addTab.tbBlockSize = blockProps->add<TextBox>("1.0");
	rightPanel.addTab.tbBlockSize->setEditable(true);
	rightPanel.addTab.tbBlockSize->setUnits("m");
	rightPanel.addTab.tbBlockSize->setFixedWidth(80);

	blockProps->add<Label>("Mass:");
	rightPanel.addTab.tbBlockMass = blockProps->add<TextBox>("10.0");
	rightPanel.addTab.tbBlockMass->setEditable(true);
	rightPanel.addTab.tbBlockMass->setUnits("kg");
	rightPanel.addTab.tbBlockMass->setFixedWidth(80);

	parent->add<Label>("Options", "sans-bold");

	rightPanel.addTab.cbSnapToGrid = parent->add<CheckBox>("Snap to grid");
	rightPanel.addTab.cbSnapToGrid->setChecked(true);

	auto* snapGrid = parent->add<Widget>();
	snapGrid->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
	snapGrid->add<Label>("Grid size:");
	rightPanel.addTab.tbGridSize = snapGrid->add<TextBox>("0.5");
	rightPanel.addTab.tbGridSize->setEditable(true);
	rightPanel.addTab.tbGridSize->setUnits("m");
	rightPanel.addTab.tbGridSize->setFixedWidth(60);
}

void World::GUI::prepare_edit_tab(nanogui::Widget* parent)
{
	using namespace nanogui;

	parent->add<Label>("Selected Object", "sans-bold");

	rightPanel.editTab.lbSelectedName = parent->add<Label>("(none selected)");

	parent->add<Label>("Position", "sans-bold");

	auto* posGrid = parent->add<Widget>();
	posGrid->setLayout(new GridLayout(Orientation::Horizontal, 2, Alignment::Fill, 2, 2));

	posGrid->add<Label>("X:");
	rightPanel.editTab.tbPosX = posGrid->add<TextBox>("0.00");
	rightPanel.editTab.tbPosX->setEditable(true);
	rightPanel.editTab.tbPosX->setFixedWidth(80);

	posGrid->add<Label>("Y:");
	rightPanel.editTab.tbPosY = posGrid->add<TextBox>("0.00");
	rightPanel.editTab.tbPosY->setEditable(true);
	rightPanel.editTab.tbPosY->setFixedWidth(80);

	posGrid->add<Label>("Z:");
	rightPanel.editTab.tbPosZ = posGrid->add<TextBox>("0.00");
	rightPanel.editTab.tbPosZ->setEditable(true);
	rightPanel.editTab.tbPosZ->setFixedWidth(80);

	parent->add<Label>("Rotation", "sans-bold");

	parent->add<Label>("Yaw:");
	rightPanel.editTab.slYaw = parent->add<Slider>();
	rightPanel.editTab.slYaw->setRange({static_cast<float>(-M_PI), static_cast<float>(M_PI)});

	rightPanel.editTab.lbYawValue = parent->add<Label>("0.0 deg");

	rightPanel.editTab.slYaw->setCallback(
		[this](float v)
		{ rightPanel.editTab.lbYawValue->setCaption(mrpt::format("%.1f deg", v * 180.0 / M_PI)); });

	parent->add<Label>("Actions", "sans-bold");

	auto* actionBtns = parent->add<Widget>();
	actionBtns->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Fill, 2, 4));

	rightPanel.editTab.btnDelete = actionBtns->add<Button>("Delete", ENTYPO_ICON_TRASH);
	rightPanel.editTab.btnDelete->setCallback(
		[this]()
		{
			if (gui_selectedObject.simulable)
			{
				const std::string name = gui_selectedObject.simulable->getName();
				parent_.removeObject(name);
				gui_selectedObject = InfoPerObject();
				update_right_panel();
			}
		});

	rightPanel.editTab.btnDuplicate = actionBtns->add<Button>("Duplicate", ENTYPO_ICON_DOCUMENTS);
	rightPanel.editTab.btnDuplicate->setCallback(
		[this]()
		{
			// TODO: Implement duplication
		});

	// Apply changes button
	rightPanel.editTab.btnApply = parent->add<Button>("Apply Changes", ENTYPO_ICON_CHECK);
	rightPanel.editTab.btnApply->setCallback([this]() { applyEditTabChanges(); });
}

void World::GUI::prepare_scene_tab(nanogui::Widget* parent)
{
	using namespace nanogui;

	parent->add<Label>("History", "sans-bold");

	auto* histBtns = parent->add<Widget>();
	histBtns->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Fill, 2, 4));

	rightPanel.sceneTab.btnUndo = histBtns->add<Button>("Undo", ENTYPO_ICON_CCW);
	rightPanel.sceneTab.btnUndo->setCallback([this]() { undo(); });

	rightPanel.sceneTab.btnRedo = histBtns->add<Button>("Redo", ENTYPO_ICON_CW);
	rightPanel.sceneTab.btnRedo->setCallback([this]() { redo(); });

	rightPanel.sceneTab.lbHistoryCount = parent->add<Label>("History: 0 / 0");

	parent->add<Label>("Export", "sans-bold");

	auto* btnExport = parent->add<Button>("Export World XML...", ENTYPO_ICON_EXPORT);
	btnExport->setCallback(
		[this]()
		{
			const std::string xml = parent_.exportWorldToXML();
			// TODO: Show save dialog or copy to clipboard
			MRPT_LOG_INFO_STREAM("Exported XML:\n" << xml);
		});

	parent->add<Label>("Object List", "sans-bold");

	// Scrollable object list
	rightPanel.sceneTab.objectScroll = parent->add<VScrollPanel>();
	rightPanel.sceneTab.objectScroll->setFixedHeight(200);

	rightPanel.sceneTab.objectListWidget = rightPanel.sceneTab.objectScroll->add<Widget>();
	rightPanel.sceneTab.objectListWidget->setLayout(
		new BoxLayout(Orientation::Vertical, Alignment::Fill, 2, 2));
}

void World::GUI::update_right_panel()
{
	if (!rightPanel.panel || !rightPanel.panel->visible())
	{
		return;
	}

	// Update Edit tab with selected object info
	if (gui_selectedObject.simulable)
	{
		const auto& name = gui_selectedObject.simulable->getName();
		rightPanel.editTab.lbSelectedName->setCaption(name.empty() ? "(unnamed)" : name);

		const auto pose = gui_selectedObject.simulable->getPose();
		rightPanel.editTab.tbPosX->setValue(mrpt::format("%.2f", pose.x));
		rightPanel.editTab.tbPosY->setValue(mrpt::format("%.2f", pose.y));
		rightPanel.editTab.tbPosZ->setValue(mrpt::format("%.2f", pose.z));
		rightPanel.editTab.slYaw->setValue(static_cast<float>(pose.yaw));
		rightPanel.editTab.lbYawValue->setCaption(
			mrpt::format("%.1f deg", pose.yaw * 180.0 / M_PI));

		rightPanel.editTab.btnDelete->setEnabled(true);
		rightPanel.editTab.btnDuplicate->setEnabled(true);
		rightPanel.editTab.btnApply->setEnabled(true);
	}
	else
	{
		rightPanel.editTab.lbSelectedName->setCaption("(none selected)");
		rightPanel.editTab.btnDelete->setEnabled(false);
		rightPanel.editTab.btnDuplicate->setEnabled(false);
		rightPanel.editTab.btnApply->setEnabled(false);
	}

	// Update history count
	rightPanel.sceneTab.lbHistoryCount->setCaption(
		mrpt::format(
			"History: %zu / %zu", worldEditState.undoStack.size(),
			worldEditState.redoStack.size()));

	// Update undo/redo button states
	rightPanel.sceneTab.btnUndo->setEnabled(!worldEditState.undoStack.empty());
	rightPanel.sceneTab.btnRedo->setEnabled(!worldEditState.redoStack.empty());
}

void World::GUI::applyEditTabChanges()
{
	if (!gui_selectedObject.simulable)
	{
		return;
	}

	try
	{
		const double x = std::stod(rightPanel.editTab.tbPosX->value());
		const double y = std::stod(rightPanel.editTab.tbPosY->value());
		const double z = std::stod(rightPanel.editTab.tbPosZ->value());
		const double yaw = static_cast<double>(rightPanel.editTab.slYaw->value());

		const auto oldPose = gui_selectedObject.simulable->getPose();
		const mrpt::math::TPose3D newPose(x, y, z, yaw, oldPose.pitch, oldPose.roll);

		// Record for undo
		WorldEditAction action;
		action.type = WorldEditAction::Type::Move;
		action.objectName = gui_selectedObject.simulable->getName();
		action.poseBefore = oldPose;
		action.poseAfter = newPose;
		action.timestamp = parent_.get_simul_time();
		recordEditAction(action);

		// Apply the pose
		gui_selectedObject.simulable->setPose(newPose);
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM("Error applying changes: " << e.what());
	}
}

// ============================================================================
// Bottom Bar - Status display
// ============================================================================
void World::GUI::prepare_bottom_bar()
{
	using namespace nanogui;

	bottomBar.panel = new DockablePanel(gui_win.get(), "", DockPosition::Bottom);
	bottomBar.panel->setFixedHeight(BOTTOM_BAR_HEIGHT);
	bottomBar.panel->setDockMargin(0);
	bottomBar.panel->setCollapsible(false);
	bottomBar.panel->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 8, 20));

	bottomBar.lbMousePos = bottomBar.panel->add<Label>("Mouse: (0.00, 0.00)");
	bottomBar.panel->add<Label>("  |  ");
	bottomBar.lbSelected = bottomBar.panel->add<Label>("Selected: (none)");
	bottomBar.panel->add<Label>("  |  ");
	bottomBar.lbMode = bottomBar.panel->add<Label>("Mode: Simulation");
	bottomBar.panel->add<Label>("  |  ");
	bottomBar.lbStatus = bottomBar.panel->add<Label>("Ready");
}

void World::GUI::update_bottom_bar()
{
	if (!bottomBar.panel)
	{
		return;
	}

	// Mouse position is updated in mouse callback
	// bottomBar.lbMousePos updated elsewhere

	// Selected object
	if (gui_selectedObject.simulable)
	{
		const auto& name = gui_selectedObject.simulable->getName();
		bottomBar.lbSelected->setCaption("Selected: " + (name.empty() ? "(unnamed)" : name));
	}
	else
	{
		bottomBar.lbSelected->setCaption("Selected: (none)");
	}

	// Mode
	bottomBar.lbMode->setCaption(
		currentMode == GUIMode::Simulation ? "Mode: Simulation" : "Mode: World Edit");
}

// ============================================================================
// Mode switching
// ============================================================================
void World::GUI::setMode(GUIMode mode)
{
	currentMode = mode;

	// Update mode combo if it exists
	if (topBar.cbMode)
	{
		topBar.cbMode->setSelectedIndex(static_cast<int>(mode));
	}

	// Show/hide right panel based on mode
	if (rightPanel.panel)
	{
		rightPanel.panel->setVisible(mode == GUIMode::WorldEdit);
	}

	// Update bottom bar
	update_bottom_bar();

	// Re-layout
	if (gui_win)
	{
		gui_win->performLayout();
	}
}

// ============================================================================
// Helper methods
// ============================================================================
void World::GUI::updateVehicleList()
{
	std::vector<std::string> vehicleNames;
	vehicleNames.push_back("(none)");

	for (const auto& v : parent_.vehicles_)
	{
		vehicleNames.push_back(v.first);
	}

	if (leftPanel.viewTab.cbFollowVeh)
	{
		leftPanel.viewTab.cbFollowVeh->setItems(vehicleNames);
		leftPanel.viewTab.cbFollowVeh->setSelectedIndex(0);
	}

	if (leftPanel.navTab.cbVehicle)
	{
		leftPanel.navTab.cbVehicle->setItems(vehicleNames);
		leftPanel.navTab.cbVehicle->setSelectedIndex(0);
	}
}

void World::GUI::updateMousePosition(const mrpt::math::TPoint3D& worldPt)
{
	if (bottomBar.lbMousePos)
	{
		bottomBar.lbMousePos->setCaption(mrpt::format("Mouse: (%.2f, %.2f)", worldPt.x, worldPt.y));
	}
}

void World::GUI::setStatusMessage(const std::string& msg)
{
	if (bottomBar.lbStatus)
	{
		bottomBar.lbStatus->setCaption(msg);
	}
}