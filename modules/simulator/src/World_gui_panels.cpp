/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

/**
 * @file World_gui_panels.cpp
 * @brief Implementation of new GUI panels for statistics, navigation, and editing
 */

#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSphere.h>
#include <mvsim/World.h>

using namespace mvsim;

// ============================================================================
// Mode switching
// ============================================================================

void World::GUI::setMode(GUIMode mode)
{
	currentMode = mode;

	// Show/hide mode-specific panels
	if (editPanel.window)
	{
		editPanel.window->setVisible(mode == GUIMode::WorldEdit);
	}

	// Navigation panel visible in simulation mode
	if (navPanel.window)
	{
		navPanel.window->setVisible(mode == GUIMode::Simulation);
	}

	if (gui_win)
	{
		gui_win->performLayout();
	}
}

void World::setGUIMode(GUIMode mode) { gui_.setMode(mode); }

// ============================================================================
// Statistics Panel
// ============================================================================

void World::GUI::prepare_stats_panel()
{
	if (!parent_.guiOptions_.show_stats_panel)
	{
		return;
	}

	nanogui::Window* w = gui_win->createManagedSubWindow("Statistics");

	// Position based on config
	int xPos = 1;
	if (parent_.guiOptions_.stats_panel_position == "right")
	{
		xPos = static_cast<int>(parent_.guiOptions_.win_w) - 290;
	}
	w->setPosition({xPos, 150});
	w->setLayout(
		new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 2));
	w->setFixedWidth(280);

	statsPanel.window = w;

	// Simulation stats section
	w->add<nanogui::Label>("Simulation", "sans-bold");

	statsPanel.lbSimTime = w->add<nanogui::Label>("Time: 00:00:00.000");
	statsPanel.lbCpuUsage = w->add<nanogui::Label>("CPU: 0.0%");
	statsPanel.lbSimRate = w->add<nanogui::Label>("Rate: 1.0x realtime");
	statsPanel.lbPhysicsFps = w->add<nanogui::Label>("Physics: 0 Hz");
	statsPanel.lbRenderFps = w->add<nanogui::Label>("Render: 0 FPS");

	w->add<nanogui::Label>(" ");  // Separator

	// Selected vehicle section
	w->add<nanogui::Label>("Selected Object", "sans-bold");
	statsPanel.lbSelectedVehicle = w->add<nanogui::Label>("(none)");
	statsPanel.lbVehiclePosition = w->add<nanogui::Label>("Pos: -");
	statsPanel.lbVehicleVelocity = w->add<nanogui::Label>("Vel: -");

	w->add<nanogui::Label>(" ");  // Separator

	// Sensor stats section (dynamic content)
	w->add<nanogui::Label>("Sensor Rates", "sans-bold");

	// Create a scrollable container for sensor stats
	auto* scroll = w->add<nanogui::VScrollPanel>();
	scroll->setFixedHeight(200);

	statsPanel.sensorStatsContainer = scroll->add<nanogui::Widget>();
	statsPanel.sensorStatsContainer->setLayout(
		new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 2, 2));
}

void World::GUI::update_stats_panel()
{
	if (!statsPanel.window)
	{
		return;
	}

	// Update simulation time
	const double simTime = parent_.get_simul_time();
	const int hours = static_cast<int>(simTime / 3600.0);
	const int mins = static_cast<int>(std::fmod(simTime, 3600.0) / 60.0);
	const double secs = std::fmod(simTime, 60.0);

	if (statsPanel.lbSimTime)
	{
		statsPanel.lbSimTime->setCaption(mrpt::format("Time: %02d:%02d:%06.3f", hours, mins, secs));
	}

	// CPU usage
	const double cpuDt = std::max(1e-10, parent_.timlogger_.getMeanTime("run_simulation.cpu_dt"));
	const double simDt = std::max(1e-10, parent_.timlogger_.getMeanTime("run_simulation.dt"));
	const double cpuUsage = (cpuDt / simDt) * 100.0;

	if (statsPanel.lbCpuUsage)
	{
		statsPanel.lbCpuUsage->setCaption(mrpt::format("CPU: %.1f%%", cpuUsage));
	}

	// Simulation rate
	const double simRate = (cpuDt > 1e-10) ? (simDt / cpuDt) : 0.0;
	if (statsPanel.lbSimRate)
	{
		statsPanel.lbSimRate->setCaption(mrpt::format("Rate: %.2fx realtime", simRate));
	}

	// Physics FPS
	const double physDt = parent_.get_simul_timestep();
	if (statsPanel.lbPhysicsFps && physDt > 1e-10)
	{
		statsPanel.lbPhysicsFps->setCaption(mrpt::format("Physics: %.0f Hz", 1.0 / physDt));
	}

	// Render FPS
	if (statsPanel.lbRenderFps)
	{
		statsPanel.lbRenderFps->setCaption(
			mrpt::format("Render: %d FPS", parent_.guiOptions_.refresh_fps));
	}

	// Selected object info
	if (gui_selectedObject.simulable)
	{
		const auto& name = gui_selectedObject.simulable->getName();
		if (statsPanel.lbSelectedVehicle)
		{
			statsPanel.lbSelectedVehicle->setCaption(name);
		}

		const auto pose = gui_selectedObject.simulable->getPose();
		if (statsPanel.lbVehiclePosition)
		{
			statsPanel.lbVehiclePosition->setCaption(
				mrpt::format("Pos: (%.2f, %.2f, %.2f)", pose.x, pose.y, pose.z));
		}

		const auto twist = gui_selectedObject.simulable->getTwist();
		const double speed = std::sqrt(twist.vx * twist.vx + twist.vy * twist.vy);
		if (statsPanel.lbVehicleVelocity)
		{
			statsPanel.lbVehicleVelocity->setCaption(
				mrpt::format("Vel: %.2f m/s, %.1f deg/s", speed, mrpt::RAD2DEG(twist.omega)));
		}

		// Update sensor stats for selected vehicle
		if (auto* veh = dynamic_cast<VehicleBase*>(gui_selectedObject.simulable.get()); veh)
		{
			updateSensorStatsForVehicle(*veh);
		}
	}
	else
	{
		if (statsPanel.lbSelectedVehicle)
		{
			statsPanel.lbSelectedVehicle->setCaption("(none)");
		}
		if (statsPanel.lbVehiclePosition)
		{
			statsPanel.lbVehiclePosition->setCaption("Pos: -");
		}
		if (statsPanel.lbVehicleVelocity)
		{
			statsPanel.lbVehicleVelocity->setCaption("Vel: -");
		}
	}
}

void World::GUI::updateSensorStatsForVehicle(const VehicleBase& veh)
{
	if (!statsPanel.sensorStatsContainer)
	{
		return;
	}

	const auto& sensors = veh.getSensors();
	const double currentTime = parent_.get_simul_time();

	for (const auto& sensor : sensors)
	{
		if (!sensor)
		{
			continue;
		}

		const std::string key = veh.getName() + "." + sensor->getName();

		// Find or create widget for this sensor
		nanogui::Widget* sensorWidget = nullptr;
		auto it = statsPanel.sensorStatWidgets.find(key);

		if (it == statsPanel.sensorStatWidgets.end())
		{
			// Create new widget
			sensorWidget = statsPanel.sensorStatsContainer->add<nanogui::Widget>();
			sensorWidget->setLayout(new nanogui::BoxLayout(
				nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 1, 1));

			// Sensor name label
			auto* lblName = sensorWidget->add<nanogui::Label>(sensor->getName());
			lblName->setFontSize(14);

			// Rate label (will be updated)
			auto* lblRate = sensorWidget->add<nanogui::Label>("Rate: -");
			lblRate->setFontSize(12);

			statsPanel.sensorStatWidgets[key] = sensorWidget;

			if (gui_win)
			{
				gui_win->performLayout();
			}
		}
		else
		{
			sensorWidget = it->second;
		}

		// Update rate display
		if (sensorWidget && sensorWidget->childCount() >= 2)
		{
			auto* lblRate = dynamic_cast<nanogui::Label*>(sensorWidget->childAt(1));
			if (lblRate)
			{
				const auto& stats = sensor->getStats();
				const double actualRate = stats.getActualRateHz();
				const double targetRate = sensor->getTargetRateHz();
				const double timeSince = stats.getTimeSinceLastObservation(currentTime);

				std::string rateStr;
				if (timeSince < 0)
				{
					rateStr = "Rate: (no data)";
				}
				else
				{
					rateStr = mrpt::format(
						"%.1f/%.1f Hz (%.0fms ago)", actualRate, targetRate, timeSince * 1000.0);
				}

				lblRate->setCaption(rateStr);
			}
		}
	}
}

// ============================================================================
// Navigation Panel
// ============================================================================

void World::GUI::prepare_nav_panel()
{
	if (!parent_.guiOptions_.enable_click_navigation)
	{
		return;
	}

	nanogui::Window* w = gui_win->createManagedSubWindow("Navigation");
	w->setPosition({1, 500});
	w->setLayout(
		new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 3));
	w->setFixedWidth(280);

	navPanel.window = w;

	w->add<nanogui::Label>("Click-to-Navigate", "sans-bold");

	// Vehicle selector
	w->add<nanogui::Label>("Target Vehicle:");

	std::vector<std::string> vehicleNames;
	vehicleNames.push_back("(none)");
	for (const auto& v : parent_.vehicles_)
	{
		vehicleNames.push_back(v.first);
	}

	navPanel.cbVehicle = w->add<nanogui::ComboBox>(vehicleNames);
	navPanel.cbVehicle->setSelectedIndex(0);
	navPanel.cbVehicle->setCallback(
		[this, vehicleNames](int idx)
		{
			if (idx > 0 && idx < static_cast<int>(vehicleNames.size()))
			{
				navigationState.targetVehicleName = vehicleNames[idx];
			}
			else
			{
				if (!navigationState.targetVehicleName.empty())
				{
					parent_.clearNavigationTarget(navigationState.targetVehicleName);
				}
				navigationState.targetVehicleName.clear();
			}
		});

	w->add<nanogui::Label>(" ");

	// Instructions
	w->add<nanogui::Label>("Click on ground to set target");

	w->add<nanogui::Label>(" ");

	// Target info
	w->add<nanogui::Label>("Current Target:", "sans-bold");
	navPanel.lbTargetPos = w->add<nanogui::Label>("Position: (none)");
	navPanel.lbDistanceToTarget = w->add<nanogui::Label>("Distance: -");
	navPanel.lbETA = w->add<nanogui::Label>("ETA: -");
	navPanel.lbStatus = w->add<nanogui::Label>("Status: Inactive");

	w->add<nanogui::Label>(" ");

	// Max speed control
	auto* speedPanel = w->add<nanogui::Widget>();
	speedPanel->setLayout(
		new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Middle, 5, 2));

	speedPanel->add<nanogui::Label>("Max Speed:");
	navPanel.slMaxSpeed = speedPanel->add<nanogui::Slider>();
	navPanel.slMaxSpeed->setRange({0.1f, 3.0f});
	navPanel.slMaxSpeed->setValue(1.0f);
	navPanel.slMaxSpeed->setFixedWidth(100);

	navPanel.lbMaxSpeed = speedPanel->add<nanogui::Label>("1.0 m/s");
	navPanel.lbMaxSpeed->setFixedWidth(60);

	navPanel.slMaxSpeed->setCallback(
		[this](float value)
		{
			navPanel.lbMaxSpeed->setCaption(mrpt::format("%.1f m/s", value));
			navigationState.target.maxLinearSpeed = static_cast<double>(value);
		});

	w->add<nanogui::Label>(" ");

	// Visualization options
	auto* chkPath =
		w->add<nanogui::CheckBox>("Show path", [this](bool b) { navigationState.showPath = b; });
	chkPath->setChecked(true);

	auto* chkMarker = w->add<nanogui::CheckBox>(
		"Show target marker",
		[this](bool b)
		{
			navigationState.showTargetMarker = b;
			updateNavigationTargetMarker();
		});
	chkMarker->setChecked(true);

	w->add<nanogui::Label>(" ");

	// Clear button
	navPanel.btnClearTarget = w->add<nanogui::Button>("Clear Target");
	navPanel.btnClearTarget->setCallback(
		[this]()
		{
			if (!navigationState.targetVehicleName.empty())
			{
				parent_.clearNavigationTarget(navigationState.targetVehicleName);
			}
			navigationState.target.active = false;
			updateNavigationTargetMarker();
		});
}

void World::GUI::update_nav_panel()
{
	if (!navPanel.window)
	{
		return;
	}

	if (navigationState.targetVehicleName.empty() || !navigationState.target.active)
	{
		if (navPanel.lbTargetPos)
		{
			navPanel.lbTargetPos->setCaption("Position: (none)");
		}
		if (navPanel.lbDistanceToTarget)
		{
			navPanel.lbDistanceToTarget->setCaption("Distance: -");
		}
		if (navPanel.lbETA)
		{
			navPanel.lbETA->setCaption("ETA: -");
		}
		if (navPanel.lbStatus)
		{
			navPanel.lbStatus->setCaption("Status: Inactive");
		}
		return;
	}

	const auto& target = navigationState.target;

	// Target position
	if (navPanel.lbTargetPos)
	{
		navPanel.lbTargetPos->setCaption(
			mrpt::format("Position: (%.2f, %.2f)", target.position.x, target.position.y));
	}

	// Find vehicle and compute distance
	auto it = parent_.vehicles_.find(navigationState.targetVehicleName);
	if (it != parent_.vehicles_.end())
	{
		const auto pose = it->second->getPose();
		const double dx = target.position.x - pose.x;
		const double dy = target.position.y - pose.y;
		const double distance = std::sqrt(dx * dx + dy * dy);

		if (navPanel.lbDistanceToTarget)
		{
			navPanel.lbDistanceToTarget->setCaption(mrpt::format("Distance: %.2f m", distance));
		}

		// ETA based on max speed
		if (navPanel.lbETA)
		{
			if (target.maxLinearSpeed > 0.01)
			{
				const double eta = distance / target.maxLinearSpeed;
				navPanel.lbETA->setCaption(mrpt::format("ETA: %.1f s", eta));
			}
			else
			{
				navPanel.lbETA->setCaption("ETA: -");
			}
		}
	}

	// Status
	if (navPanel.lbStatus)
	{
		std::string statusStr;
		switch (target.status)
		{
			case NavigationTarget::Status::Inactive:
				statusStr = "Inactive";
				break;
			case NavigationTarget::Status::Navigating:
				statusStr = "Navigating";
				break;
			case NavigationTarget::Status::Reached:
				statusStr = "Reached";
				break;
			case NavigationTarget::Status::Stuck:
				statusStr = "Stuck";
				break;
		}
		navPanel.lbStatus->setCaption("Status: " + statusStr);
	}
}

void World::GUI::updateNavigationTargetMarker()
{
	if (!navigationState.target.active || !navigationState.showTargetMarker)
	{
		// Hide marker
		if (navigationState.glTargetMarker)
		{
			navigationState.glTargetMarker->setVisibility(false);
		}
		return;
	}

	// Create marker if needed
	if (!navigationState.glTargetMarker)
	{
		navigationState.glTargetMarker = mrpt::opengl::CSetOfObjects::Create();
		navigationState.glTargetMarker->setName("nav_target_marker");

		// Create a cylinder marker
		auto cylinder = mrpt::opengl::CCylinder::Create(
			0.2f,  // base radius
			0.05f,	// top radius
			2.0f  // height
		);
		cylinder->setColor_u8(0, 255, 0, 180);	// Green, semi-transparent
		navigationState.glTargetMarker->insert(cylinder);

		// Create a small sphere at top
		auto sphere = mrpt::opengl::CSphere::Create(0.15f);
		sphere->setColor_u8(0, 255, 0, 255);
		sphere->setLocation(0, 0, 2.0f);
		navigationState.glTargetMarker->insert(sphere);

		// Add to user objects
		auto lck = mrpt::lockHelper(parent_.guiUserObjectsMtx_);
		if (!parent_.guiUserObjectsViz_)
		{
			parent_.guiUserObjectsViz_ = mrpt::opengl::CSetOfObjects::Create();
		}
		parent_.guiUserObjectsViz_->insert(navigationState.glTargetMarker);
	}

	// Update position
	const auto& target = navigationState.target;
	const auto offset = parent_.worldRenderOffset();

	navigationState.glTargetMarker->setPose(
		mrpt::poses::CPose3D(
			target.position.x + offset.x, target.position.y + offset.y, 0.0 + offset.z, 0.0, 0.0,
			0.0));
	navigationState.glTargetMarker->setVisibility(true);
}

// ============================================================================
// Edit Panel
// ============================================================================

void World::GUI::prepare_edit_panel()
{
	if (!parent_.guiOptions_.enable_world_edit)
	{
		return;
	}

	nanogui::Window* w = gui_win->createManagedSubWindow("World Editor");

	// Position on right side
	const int xPos = static_cast<int>(parent_.guiOptions_.win_w) - 300;
	w->setPosition({xPos, 1});
	w->setLayout(
		new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 3));
	w->setFixedWidth(290);

	editPanel.window = w;

	// Initially hidden (shown only in WorldEdit mode)
	w->setVisible(false);

	// Tools section
	w->add<nanogui::Label>("Add Elements", "sans-bold");

	auto* toolGrid = w->add<nanogui::Widget>();
	toolGrid->setLayout(new nanogui::GridLayout(
		nanogui::Orientation::Horizontal, 3, nanogui::Alignment::Fill, 2, 2));

	auto createToolBtn = [this, toolGrid](
							 const std::string& label, WorldEditTool tool) -> nanogui::Button*
	{
		auto* btn = toolGrid->add<nanogui::Button>(label);
		btn->setFlags(nanogui::Button::RadioButton);
		btn->setCallback([this, tool]() { worldEditState.currentTool = tool; });
		editPanel.toolButtons.push_back(btn);
		return btn;
	};

	auto* btnSelect = createToolBtn("Select", WorldEditTool::Select);
	btnSelect->setPushed(true);	 // Default tool

	createToolBtn("Move", WorldEditTool::Move);
	createToolBtn("Rotate", WorldEditTool::Rotate);
	createToolBtn("Wall", WorldEditTool::AddWall);
	createToolBtn("Box", WorldEditTool::AddBox);
	createToolBtn("Cylinder", WorldEditTool::AddCylinder);

	// Link all tool buttons in same group
	for (auto* btn : editPanel.toolButtons)
	{
		btn->setButtonGroup(editPanel.toolButtons);
	}

	w->add<nanogui::Label>(" ");

	// Wall properties
	w->add<nanogui::Label>("Wall Properties", "sans-bold");

	auto* wallPropsGrid = w->add<nanogui::Widget>();
	wallPropsGrid->setLayout(new nanogui::GridLayout(
		nanogui::Orientation::Horizontal, 2, nanogui::Alignment::Fill, 2, 2));

	wallPropsGrid->add<nanogui::Label>("Height:");
	editPanel.tbWallHeight = wallPropsGrid->add<nanogui::TextBox>("2.5");
	editPanel.tbWallHeight->setEditable(true);
	editPanel.tbWallHeight->setUnits("m");
	editPanel.tbWallHeight->setFormat("[0-9]*\\.?[0-9]+");
	editPanel.tbWallHeight->setCallback(
		[this](const std::string& str) -> bool
		{
			try
			{
				worldEditState.wallHeight = std::stod(str);
				return true;
			}
			catch (...)
			{
				return false;
			}
		});

	wallPropsGrid->add<nanogui::Label>("Thickness:");
	editPanel.tbWallThickness = wallPropsGrid->add<nanogui::TextBox>("0.15");
	editPanel.tbWallThickness->setEditable(true);
	editPanel.tbWallThickness->setUnits("m");
	editPanel.tbWallThickness->setFormat("[0-9]*\\.?[0-9]+");
	editPanel.tbWallThickness->setCallback(
		[this](const std::string& str) -> bool
		{
			try
			{
				worldEditState.wallThickness = std::stod(str);
				return true;
			}
			catch (...)
			{
				return false;
			}
		});

	w->add<nanogui::Label>(" ");

	// Grid snap
	auto* chkSnap = w->add<nanogui::CheckBox>(
		"Snap to grid (0.5m)", [this](bool b) { worldEditState.snapToGrid = b; });
	chkSnap->setChecked(true);

	w->add<nanogui::Label>(" ");

	// Action buttons
	auto* actionGrid = w->add<nanogui::Widget>();
	actionGrid->setLayout(
		new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 5, 2));

	auto* btnDelete = actionGrid->add<nanogui::Button>("Delete");
	btnDelete->setCallback(
		[this]()
		{
			if (gui_selectedObject.simulable)
			{
				const std::string name = gui_selectedObject.simulable->getName();
				parent_.removeObject(name);
				gui_selectedObject = InfoPerObject();
			}
		});

	w->add<nanogui::Label>(" ");

	// Undo/Redo
	auto* undoGrid = w->add<nanogui::Widget>();
	undoGrid->setLayout(
		new nanogui::BoxLayout(nanogui::Orientation::Horizontal, nanogui::Alignment::Fill, 5, 2));

	editPanel.btnUndo = undoGrid->add<nanogui::Button>("Undo");
	editPanel.btnUndo->setCallback([this]() { undo(); });
	editPanel.btnUndo->setEnabled(false);

	editPanel.btnRedo = undoGrid->add<nanogui::Button>("Redo");
	editPanel.btnRedo->setCallback([this]() { redo(); });
	editPanel.btnRedo->setEnabled(false);

	w->add<nanogui::Label>(" ");

	// Export
	auto* btnExport = w->add<nanogui::Button>("Export World XML...");
	btnExport->setCallback(
		[this]()
		{
			const std::string outFile =
				nanogui::file_dialog({{"xml", "MVSIM World XML"}}, true /*save*/);
			if (!outFile.empty())
			{
				const std::string xml = parent_.exportWorldToXML();
				std::ofstream ofs(outFile);
				ofs << xml;
				std::cout << "[mvsim] Exported world to: " << outFile << std::endl;
			}
		});
}

void World::GUI::update_edit_panel()
{
	// Update undo/redo button states
	if (editPanel.btnUndo)
	{
		editPanel.btnUndo->setEnabled(!worldEditState.undoStack.empty());
	}
	if (editPanel.btnRedo)
	{
		editPanel.btnRedo->setEnabled(!worldEditState.redoStack.empty());
	}
}

// ============================================================================
// Mode Panel
// ============================================================================

void World::GUI::prepare_mode_panel()
{
	nanogui::Window* w = gui_win->createManagedSubWindow("Mode");
	w->setPosition({1, 1});
	w->setLayout(
		new nanogui::BoxLayout(nanogui::Orientation::Vertical, nanogui::Alignment::Fill, 5, 3));
	w->setFixedWidth(280);

	w->add<nanogui::Label>("Operating Mode", "sans-bold");

	// Mode buttons
	auto* btnSimulation = w->add<nanogui::Button>("Simulation");
	btnSimulation->setFlags(nanogui::Button::RadioButton);

	auto* btnWorldEdit = w->add<nanogui::Button>("World Edit");
	btnWorldEdit->setFlags(nanogui::Button::RadioButton);

	// Link buttons in same group
	btnSimulation->setButtonGroup({btnSimulation, btnWorldEdit});
	btnWorldEdit->setButtonGroup({btnSimulation, btnWorldEdit});

	// Set initial state
	if (currentMode == GUIMode::Simulation)
	{
		btnSimulation->setPushed(true);
	}
	else
	{
		btnWorldEdit->setPushed(true);
	}

	btnSimulation->setCallback([this]() { setMode(GUIMode::Simulation); });

	btnWorldEdit->setCallback([this]() { setMode(GUIMode::WorldEdit); });
}

// ============================================================================
// Bottom Bar
// ============================================================================

void World::GUI::prepare_bottom_bar()
{
	if (!parent_.guiOptions_.show_bottom_bar)
	{
		return;
	}

	// Note: nanogui doesn't have a true status bar, so we create a thin window
	// This is a simplified version - a full implementation might need custom widgets
	// For now, we update the existing status window instead
}

void World::GUI::update_bottom_bar()
{
	// This functionality is integrated into update_stats_panel for simplicity
}

// ============================================================================
// Undo/Redo
// ============================================================================

void World::GUI::recordEditAction(const WorldEditAction& action)
{
	// Clear redo stack when new action is recorded
	worldEditState.redoStack.clear();

	// Add to undo stack
	worldEditState.undoStack.push_back(action);

	// Limit stack size
	while (worldEditState.undoStack.size() > WorldEditState::MAX_UNDO_SIZE)
	{
		worldEditState.undoStack.pop_front();
	}

	// Update button states
	if (editPanel.btnUndo)
	{
		editPanel.btnUndo->setEnabled(!worldEditState.undoStack.empty());
	}
	if (editPanel.btnRedo)
	{
		editPanel.btnRedo->setEnabled(!worldEditState.redoStack.empty());
	}
}

void World::GUI::undo()
{
	if (worldEditState.undoStack.empty())
	{
		return;
	}

	const WorldEditAction action = worldEditState.undoStack.back();
	worldEditState.undoStack.pop_back();

	switch (action.type)
	{
		case WorldEditAction::Type::Add:
			// Undo add = delete
			parent_.removeObject(action.objectName);
			break;

		case WorldEditAction::Type::Delete:
			// Undo delete = re-add (using stored XML)
			// TODO: Re-create from XML
			break;

		case WorldEditAction::Type::Move:
			// Undo move = restore previous pose
			{
				auto lck = mrpt::lockHelper(parent_.simulableObjectsMtx_);
				auto it = parent_.simulableObjects_.find(action.objectName);
				if (it != parent_.simulableObjects_.end() && it->second)
				{
					it->second->setPose(action.poseBefore);
				}
			}
			break;

		case WorldEditAction::Type::Modify:
			// TODO: Restore from XML
			break;
	}

	// Move to redo stack
	worldEditState.redoStack.push_back(action);

	// Update button states
	if (editPanel.btnUndo)
	{
		editPanel.btnUndo->setEnabled(!worldEditState.undoStack.empty());
	}
	if (editPanel.btnRedo)
	{
		editPanel.btnRedo->setEnabled(!worldEditState.redoStack.empty());
	}
}

void World::GUI::redo()
{
	if (worldEditState.redoStack.empty())
	{
		return;
	}

	const WorldEditAction action = worldEditState.redoStack.back();
	worldEditState.redoStack.pop_back();

	switch (action.type)
	{
		case WorldEditAction::Type::Add:
			// Redo add = re-create (using stored XML)
			// TODO: Re-create from XML
			break;

		case WorldEditAction::Type::Delete:
			// Redo delete = delete again
			parent_.removeObject(action.objectName);
			break;

		case WorldEditAction::Type::Move:
			// Redo move = apply new pose
			{
				auto lck = mrpt::lockHelper(parent_.simulableObjectsMtx_);
				auto it = parent_.simulableObjects_.find(action.objectName);
				if (it != parent_.simulableObjects_.end() && it->second)
				{
					it->second->setPose(action.poseAfter);
				}
			}
			break;

		case WorldEditAction::Type::Modify:
			// TODO: Apply from XML
			break;
	}

	// Move back to undo stack
	worldEditState.undoStack.push_back(action);

	// Update button states
	if (editPanel.btnUndo)
	{
		editPanel.btnUndo->setEnabled(!worldEditState.undoStack.empty());
	}
	if (editPanel.btnRedo)
	{
		editPanel.btnRedo->setEnabled(!worldEditState.redoStack.empty());
	}
}

void World::GUI::clearUndoHistory()
{
	worldEditState.undoStack.clear();
	worldEditState.redoStack.clear();

	if (editPanel.btnUndo)
	{
		editPanel.btnUndo->setEnabled(false);
	}
	if (editPanel.btnRedo)
	{
		editPanel.btnRedo->setEnabled(false);
	}
}

// ============================================================================
// Wall Preview
// ============================================================================

void World::GUI::updateWallPreview()
{
	if (!worldEditState.isDrawingWall)
	{
		if (worldEditState.previewObject)
		{
			worldEditState.previewObject->setVisibility(false);
		}
		return;
	}

	// Create preview object if needed
	if (!worldEditState.previewObject)
	{
		worldEditState.previewObject = mrpt::opengl::CSetOfObjects::Create();
		worldEditState.previewObject->setName("wall_preview");

		auto lck = mrpt::lockHelper(parent_.guiUserObjectsMtx_);
		if (!parent_.guiUserObjectsViz_)
		{
			parent_.guiUserObjectsViz_ = mrpt::opengl::CSetOfObjects::Create();
		}
		parent_.guiUserObjectsViz_->insert(worldEditState.previewObject);
	}

	// Clear and rebuild preview
	worldEditState.previewObject->clear();

	const auto& p1 = worldEditState.wallStartPoint;
	const auto& p2 = worldEditState.wallPreviewEndPoint;

	// Create a simple line for preview
	auto lines = mrpt::opengl::CSetOfLines::Create();
	lines->setColor_u8(255, 255, 0, 200);  // Yellow
	lines->setLineWidth(3.0f);

	const auto offset = parent_.worldRenderOffset();

	// Bottom line
	lines->appendLine(p1.x + offset.x, p1.y + offset.y, 0.1, p2.x + offset.x, p2.y + offset.y, 0.1);

	// Top line
	lines->appendLine(
		p1.x + offset.x, p1.y + offset.y, worldEditState.wallHeight, p2.x + offset.x,
		p2.y + offset.y, worldEditState.wallHeight);

	// Vertical lines at ends
	lines->appendLine(
		p1.x + offset.x, p1.y + offset.y, 0.1, p1.x + offset.x, p1.y + offset.y,
		worldEditState.wallHeight);
	lines->appendLine(
		p2.x + offset.x, p2.y + offset.y, 0.1, p2.x + offset.x, p2.y + offset.y,
		worldEditState.wallHeight);

	worldEditState.previewObject->insert(lines);
	worldEditState.previewObject->setVisibility(true);
}