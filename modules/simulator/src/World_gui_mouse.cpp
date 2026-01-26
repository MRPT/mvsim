/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  +-------------------------------------------------------------------------+ */

/**
 * @file World_gui_mouse_new.cpp
 * @brief Mouse handling for simulation and world edit modes.
 *
 * This file implements mouse click handling that works with the new
 * dockable panel-based GUI, supporting both simulation mode (navigation)
 * and world edit mode (object placement).
 */

#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mvsim/World.h>

#include <cmath>

using namespace mvsim;

// ============================================================================
// Simulation Mode Click Handling
// ============================================================================
void World::GUI::handleSimulationModeClick(bool leftClick, const mrpt::math::TPoint3D& worldPt)
{
	if (!leftClick)
	{
		return;
	}

	// Check if a vehicle is selected for navigation
	if (!leftPanel.navTab.cbVehicle)
	{
		return;
	}

	const int vehIdx = leftPanel.navTab.cbVehicle->selectedIndex();
	if (vehIdx <= 0)
	{
		// No vehicle selected - could do object selection here
		setStatusMessage("Select a vehicle for navigation");
		return;
	}

	const auto& items = leftPanel.navTab.cbVehicle->items();
	if (vehIdx >= static_cast<int>(items.size()))
	{
		return;
	}

	const std::string vehName = items[vehIdx];

	// Create navigation target
	NavigationTarget target;
	target.position.x = worldPt.x;
	target.position.y = worldPt.y;
	target.active = true;
	target.status = NavigationTarget::Status::Navigating;
	target.setTime = parent_.get_simul_time();

	// Get max speed from slider
	if (leftPanel.navTab.slMaxSpeed)
	{
		target.maxLinearSpeed = static_cast<double>(leftPanel.navTab.slMaxSpeed->value());
	}

	// Set the navigation target
	parent_.setNavigationTarget(vehName, target);

	// Update the marker visualization
	updateNavigationTargetMarker();

	// Update status
	setStatusMessage(
		mrpt::format(
			"Navigation target set for %s at (%.2f, %.2f)", vehName.c_str(), worldPt.x, worldPt.y));
}

// ============================================================================
// World Edit Mode Click Handling
// ============================================================================
void World::GUI::handleWorldEditModeClick(bool leftClick, const mrpt::math::TPoint3D& worldPt)
{
	if (!leftClick)
	{
		return;
	}

	// Apply grid snap if enabled
	mrpt::math::TPoint3D snappedPt = worldPt;
	if (rightPanel.addTab.cbSnapToGrid && rightPanel.addTab.cbSnapToGrid->checked())
	{
		double gridSize = 0.5;	// Default
		if (rightPanel.addTab.tbGridSize)
		{
			try
			{
				gridSize = std::stod(rightPanel.addTab.tbGridSize->value());
			}
			catch (...)
			{
				gridSize = 0.5;
			}
		}

		if (gridSize > 0.01)
		{
			snappedPt.x = std::round(worldPt.x / gridSize) * gridSize;
			snappedPt.y = std::round(worldPt.y / gridSize) * gridSize;
		}
	}

	switch (worldEditState.currentTool)
	{
		case WorldEditTool::Select:
			handleSelectTool(snappedPt);
			break;

		case WorldEditTool::Move:
			handleMoveTool(snappedPt);
			break;

		case WorldEditTool::Rotate:
			handleRotateTool(snappedPt);
			break;

		case WorldEditTool::AddWall:
			handleAddWallTool(snappedPt);
			break;

		case WorldEditTool::AddBox:
			handleAddBlockTool(snappedPt, "box");
			break;

		case WorldEditTool::AddCylinder:
			handleAddBlockTool(snappedPt, "cylinder");
			break;

		case WorldEditTool::AddSphere:
			handleAddBlockTool(snappedPt, "sphere");
			break;

		case WorldEditTool::AddRamp:
			handleAddBlockTool(snappedPt, "ramp");
			break;

		case WorldEditTool::Delete:
			handleDeleteTool(snappedPt);
			break;

		default:
			break;
	}
}

// ============================================================================
// Individual Tool Handlers
// ============================================================================

void World::GUI::handleSelectTool(const mrpt::math::TPoint3D& worldPt)
{
	// TODO: Implement ray casting to select objects at click position
	// For now, just update status
	setStatusMessage(
		mrpt::format("Select at (%.2f, %.2f) - TODO: implement ray casting", worldPt.x, worldPt.y));
}

void World::GUI::handleMoveTool(const mrpt::math::TPoint3D& worldPt)
{
	if (!gui_selectedObject.simulable)
	{
		setStatusMessage("No object selected to move");
		return;
	}

	// Record undo action
	WorldEditAction action;
	action.type = WorldEditAction::Type::Move;
	action.objectName = gui_selectedObject.simulable->getName();
	action.poseBefore = gui_selectedObject.simulable->getPose();
	action.timestamp = parent_.get_simul_time();

	// Move the object
	auto newPose = action.poseBefore;
	newPose.x = worldPt.x;
	newPose.y = worldPt.y;
	gui_selectedObject.simulable->setPose(newPose);

	action.poseAfter = newPose;
	recordEditAction(action);

	// Update panels
	update_right_panel();

	setStatusMessage(
		mrpt::format("Moved %s to (%.2f, %.2f)", action.objectName.c_str(), worldPt.x, worldPt.y));
}

void World::GUI::handleRotateTool(const mrpt::math::TPoint3D& worldPt)
{
	if (!gui_selectedObject.simulable)
	{
		setStatusMessage("No object selected to rotate");
		return;
	}

	// Calculate angle from object center to click point
	const auto pose = gui_selectedObject.simulable->getPose();
	const double dx = worldPt.x - pose.x;
	const double dy = worldPt.y - pose.y;
	const double newYaw = std::atan2(dy, dx);

	// Record undo action
	WorldEditAction action;
	action.type = WorldEditAction::Type::Move;
	action.objectName = gui_selectedObject.simulable->getName();
	action.poseBefore = pose;
	action.timestamp = parent_.get_simul_time();

	// Apply rotation
	auto newPose = pose;
	newPose.yaw = newYaw;
	gui_selectedObject.simulable->setPose(newPose);

	action.poseAfter = newPose;
	recordEditAction(action);

	update_right_panel();

	setStatusMessage(
		mrpt::format("Rotated %s to %.1f deg", action.objectName.c_str(), newYaw * 180.0 / M_PI));
}

void World::GUI::handleAddWallTool(const mrpt::math::TPoint3D& worldPt)
{
	if (!worldEditState.wallDrawingActive)
	{
		// First click - start wall
		worldEditState.wallDrawingActive = true;
		worldEditState.wallStartPoint.x = worldPt.x;
		worldEditState.wallStartPoint.y = worldPt.y;
		worldEditState.wallPreviewEndPoint = worldEditState.wallStartPoint;

		setStatusMessage(
			mrpt::format(
				"Wall start: (%.2f, %.2f) - Click to set end point", worldPt.x, worldPt.y));

		// Create preview visualization
		updateWallPreview();
	}
	else
	{
		// Second click - create wall
		worldEditState.wallDrawingActive = false;

		// Get wall properties from UI
		double height = 2.5;
		double thickness = 0.15;

		if (rightPanel.addTab.tbWallHeight)
		{
			try
			{
				height = std::stod(rightPanel.addTab.tbWallHeight->value());
			}
			catch (...)
			{
				height = 2.5;
			}
		}

		if (rightPanel.addTab.tbWallThickness)
		{
			try
			{
				thickness = std::stod(rightPanel.addTab.tbWallThickness->value());
			}
			catch (...)
			{
				thickness = 0.15;
			}
		}

		// Create the wall
		const std::string wallName = parent_.addWall(
			worldEditState.wallStartPoint.x, worldEditState.wallStartPoint.y, worldPt.x, worldPt.y,
			height, thickness);

		if (!wallName.empty())
		{
			// Record undo action
			WorldEditAction action;
			action.type = WorldEditAction::Type::Add;
			action.objectName = wallName;
			action.objectClass = "VerticalPlane";
			action.timestamp = parent_.get_simul_time();
			recordEditAction(action);

			setStatusMessage(mrpt::format("Created wall: %s", wallName.c_str()));
		}
		else
		{
			setStatusMessage("Failed to create wall");
		}

		// Clear preview
		updateWallPreview();
	}
}

void World::GUI::handleAddBlockTool(
	const mrpt::math::TPoint3D& worldPt, const std::string& blockType)
{
	// Get block properties from UI
	double size = 1.0;
	double mass = 10.0;

	if (rightPanel.addTab.tbBlockSize)
	{
		try
		{
			size = std::stod(rightPanel.addTab.tbBlockSize->value());
		}
		catch (...)
		{
			size = 1.0;
		}
	}

	if (rightPanel.addTab.tbBlockMass)
	{
		try
		{
			mass = std::stod(rightPanel.addTab.tbBlockMass->value());
		}
		catch (...)
		{
			mass = 10.0;
		}
	}

	// Create the block
	const std::string blockName = parent_.addBlock(
		blockType, worldPt.x, worldPt.y,
		worldPt.z + size / 2.0,	 // Place on ground
		size, mass);

	if (!blockName.empty())
	{
		// Record undo action
		WorldEditAction action;
		action.type = WorldEditAction::Type::Add;
		action.objectName = blockName;
		action.objectClass = blockType;
		action.timestamp = parent_.get_simul_time();
		recordEditAction(action);

		setStatusMessage(
			mrpt::format(
				"Created %s: %s at (%.2f, %.2f)", blockType.c_str(), blockName.c_str(), worldPt.x,
				worldPt.y));
	}
	else
	{
		setStatusMessage(mrpt::format("Failed to create %s", blockType.c_str()));
	}
}

void World::GUI::handleDeleteTool(const mrpt::math::TPoint3D& worldPt)
{
	// TODO: Implement ray casting to find object at click position
	// For now, delete selected object if any

	if (!gui_selectedObject.simulable)
	{
		setStatusMessage("No object selected to delete");
		return;
	}

	const std::string name = gui_selectedObject.simulable->getName();

	// Record undo action (save XML for potential undo)
	WorldEditAction action;
	action.type = WorldEditAction::Type::Delete;
	action.objectName = name;
	// TODO: action.xmlBefore = serialize object to XML
	action.poseBefore = gui_selectedObject.simulable->getPose();
	action.timestamp = parent_.get_simul_time();
	recordEditAction(action);

	// Remove the object
	parent_.removeObject(name);

	// Clear selection
	gui_selectedObject = InfoPerObject();

	update_right_panel();

	setStatusMessage(mrpt::format("Deleted: %s", name.c_str()));
}

// ============================================================================
// Wall Preview Visualization
// ============================================================================
void World::GUI::updateWallPreview()
{
	const std::string previewName = "__wall_preview__";

	std::lock_guard<std::mutex> lck(gui_win->background_scene_mtx);

	auto existingPreview = gui_win->background_scene->getByName(previewName);

	if (!worldEditState.wallDrawingActive)
	{
		// Remove preview if it exists
		if (existingPreview)
		{
			gui_win->background_scene->removeObject(existingPreview);
		}
		return;
	}

	// Create or update preview lines
	mrpt::opengl::CSetOfLines::Ptr lines;

	if (existingPreview)
	{
		lines = std::dynamic_pointer_cast<mrpt::opengl::CSetOfLines>(existingPreview);
	}
	else
	{
		lines = mrpt::opengl::CSetOfLines::Create();
		lines->setName(previewName);
		lines->setColor_u8(255, 255, 0, 200);  // Yellow, semi-transparent
		lines->setLineWidth(2.0f);
		gui_win->background_scene->insert(lines);
	}

	if (!lines)
	{
		return;
	}

	lines->clear();

	// Get wall height for preview
	double height = 2.5;
	if (rightPanel.addTab.tbWallHeight)
	{
		try
		{
			height = std::stod(rightPanel.addTab.tbWallHeight->value());
		}
		catch (...)
		{
			height = 2.5;
		}
	}

	const double x0 = worldEditState.wallStartPoint.x;
	const double y0 = worldEditState.wallStartPoint.y;
	const double x1 = worldEditState.wallPreviewEndPoint.x;
	const double y1 = worldEditState.wallPreviewEndPoint.y;

	// Draw wall outline (bottom, top, and vertical edges)
	// Bottom edge
	lines->appendLine(x0, y0, 0.0, x1, y1, 0.0);
	// Top edge
	lines->appendLine(x0, y0, height, x1, y1, height);
	// Vertical edges
	lines->appendLine(x0, y0, 0.0, x0, y0, height);
	lines->appendLine(x1, y1, 0.0, x1, y1, height);
}

// ============================================================================
// Navigation Target Marker
// ============================================================================
void World::GUI::updateNavigationTargetMarker()
{
	const std::string markerName = "__nav_target_marker__";

	std::lock_guard<std::mutex> lck(gui_win->background_scene_mtx);

	auto existingMarker = gui_win->background_scene->getByName(markerName);

	// Check if we should show the marker
	if (!showNavigationMarker_ || !leftPanel.navTab.cbVehicle)
	{
		if (existingMarker)
		{
			gui_win->background_scene->removeObject(existingMarker);
		}
		return;
	}

	const int vehIdx = leftPanel.navTab.cbVehicle->selectedIndex();
	if (vehIdx <= 0)
	{
		if (existingMarker)
		{
			gui_win->background_scene->removeObject(existingMarker);
		}
		return;
	}

	const auto& items = leftPanel.navTab.cbVehicle->items();
	if (vehIdx >= static_cast<int>(items.size()))
	{
		return;
	}

	const auto target = parent_.getNavigationTarget(items[vehIdx]);

	if (!target || !target->active)
	{
		if (existingMarker)
		{
			gui_win->background_scene->removeObject(existingMarker);
		}
		return;
	}

	// Create or update marker
	mrpt::opengl::CSetOfObjects::Ptr marker;

	if (existingMarker)
	{
		marker = std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(existingMarker);
	}
	else
	{
		marker = mrpt::opengl::CSetOfObjects::Create();
		marker->setName(markerName);

		// Create cylinder (pole)
		auto cyl = mrpt::opengl::CCylinder::Create(0.05f, 0.05f, 1.0f);
		cyl->setColor_u8(0, 200, 0, 200);  // Green
		marker->insert(cyl);

		// Create sphere on top
		auto sphere = mrpt::opengl::CSphere::Create(0.15f);
		sphere->setColor_u8(0, 255, 0, 255);
		sphere->setLocation(0, 0, 1.15f);
		marker->insert(sphere);

		gui_win->background_scene->insert(marker);
	}

	if (marker)
	{
		marker->setLocation(target->position.x, target->position.y, 0.0);
	}
}

// ============================================================================
// Mouse motion update for wall preview
// ============================================================================
void World::GUI::onMouseMotion(const mrpt::math::TPoint3D& worldPt)
{
	// Update mouse position display
	updateMousePosition(worldPt);

	// Update wall preview if drawing
	if (currentMode == GUIMode::WorldEdit && worldEditState.currentTool == WorldEditTool::AddWall &&
		worldEditState.wallDrawingActive)
	{
		// Apply grid snap
		mrpt::math::TPoint3D snappedPt = worldPt;
		if (rightPanel.addTab.cbSnapToGrid && rightPanel.addTab.cbSnapToGrid->checked())
		{
			double gridSize = 0.5;
			if (rightPanel.addTab.tbGridSize)
			{
				try
				{
					gridSize = std::stod(rightPanel.addTab.tbGridSize->value());
				}
				catch (...)
				{
					gridSize = 0.5;
				}
			}

			if (gridSize > 0.01)
			{
				snappedPt.x = std::round(worldPt.x / gridSize) * gridSize;
				snappedPt.y = std::round(worldPt.y / gridSize) * gridSize;
			}
		}

		worldEditState.wallPreviewEndPoint.x = snappedPt.x;
		worldEditState.wallPreviewEndPoint.y = snappedPt.y;
		updateWallPreview();
	}
}