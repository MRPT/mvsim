/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

/**
 * @file World_gui_mouse.cpp
 * @brief Mouse handling for simulation and world edit modes
 */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TLine3D.h>
#include <mrpt/math/TObject3D.h>
#include <mrpt/math/geometry.h>
#include <mvsim/World.h>

using namespace mvsim;

// ============================================================================
// Simulation Mode Mouse Handling
// ============================================================================

void World::GUI::handleSimulationModeClick(bool leftClick, const mrpt::math::TPoint3D& worldPt)
{
	// Existing "Replace object" functionality
	if (btnReplaceObject && btnReplaceObject->pushed())
	{
		static bool isReplacing = false;

		if (!isReplacing && !leftClick)
		{
			isReplacing = true;
		}

		if (gui_selectedObject.simulable)
		{
			mrpt::math::TPose3D p = gui_selectedObject.simulable->getPose();
			p.x = worldPt.x;
			p.y = worldPt.y;
			gui_selectedObject.simulable->setPose(p);
		}

		if (isReplacing && leftClick)
		{
			isReplacing = false;
			btnReplaceObject->setPushed(false);
		}
		return;
	}

	// Navigation: Set target on left click if vehicle selected
	if (leftClick && !navigationState.targetVehicleName.empty() &&
		parent_.guiOptions_.enable_click_navigation)
	{
		NavigationTarget target;
		target.position = mrpt::math::TPoint2D(worldPt.x, worldPt.y);

		if (navPanel.slMaxSpeed)
		{
			target.maxLinearSpeed = static_cast<double>(navPanel.slMaxSpeed->value());
		}
		else
		{
			target.maxLinearSpeed = 1.0;
		}

		target.active = true;
		target.status = NavigationTarget::Status::Navigating;
		target.setTime = parent_.get_simul_time();

		navigationState.target = target;
		parent_.setNavigationTarget(navigationState.targetVehicleName, target);

		updateNavigationTargetMarker();
	}
}

// ============================================================================
// World Edit Mode Mouse Handling
// ============================================================================

void World::GUI::handleWorldEditModeClick(bool leftClick, const mrpt::math::TPoint3D& worldPt)
{
	mrpt::math::TPoint3D snapPt = worldPt;

	// Apply grid snap if enabled
	if (worldEditState.snapToGrid)
	{
		const double gs = worldEditState.gridSnapSize;
		snapPt.x = std::round(worldPt.x / gs) * gs;
		snapPt.y = std::round(worldPt.y / gs) * gs;
	}

	switch (worldEditState.currentTool)
	{
		case WorldEditTool::Select:
		{
			if (leftClick)
			{
				// TODO: Ray cast to find clicked object
				// For now, deselect
			}
			break;
		}

		case WorldEditTool::AddWall:
		{
			if (!worldEditState.isDrawingWall)
			{
				// Preview mode - update preview endpoint
				worldEditState.wallPreviewEndPoint = snapPt;

				if (leftClick)
				{
					// First click: set start point
					worldEditState.wallStartPoint = snapPt;
					worldEditState.isDrawingWall = true;
				}
			}
			else
			{
				// Currently drawing - update preview
				worldEditState.wallPreviewEndPoint = snapPt;
				updateWallPreview();

				if (leftClick)
				{
					// Second click: create wall
					const std::string wallName = parent_.addWall(
						mrpt::math::TPoint2D(
							worldEditState.wallStartPoint.x, worldEditState.wallStartPoint.y),
						mrpt::math::TPoint2D(snapPt.x, snapPt.y), worldEditState.wallHeight,
						worldEditState.wallThickness, worldEditState.wallTexture);

					if (!wallName.empty())
					{
						// Record for undo
						WorldEditAction action;
						action.type = WorldEditAction::Type::Add;
						action.objectName = wallName;
						action.objectClass = "vertical_plane";
						action.timestamp = parent_.get_simul_time();
						recordEditAction(action);
					}

					worldEditState.isDrawingWall = false;

					// Clear preview
					if (worldEditState.previewObject)
					{
						worldEditState.previewObject->setVisibility(false);
					}
				}
			}
			break;
		}

		case WorldEditTool::AddBox:
		{
			if (leftClick)
			{
				const mrpt::math::TPose3D pose(
					snapPt.x, snapPt.y,
					0.25,  // Half height above ground
					0.0, 0.0, 0.0);
				const std::string blockName = parent_.addBlock("box", pose);

				if (!blockName.empty())
				{
					WorldEditAction action;
					action.type = WorldEditAction::Type::Add;
					action.objectName = blockName;
					action.objectClass = "box";
					action.timestamp = parent_.get_simul_time();
					recordEditAction(action);
				}
			}
			break;
		}

		case WorldEditTool::AddCylinder:
		{
			if (leftClick)
			{
				const mrpt::math::TPose3D pose(snapPt.x, snapPt.y, 0.25, 0.0, 0.0, 0.0);
				const std::string blockName = parent_.addBlock("cylinder", pose);

				if (!blockName.empty())
				{
					WorldEditAction action;
					action.type = WorldEditAction::Type::Add;
					action.objectName = blockName;
					action.objectClass = "cylinder";
					action.timestamp = parent_.get_simul_time();
					recordEditAction(action);
				}
			}
			break;
		}

		case WorldEditTool::AddSphere:
		{
			if (leftClick)
			{
				const mrpt::math::TPose3D pose(snapPt.x, snapPt.y, 0.25, 0.0, 0.0, 0.0);
				const std::string blockName = parent_.addBlock("sphere", pose);

				if (!blockName.empty())
				{
					WorldEditAction action;
					action.type = WorldEditAction::Type::Add;
					action.objectName = blockName;
					action.objectClass = "sphere";
					action.timestamp = parent_.get_simul_time();
					recordEditAction(action);
				}
			}
			break;
		}

		case WorldEditTool::Move:
		{
			if (gui_selectedObject.simulable && leftClick)
			{
				mrpt::math::TPose3D p = gui_selectedObject.simulable->getPose();

				WorldEditAction action;
				action.type = WorldEditAction::Type::Move;
				action.objectName = gui_selectedObject.simulable->getName();
				action.poseBefore = p;

				p.x = snapPt.x;
				p.y = snapPt.y;

				action.poseAfter = p;
				action.timestamp = parent_.get_simul_time();

				gui_selectedObject.simulable->setPose(p);
				recordEditAction(action);
			}
			break;
		}

		case WorldEditTool::Rotate:
		{
			// TODO: Implement rotation by mouse drag
			break;
		}

		case WorldEditTool::Delete:
		{
			// TODO: Select and delete object at click point via ray casting
			break;
		}

		case WorldEditTool::AddRamp:
		{
			// TODO: Implement ramp creation
			break;
		}
	}
}