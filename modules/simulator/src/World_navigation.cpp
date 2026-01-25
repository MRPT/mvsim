/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

/**
 * @file World_navigation.cpp
 * @brief Implementation of navigation target handling and dynamic world editing
 */

#include <mrpt/core/format.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/math/wrap2pi.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/VerticalPlane.h>

#include <algorithm>
#include <cmath>
#include <rapidxml.hpp>
#include <sstream>

using namespace mvsim;

// ============================================================================
// Navigation Target API
// ============================================================================

void World::setNavigationTarget(const std::string& vehicleName, const NavigationTarget& target)
{
	std::lock_guard<std::mutex> lck(navigationTargetsMtx_);
	navigationTargets_[vehicleName] = target;
}

void World::clearNavigationTarget(const std::string& vehicleName)
{
	std::lock_guard<std::mutex> lck(navigationTargetsMtx_);
	navigationTargets_.erase(vehicleName);
}

std::optional<NavigationTarget> World::getNavigationTarget(const std::string& vehicleName) const
{
	std::lock_guard<std::mutex> lck(navigationTargetsMtx_);

	auto it = navigationTargets_.find(vehicleName);
	if (it != navigationTargets_.end())
	{
		return it->second;
	}
	return std::nullopt;
}

void World::processNavigationTargets(const TSimulContext& context)
{
	std::lock_guard<std::mutex> lck(navigationTargetsMtx_);

	for (auto& [vehicleName, target] : navigationTargets_)
	{
		if (!target.active)
		{
			continue;
		}

		// Find vehicle
		auto vehIt = vehicles_.find(vehicleName);
		if (vehIt == vehicles_.end())
		{
			continue;
		}

		auto& veh = vehIt->second;
		const auto currentPose = veh->getPose();

		// Compute distance to target
		const double dx = target.position.x - currentPose.x;
		const double dy = target.position.y - currentPose.y;
		const double distance = std::sqrt(dx * dx + dy * dy);

		// Check if reached
		if (distance < target.reachDistanceTolerance)
		{
			// Check heading if specified
			if (target.targetHeading.has_value())
			{
				const double headingError = std::abs(
					mrpt::math::angDistance(currentPose.yaw, target.targetHeading.value()));
				if (headingError < target.reachHeadingTolerance)
				{
					target.status = NavigationTarget::Status::Reached;
					target.active = false;
					continue;
				}
			}
			else
			{
				target.status = NavigationTarget::Status::Reached;
				target.active = false;
				continue;
			}
		}

		// Stuck detection
		const double progressThreshold = 0.01;	// 1 cm
		if (std::abs(distance - target.lastDistanceToTarget) < progressThreshold)
		{
			target.stuckTime += context.dt;
			if (target.stuckTime > 5.0)	 // 5 seconds stuck
			{
				target.status = NavigationTarget::Status::Stuck;
				// Don't deactivate, let user decide
			}
		}
		else
		{
			target.stuckTime = 0.0;
			target.status = NavigationTarget::Status::Navigating;
		}
		target.lastDistanceToTarget = distance;

		// Compute velocity command
		mrpt::math::TTwist2D twist = computeNavigationTwist(currentPose, target);

		// Send to vehicle controller
		auto* controller = veh->getControllerInterface();
		if (controller)
		{
			controller->setTwistCommand(twist);
		}
	}
}

mrpt::math::TTwist2D World::computeNavigationTwist(
	const mrpt::math::TPose3D& currentPose, const NavigationTarget& target) const
{
	mrpt::math::TTwist2D twist;

	// Compute vector to target
	const double dx = target.position.x - currentPose.x;
	const double dy = target.position.y - currentPose.y;
	const double distance = std::sqrt(dx * dx + dy * dy);

	if (distance < 1e-6)
	{
		// Already at target
		twist.vx = 0.0;
		twist.vy = 0.0;
		twist.omega = 0.0;
		return twist;
	}

	// Desired heading to target
	const double targetHeading = std::atan2(dy, dx);

	// Heading error (normalized to [-pi, pi])
	double headingError = mrpt::math::angDistance(currentPose.yaw, targetHeading);

	// Simple proportional controller
	const double Kp_angular = 2.0;	// Angular proportional gain
	const double Kp_linear = 1.0;  // Linear proportional gain

	// Angular velocity to turn towards target
	double omega = Kp_angular * headingError;

	// Clamp angular velocity
	omega = std::clamp(omega, -target.maxAngularSpeed, target.maxAngularSpeed);

	// Linear velocity - reduce when not facing target
	const double facingFactor = std::cos(headingError);

	double vx = 0.0;
	if (facingFactor > 0.0)
	{
		// Moving forward: scale by how well we're facing the target
		vx = Kp_linear * distance * facingFactor;

		// Slow down when close to target
		const double slowdownRadius = 0.5;	// meters
		if (distance < slowdownRadius)
		{
			vx *= (distance / slowdownRadius);
		}
	}

	// Clamp linear velocity
	vx = std::clamp(vx, 0.0, target.maxLinearSpeed);

	twist.vx = vx;
	twist.vy = 0.0;	 // Differential drive: no lateral motion
	twist.omega = omega;

	return twist;
}

// ============================================================================
// World Editing API
// ============================================================================

std::string World::addWall(
	const mrpt::math::TPoint2D& p1, const mrpt::math::TPoint2D& p2, double height, double thickness,
	const std::string& texture)
{
	// Generate unique name
	const std::string wallName = mrpt::format("wall_%05zu", dynamicWallCounter_++);

	// Build XML for the wall (using VerticalPlane with thickness)
	std::string textureStr = texture;
	if (textureStr.empty())
	{
		textureStr =
			"https://mrpt.github.io/mvsim-models/"
			"textures-cgbookcase/wall-bricks-01.png";
	}

	const std::string wallXml = mrpt::format(
		R"(<element class="vertical_plane" name="%s">
            <cull_face>NONE</cull_face>
            <x0>%f</x0> <y0>%f</y0>
            <x1>%f</x1> <y1>%f</y1>
            <z>0.0</z>
            <height>%f</height>
            <thickness>%f</thickness>
            <texture>%s</texture>
            <texture_size_x>2.0</texture_size_x>
            <texture_size_y>2.0</texture_size_y>
        </element>)",
		wallName.c_str(), p1.x, p1.y, p2.x, p2.y, height, thickness, textureStr.c_str());

	// Parse and add to world
	try
	{
		rapidxml::xml_document<> doc;
		std::vector<char> xmlCopy(wallXml.begin(), wallXml.end());
		xmlCopy.push_back('\0');
		doc.parse<0>(xmlCopy.data());

		auto* node = doc.first_node("element");
		if (node)
		{
			auto we = WorldElementBase::factory(this, node, "vertical_plane");
			if (we)
			{
				worldElements_.push_back(we);

				MRPT_LOG_INFO_FMT(
					"[World::addWall] Created wall '%s' from (%.2f, %.2f) to "
					"(%.2f, %.2f), height=%.2f, thickness=%.2f",
					wallName.c_str(), p1.x, p1.y, p2.x, p2.y, height, thickness);

				return wallName;
			}
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_FMT("[World::addWall] Failed to create wall: %s", e.what());
	}

	return "";
}

std::string World::addBlock(
	const std::string& blockClass, const mrpt::math::TPose3D& pose, const std::string& params)
{
	// Generate unique name
	const std::string blockName =
		mrpt::format("block_%s_%05zu", blockClass.c_str(), dynamicBlockCounter_++);

	// Default parameters based on class
	std::string geometryXml;
	std::string colorHex = "#808080";

	if (blockClass == "box")
	{
		geometryXml = R"(<geometry type="box" lx="0.5" ly="0.5" lz="0.5" />)";
	}
	else if (blockClass == "cylinder")
	{
		geometryXml = R"(<geometry type="cylinder" radius="0.25" length="0.5" />)";
	}
	else if (blockClass == "sphere")
	{
		geometryXml = R"(<geometry type="sphere" radius="0.25" />)";
	}
	else
	{
		// Default to box
		geometryXml = R"(<geometry type="box" lx="0.5" ly="0.5" lz="0.5" />)";
	}

	// Build XML for the block
	const std::string blockXml = mrpt::format(
		R"(<block name="%s">
            <mass>10.0</mass>
            <ground_friction>0.5</ground_friction>
            <color>%s</color>
            %s
            %s
            <init_pose3d>%f %f %f %f %f %f</init_pose3d>
        </block>)",
		blockName.c_str(), colorHex.c_str(), geometryXml.c_str(), params.c_str(), pose.x, pose.y,
		pose.z, mrpt::RAD2DEG(pose.yaw), mrpt::RAD2DEG(pose.pitch), mrpt::RAD2DEG(pose.roll));

	// Parse and add to world
	try
	{
		auto block = Block::factory(this, blockXml);
		if (block)
		{
			block->setName(blockName);
			insertBlock(block);
			block->create_multibody_system(*box2d_world_);

			MRPT_LOG_INFO_FMT(
				"[World::addBlock] Created block '%s' of class '%s' at "
				"(%.2f, %.2f, %.2f)",
				blockName.c_str(), blockClass.c_str(), pose.x, pose.y, pose.z);

			return blockName;
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_FMT("[World::addBlock] Failed to create block: %s", e.what());
	}

	return "";
}

bool World::removeObject(const std::string& objectName)
{
	auto lck = mrpt::lockHelper(simulableObjectsMtx_);

	// Try to find and remove from simulable objects
	auto simIt = simulableObjects_.find(objectName);
	if (simIt == simulableObjects_.end())
	{
		MRPT_LOG_WARN_FMT("[World::removeObject] Object '%s' not found", objectName.c_str());
		return false;
	}

	// Determine type and remove from appropriate container
	Simulable::Ptr obj = simIt->second;

	// Check if it's a block
	if (auto block = std::dynamic_pointer_cast<Block>(obj); block)
	{
		// Remove from blocks_ map
		for (auto it = blocks_.begin(); it != blocks_.end(); ++it)
		{
			if (it->second == block)
			{
				// Remove Box2D body
				if (auto* body = block->getBox2DBlockBody(); body)
				{
					box2d_world_->DestroyBody(body);
				}
				blocks_.erase(it);
				break;
			}
		}
	}

	// Check if it's a world element
	for (auto it = worldElements_.begin(); it != worldElements_.end(); ++it)
	{
		if ((*it)->getName() == objectName)
		{
			worldElements_.erase(it);
			break;
		}
	}

	// Remove from simulable objects
	simulableObjects_.erase(simIt);

	// Mark LUT cache as dirty
	lut2d_objects_is_up_to_date_ = false;

	MRPT_LOG_INFO_FMT("[World::removeObject] Removed object '%s'", objectName.c_str());

	return true;
}

std::string World::exportWorldToXML() const
{
	std::ostringstream oss;

	oss << R"(<?xml version="1.0" encoding="UTF-8"?>)" << "\n";
	oss << R"(<mvsim_world version="1.0">)" << "\n";
	oss << "\n";

	// Export world elements
	oss << "  <!-- World Elements -->\n";
	for (const auto& we : worldElements_)
	{
		if (!we)
		{
			continue;
		}

		// Get element type from class name (simplified)
		const std::string name = we->getName();

		// For VerticalPlane (walls)
		if (auto* vp = dynamic_cast<VerticalPlane*>(we.get()); vp)
		{
			oss << mrpt::format(
				R"(  <element class="vertical_plane" name="%s">)"
				"\n",
				name.c_str());
			// Note: Would need accessors to export full properties
			oss << "  </element>\n";
		}
		// Add other element types as needed
	}

	oss << "\n";

	// Export blocks
	oss << "  <!-- Blocks -->\n";
	for (const auto& [blockName, block] : blocks_)
	{
		if (!block)
		{
			continue;
		}

		const auto pose = block->getPose();
		oss << mrpt::format(
			R"(  <block name="%s">)"
			"\n",
			blockName.c_str());
		oss << mrpt::format(
			"    <init_pose3d>%f %f %f %f %f %f</init_pose3d>\n", pose.x, pose.y, pose.z,
			mrpt::RAD2DEG(pose.yaw), mrpt::RAD2DEG(pose.pitch), mrpt::RAD2DEG(pose.roll));
		oss << "  </block>\n";
	}

	oss << "\n";

	// Export vehicles
	oss << "  <!-- Vehicles -->\n";
	for (const auto& [vehName, veh] : vehicles_)
	{
		if (!veh)
		{
			continue;
		}

		const auto pose = veh->getPose();
		oss << mrpt::format(
			R"(  <!-- Vehicle: %s at (%.2f, %.2f) -->)"
			"\n",
			vehName.c_str(), pose.x, pose.y);
	}

	oss << "\n";
	oss << "</mvsim_world>\n";

	return oss.str();
}