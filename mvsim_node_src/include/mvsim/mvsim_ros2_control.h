/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

// ros2_control integration: drives MVSim vehicles with standard ros2_control
// controllers (e.g. diff_drive_controller), in lock-step with the simulation.

#include <mvsim/VehicleBase.h>
#include <mvsim/WheelJointsInterface.h>
#include <mvsim/World.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

namespace controller_manager
{
class ControllerManager;
}

namespace mvsim_node
{
/** Generates a minimal URDF for a vehicle: base_link, one continuous joint
 * per wheel, and a <ros2_control> block for those joints. */
std::string generateVehicleUrdf(
	const mvsim::VehicleBase& veh, mvsim::WheelJointsInterface::CommandMode mode);

/** ros2_control integration for one vehicle: owns a controller manager whose
 * hardware is the vehicle wheels (via mvsim::WheelJointsInterface).
 *
 * The robot description (URDF) is read from the `robot_description` topic in
 * the vehicle namespace. In "generated" mode (default), this class publishes
 * there a URDF generated from the vehicle; otherwise, e.g.
 * robot_state_publisher must publish the user URDF. The <ros2_control>
 * systems whose joints all match wheel joint names of the vehicle are bound to
 * the simulated wheels, whatever their hardware plugin is.
 *
 * The controller manager read/update/write cycle runs in the simulation
 * thread, at its `update_rate`, in simulation time.
 */
class Ros2ControlVehicle
{
   public:
	Ros2ControlVehicle(
		mvsim::World& world, mvsim::VehicleBase& veh, mvsim::WheelJointsInterface& joints,
		const std::string& ns);
	~Ros2ControlVehicle();

	Ros2ControlVehicle(const Ros2ControlVehicle&) = delete;
	Ros2ControlVehicle& operator=(const Ros2ControlVehicle&) = delete;

   private:
	mvsim::World& world_;
	mvsim::VehicleBase& veh_;
	mvsim::WheelJointsInterface& joints_;

	std::shared_ptr<rclcpp::Executor> executor_;
	std::shared_ptr<controller_manager::ControllerManager> cm_;
	rclcpp::Node::SharedPtr descriptionNode_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubDescription_;
	std::thread executorThread_;

	size_t postStepCallbackId_ = 0;
	unsigned int stepsPerUpdate_ = 1;
	unsigned int stepCount_ = 0;
	double updatePeriod_ = 0.01;

	void onSimulationStep(double simTime);
};

}  // namespace mvsim_node
