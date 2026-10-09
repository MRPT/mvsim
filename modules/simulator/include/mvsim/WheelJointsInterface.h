/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/PID_Controller.h>

#include <mutex>
#include <string>
#include <vector>

namespace rapidxml
{
template <class Ch>
class xml_node;
}

namespace mvsim
{
class VehicleBase;

/** Per-wheel joint commands and states, exchanged with an external,
 * joint-level controller (e.g. ros2_control hardware interfaces). It is
 * independent of ROS.
 *
 * Joint conventions: positive joint velocity, position and effort make the
 * vehicle move forward.
 *
 * Commands and states are exchanged under a mutex, so they can be accessed
 * from any thread.
 *
 * XML parameters (children of the `<controller>` node):
 *  - `command_interface`: `velocity` (default) or `effort`.
 *  - `KP`, `KI`, `KD`, `max_torque`: inner per-wheel velocity PID, in the same
 *    units than the `twist_pid` controller (error in m/s at the wheel rim,
 *    output in Nm).
 *  - `controllers_yaml`: file with parameters for the controller manager and
 *    the controllers (used by the ROS node).
 *  - `robot_description`: `generated` (default): the ROS node generates a
 *    URDF with the wheel joints; or `topic`: it is read from the
 *    `robot_description` topic (e.g. published by robot_state_publisher).
 *
 *  \ingroup mvsim_simulator_module
 */
class WheelJointsInterface
{
   public:
	enum class CommandMode : uint8_t
	{
		Velocity = 0,
		Effort
	};

	struct JointState
	{
		double position = 0;  //!< [rad] (continuous, unwrapped)
		double velocity = 0;  //!< [rad/s]
		double effort = 0;	//!< [Nm] (last applied)
	};

	/** Loads XML parameters. Relative paths are resolved with the world
	 * base directory. */
	void loadConfig(const rapidxml::xml_node<char>& node, const VehicleBase& veh);

	CommandMode commandMode() const { return commandMode_; }
	const std::string& controllersYaml() const { return controllersYaml_; }
	bool robotDescriptionFromTopic() const { return robotDescriptionFromTopic_; }

	/** Sets the commands (velocities in rad/s, or efforts in Nm), one per
	 * wheel, in the vehicle wheel order. */
	void setCommands(const std::vector<double>& commands);

	/** Current joint states, in the vehicle wheel order. */
	std::vector<JointState> getStates() const;

	/** Called by the vehicle controller at each simulation step.
	 * \return The wheel torques, one per wheel, in mvsim sign convention
	 * (positive means backwards). */
	std::vector<double> computeWheelTorques(const VehicleBase& veh, double dt);

	/** Called by the vehicle controller after each simulation step, to update
	 * the joint states */
	void updateStates(const VehicleBase& veh);

	double KP = 10, KI = 0, KD = 0;
	double max_torque = 100;  //!< [Nm]

   private:
	CommandMode commandMode_ = CommandMode::Velocity;
	std::string controllersYaml_;
	bool robotDescriptionFromTopic_ = false;

	mutable std::mutex mtx_;
	std::vector<double> commands_;
	std::vector<JointState> states_;
	std::vector<double> lastTorques_;  //!< mvsim sign convention
	std::vector<PID_Controller> pids_;
};

}  // namespace mvsim
