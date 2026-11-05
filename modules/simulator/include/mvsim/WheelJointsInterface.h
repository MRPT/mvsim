/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/PID_Controller.h>

#include <array>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rapidxml
{
template <class Ch>
class xml_node;
}

namespace mvsim
{
class VehicleBase;

/** Joint commands and states of a vehicle (wheel spin, wheel steering, and
 * the virtual joints of a bicycle model), exchanged with an external,
 * joint-level controller (e.g. ros2_control hardware interfaces). It is
 * independent of ROS.
 *
 * Joints, in this order:
 *  - One spin joint per wheel (`WheelSpin`), in the vehicle wheel order, so
 *    the index of a wheel is also the index of its joint. Named after the
 *    wheel `joint_name`.
 *  - Ackermann vehicles only (see loadConfigAckermann()): the virtual
 *    traction joint of the drivetrain (`CentralTraction`), if any, then the
 *    steering joints: either one per front wheel (`WheelSteering`), or the
 *    central one (`CentralSteering`, the equivalent steering angle of a
 *    bicycle model) plus the front wheel steering joints as state-only.
 *
 * Joint conventions: positive joint velocity, position and effort of
 * traction joints make the vehicle move forward. Positive steering angles
 * turn left. A non-finite (NaN) command means no command: a wheel rolls
 * freely, and a steering joint keeps its angle.
 *
 * Commands and states are exchanged under a mutex, so they can be accessed
 * from any thread.
 *
 * XML parameters (children of the `<controller>` node):
 *  - `command_interface`: `velocity` (default) or `effort`, for the traction
 *    joints.
 *  - `KP`, `KI`, `KD`, `max_torque`: inner velocity PID of each traction
 *    joint, in the same units than the `twist_pid` controller (error in m/s
 *    at the wheel rim, output in Nm).
 *  - `controllers_yaml`: file with parameters for the controller manager and
 *    the controllers (used by the ROS node).
 *  - `robot_description`: `generated` (default): the ROS node generates a
 *    URDF with the vehicle joints; or `topic`: it is read from the
 *    `robot_description` topic (e.g. published by robot_state_publisher).
 *  - Ackermann vehicles, see loadConfigAckermann().
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

	enum class JointKind : uint8_t
	{
		WheelSpin = 0,	//!< Spin of the wheel `Joint::wheel`
		WheelSteering,	//!< Steering (yaw) angle of the wheel `Joint::wheel`
		/// Equivalent steering angle of the front axle (bicycle model)
		CentralSteering,
		/// Equivalent traction wheel of the drivetrain (bicycle model): its
		/// speed times its radius is the vehicle speed, see
		/// DynamicsAckermannDrivetrain::driveshaftSpeed()
		CentralTraction
	};

	/** The command interface of a joint */
	enum class JointCommand : uint8_t
	{
		None = 0,  //!< State only
		Velocity,  //!< [rad/s]
		Effort,	 //!< [Nm]
		Position  //!< [rad]
	};

	struct Joint
	{
		std::string name;
		JointKind kind = JointKind::WheelSpin;
		size_t wheel = 0;  //!< For WheelSpin and WheelSteering joints
		JointCommand command = JointCommand::None;
		double positionMin = 0;	 //!< Steering joints: angle range [rad] (both 0: none)
		double positionMax = 0;
		double velocityLimit = 0;  //!< Steering joints: max |rate| [rad/s] (0: none)

		/** Traction joints (velocity or effort commands) also report their
		 * effort */
		bool hasEffortState() const
		{
			return command == JointCommand::Velocity || command == JointCommand::Effort;
		}
	};

	struct JointState
	{
		double position = 0;  //!< [rad] (continuous, unwrapped for traction joints)
		double velocity = 0;  //!< [rad/s]
		double effort = 0;	//!< [Nm] (last applied, traction joints only)
	};

	/** Loads XML parameters, with one commanded spin joint per wheel.
	 * Relative paths are resolved with the world base directory. */
	void loadConfig(const rapidxml::xml_node<char>& node, const VehicleBase& veh);

	/** The joints of an Ackermann vehicle, see loadConfigAckermann() */
	struct AckermannLayout
	{
		size_t fl = 0;	//!< Index of the front-left wheel
		size_t fr = 0;	//!< Index of the front-right wheel
		/// A traction joint for the drivetrain, instead of one per wheel
		bool centralTraction = false;
		double maxSteer = 0;  //!< Max equivalent steering angle [rad]
		std::pair<double, double> flSteerRange;	 //!< Angle range of each front wheel [rad]
		std::pair<double, double> frSteerRange;
		double maxSteerRate = 0;  //!< [rad/s] (0: instantaneous)
	};

	/** Like loadConfig(), for Ackermann vehicles: their joints are the wheel
	 * spins (commanded unless `centralTraction`), the drivetrain traction
	 * joint (if `centralTraction`), and the steering joints.
	 *
	 * Additional XML parameters:
	 *  - `steering_joints`: `per_wheel` (one position-commanded joint per
	 *    front wheel, e.g. for ackermann_steering_controller) or `central`
	 *    (one joint, the equivalent steering angle, applied with the
	 *    Ackermann geometry; e.g. for bicycle_steering_controller). The
	 *    default is `central` if `centralTraction`, else `per_wheel`.
	 *  - `fl_steer_joint_name`, `fr_steer_joint_name` (default:
	 *    `fl_steer_joint`, `fr_steer_joint`), `steer_joint_name` (default:
	 *    `steer_joint`), `traction_joint_name` (default: `traction_joint`).
	 */
	void loadConfigAckermann(
		const rapidxml::xml_node<char>& node, const VehicleBase& veh,
		const AckermannLayout& layout);

	CommandMode commandMode() const { return commandMode_; }
	const std::string& controllersYaml() const { return controllersYaml_; }
	bool robotDescriptionFromTopic() const { return robotDescriptionFromTopic_; }

	/** The joints, set by the load methods (constant afterwards) */
	const std::vector<Joint>& joints() const { return joints_; }

	/** Index of the first joint of a kind (and wheel, for wheel joints) */
	std::optional<size_t> findJoint(JointKind kind, size_t wheel = 0) const;

	/** Sets the commands (velocities in rad/s, efforts in Nm, positions in
	 * rad), one per joint, in joints() order. Commands of state-only joints
	 * are ignored. */
	void setCommands(const std::vector<double>& commands);

	/** Sets the command of one joint, leaving the others unchanged. */
	void setCommand(size_t jointIndex, double command);

	/** Current commands, in joints() order (NaN: no command). */
	std::vector<double> getCommands() const;

	/** Current joint states, in joints() order. Empty before the first
	 * simulation step. */
	std::vector<JointState> getStates() const;

	/** Called by the vehicle controller at each simulation step, for the
	 * commanded wheel spin joints.
	 * \param[out] motors Models of the wheel motors (forward positive), for
	 * the implicit integration of the wheel spin.
	 * \return The wheel torques, one per wheel, in mvsim sign convention
	 * (positive means backwards). Zero for wheels without commands. */
	std::vector<double> computeWheelTorques(
		const VehicleBase& veh, double dt, std::vector<MotorTorqueModel>& motors);

	/** Called by the vehicle controller at each simulation step, for the
	 * CentralTraction joint.
	 * \param speed Current vehicle speed measured by the drivetrain [m/s].
	 * \param radius Radius of the equivalent traction wheel [m].
	 * \param[out] motor Model of the drive motor (forward positive), with its
	 * damping per unit of `speed`.
	 * \return The drive torque, in mvsim sign convention (positive means
	 * backwards). */
	double computeCentralTractionTorque(
		double speed, double radius, double dt, MotorTorqueModel& motor);

	/** Ackermann vehicles: the commanded steering, either the equivalent
	 * steering angle `steer` (with a CentralSteering joint), or else the
	 * angles of the front wheels `wheelsSteer` (FL, FR). Joints without a
	 * command keep the current angles, `currentSteer` and `currentWheels`. */
	void ackermannSteering(
		double currentSteer, const std::array<double, 2>& currentWheels, double& steer,
		std::optional<std::array<double, 2>>& wheelsSteer) const;

	/** Values of the central (virtual) joints, given by the vehicle to
	 * updateStates() */
	struct CentralJointValues
	{
		double steering = 0;  //!< Equivalent steering angle [rad]
		double tractionVelocity = 0;  //!< Equivalent traction wheel [rad/s]
	};

	/** Called by the vehicle controller after each simulation step, to update
	 * the joint states */
	void updateStates(const VehicleBase& veh, double dt, const CentralJointValues& central);

	double KP = 10, KI = 0, KD = 0;
	double max_torque = 100;  //!< [Nm]

   private:
	CommandMode commandMode_ = CommandMode::Velocity;
	std::string controllersYaml_;
	bool robotDescriptionFromTopic_ = false;
	std::vector<Joint> joints_;

	mutable std::mutex mtx_;
	std::vector<double> commands_;	//!< One per joint
	std::vector<JointState> states_;  //!< One per joint
	std::vector<double> lastTorques_;  //!< One per joint, forward positive
	std::vector<PID_Controller> pids_;	//!< One per joint

	/** Velocity PID or effort command of a traction joint, with its speed
	 * and radius. Returns the torque, forward positive. Must be called with
	 * the mutex locked. */
	double tractionTorque(
		size_t joint, double cmd, double speed, double radius, double dampingScale, double dt,
		MotorTorqueModel& motor);
};

}  // namespace mvsim
