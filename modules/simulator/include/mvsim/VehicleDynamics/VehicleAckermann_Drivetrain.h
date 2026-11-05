/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/img/TColor.h>
#include <mvsim/PID_Controller.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/WheelJointsInterface.h>

#include <algorithm>
#include <array>
#include <optional>
#include <utility>

namespace mvsim
{
/** Implementation of 4 wheels Ackermann-driven vehicles with drivetrain
 * As motor input of drivetrain acts controller torque.
 * Differential model is based on observations of Torsen-like differentials
 * work.
 * http://www.flashoffroad.com/features/Torsen/Torsen_white_paper.pdf
 *
 * \sa class factory in VehicleBase::factory
 *
 * \ingroup vehicle_dynamics_module
 */
class DynamicsAckermannDrivetrain : public VehicleBase
{
	DECLARES_REGISTER_VEHICLE_DYNAMICS(DynamicsAckermannDrivetrain)
   public:
	// Wheels: [0]:rear-left, [1]:rear-right, [2]: front-left, [3]: front-right
	enum
	{
		WHEEL_RL = 0,
		WHEEL_RR = 1,
		WHEEL_FL = 2,
		WHEEL_FR = 3
	};

	enum DifferentialType
	{
		DIFF_OPEN_FRONT = 0,
		DIFF_OPEN_REAR = 1,
		DIFF_OPEN_4WD = 2,

		DIFF_TORSEN_FRONT = 3,
		DIFF_TORSEN_REAR = 4,
		DIFF_TORSEN_4WD = 5,

		DIFF_MAX
	};

	DynamicsAckermannDrivetrain(World* parent);

	/** The maximum steering angle (rad). Determines min turning radius */
	double getMaxSteeringAngle() const { return max_steer_ang_; }
	void setMaxSteeringAngle(double val) { max_steer_ang_ = val; }
	/** Range [min,max] of the steering angle of a front wheel (WHEEL_FL or
	 * WHEEL_FR) [rad]: at the maximum (equivalent) steering angle, a wheel
	 * reaches the inner angle when turning toward its side, and the outer one
	 * when turning away from it. */
	std::pair<double, double> wheelSteeringRange(size_t wheel) const
	{
		// Turning left: the left wheel is the inner one
		double inner = 0;
		double outer = 0;
		computeFrontWheelAngles(max_steer_ang_, inner, outer);
		return wheel == WHEEL_FL ? std::pair{-outer, inner} : std::pair{-inner, outer};
	}
	/** Maximum steering rate (rad/s), 0 means instantaneous steering */
	double getMaxSteeringRate() const { return max_steer_rate_; }
	/** @name Controllers
		@{ */

	struct TControllerInput
	{
		TSimulContext context;
	};
	struct TControllerOutput
	{
		double drive_torque;
		double steer_ang;  //!< Equivalent Ackermann steering angle
		/// If set, the steering angles of the front wheels [FL, FR] (rad),
		/// used instead of `steer_ang` (e.g. from ros2_control joints)
		std::optional<std::array<double, 2>> front_wheels_steer_ang;
		/// Optional closed-loop model of the drive motor (forward positive),
		/// with its damping per unit of driveshaftSpeed() [N.m.s/m]
		MotorTorqueModel drive_motor;
		TControllerOutput() : drive_torque(0), steer_ang(0) {}
	};

	/** Virtual base for controllers of vehicles of type DynamicsAckermannLSDiff
	 */
	using ControllerBase = ControllerBaseTempl<DynamicsAckermannDrivetrain>;

	class ControllerRawForces : public ControllerBase
	{
	   public:
		ControllerRawForces(DynamicsAckermannDrivetrain& veh);
		static const char* class_name() { return "raw"; }
		//!< Directly set these values to tell the controller the desired
		//! setpoints
		double setpoint_wheel_torque, setpoint_steer_ang;
		virtual void control_step(
			const DynamicsAckermannDrivetrain::TControllerInput& ci,
			DynamicsAckermannDrivetrain::TControllerOutput& co) override;
		virtual void load_config(const rapidxml::xml_node<char>& node) override;
		virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override;
	};

	/** PID controller that controls the vehicle with front traction & steering
	 * from Twist commands */
	class ControllerTwistFrontSteerPID : public ControllerBase
	{
	   public:
		ControllerTwistFrontSteerPID(DynamicsAckermannDrivetrain& veh);
		static const char* class_name() { return "twist_front_steer_pid"; }
		//!< Directly set these values to tell the controller the desired
		//! setpoints
		double setpoint_lin_speed,
			setpoint_ang_speed;	 //!< desired velocities (m/s) and (rad/s)
		virtual void control_step(
			const DynamicsAckermannDrivetrain::TControllerInput& ci,
			DynamicsAckermannDrivetrain::TControllerOutput& co) override;
		virtual void load_config(const rapidxml::xml_node<char>& node) override;
		virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override;

		double KP, KI, KD;	//!< PID controller parameters
		double N = 0;  //!< Derivative filter coefficient
		double max_torque;	//!< Maximum abs. value torque (for clamp) [Nm]

		bool enable_antiwindup = false;
		bool enable_feedforward = false;
		double feedforward_gain = 1.0;
		bool enable_reference_filter = false;
		double reference_filter_tau = 0.1;
		int reference_filter_order = 1;

		// See base docs.
		virtual bool setTwistCommand(const mrpt::math::TTwist2D& t) override
		{
			setpoint_lin_speed = t.vx;
			setpoint_ang_speed = t.omega;
			return true;
		}

	   private:
		double dist_fWheels_, r2f_L_;
		PID_Controller PID_;

		double joyMaxLinSpeed = 1.0;
		double joyMaxAngSpeed = 0.7;
	};

	class ControllerFrontSteerPID : public ControllerBase
	{
	   public:
		ControllerFrontSteerPID(DynamicsAckermannDrivetrain& veh);
		static const char* class_name() { return "front_steer_pid"; }
		//!< Directly set these values to tell the controller the desired
		//! setpoints
		double setpoint_lin_speed, setpoint_steer_ang;	//!< desired velocities
														//!(m/s) and steering
														//! angle (rad)
		virtual void control_step(
			const DynamicsAckermannDrivetrain::TControllerInput& ci,
			DynamicsAckermannDrivetrain::TControllerOutput& co) override;
		virtual void load_config(const rapidxml::xml_node<char>& node) override;
		virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override;

		double KP, KI, KD;	//!< PID controller parameters
		double max_torque;	//!< Maximum abs. value torque (for clamp) [Nm]
	   private:
		ControllerTwistFrontSteerPID twist_control_;
		double r2f_L_;
	};

	/** Joint commands from an external controller (e.g. ros2_control): the
	 * traction joint of the drivetrain (its speed is driveshaftSpeed() /
	 * driveshaftRadius()), and steering joints, see
	 * WheelJointsInterface::loadConfigAckermann() */
	class ControllerJointCommands : public ControllerBase
	{
	   public:
		ControllerJointCommands(DynamicsAckermannDrivetrain& veh) : ControllerBase(veh) {}
		static const char* class_name() { return "ros2_control"; }

		void control_step(
			const DynamicsAckermannDrivetrain::TControllerInput& ci,
			DynamicsAckermannDrivetrain::TControllerOutput& co) override;
		void on_post_step(const TSimulContext& context) override;
		void load_config(const rapidxml::xml_node<char>& node) override;

		WheelJointsInterface* wheelJointsInterface() override { return &joints_; }

	   private:
		WheelJointsInterface joints_;
	};

	const ControllerBase::Ptr& getController() const { return controller_; }
	ControllerBase::Ptr& getController() { return controller_; }
	virtual ControllerBaseInterface* getControllerInterface() override { return controller_.get(); }

	/** @} */  // end controllers

	virtual mrpt::math::TTwist2D getVelocityLocalOdoEstimate() const override;

	/** Mean longitudinal speed of the driven axles [m/s], from the wheels own
	 *  spin (as an encoder in the driveshaft would measure it). Optionally
	 *  returns its derivative with respect to each wheel spin speed. */
	double driveshaftSpeed(std::array<double, 4>* dSpeed_dw = nullptr) const;

	/** Mean radius of the driven wheels [m]: driveshaftSpeed() divided by it
	 *  is the spin speed of the equivalent traction wheel. */
	double driveshaftRadius() const;

	/** Computes the exact angles of the front wheels required to have an
	 * equivalent central steering angle.
	 * The method takes into account all wheels info & steering limits stored
	 * in the object.
	 */
	void computeFrontWheelAngles(
		const double desired_equiv_steer_ang, double& out_fl_ang, double& out_fr_ang) const;

	/** Torque transferred by a Torsen differential from its output "a" to "b"
	 *  (forward positive), given their spin speeds: it locks both outputs
	 *  together (stiffness k=inertia/dt, removing their speed difference in one
	 *  step) until the torque ratio reaches the bias ratio. Returns whether the
	 *  bias limit was reached. */
	static bool torsenLockTorque(
		double wa, double wb, double inputTorque, double split, double biasRatio, double k,
		double& lockTorque);

   protected:
	// See base class docs
	virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char>* xml_node) override;
	// See base class doc
	virtual std::vector<double> invoke_motor_controllers(const TSimulContext& context) override;
	virtual void invoke_motor_controllers_post_step(const TSimulContext& context) override;

   private:
	ControllerBase::Ptr controller_;  //!< The installed controller

	/** Weights of the front and rear axles in driveshaftSpeed() */
	void driveshaftAxleWeights(double& wFront, double& wRear) const;

	double max_steer_ang_;	//!< The maximum steering angle (rad). Determines
							//! min turning radius

	/** Maximum steering rate (rad/s) of the equivalent steering angle, 0 means
	 *  instantaneous steering. XML: <max_steer_rate_deg> */
	double max_steer_rate_ = 0;
	double steer_ang_ = 0;	//!< Current equivalent steering angle (rad)

	/** Limits the steering rate (see max_steer_rate_) towards the desired
	 *  equivalent steering angle, and returns the resulting one. */
	double rateLimitedSteering(double desired, double dt)
	{
		// The wheels cannot go beyond their limit, so neither does the state:
		desired = std::clamp(desired, -max_steer_ang_, max_steer_ang_);
		if (max_steer_rate_ > 0)
		{
			const double maxDelta = max_steer_rate_ * dt;
			desired = steer_ang_ + std::clamp(desired - steer_ang_, -maxDelta, maxDelta);
		}
		steer_ang_ = desired;
		return desired;
	}

	/** Like rateLimitedSteering(), for the angle of one front wheel, within
	 *  its wheelSteeringRange(). */
	double rateLimitedWheelSteering(size_t wheel, double desired, double current, double dt) const
	{
		const auto [minAng, maxAng] = wheelSteeringRange(wheel);
		desired = std::clamp(desired, minAng, maxAng);
		if (max_steer_rate_ > 0)
		{
			const double maxDelta = max_steer_rate_ * dt;
			desired = current + std::clamp(desired - current, -maxDelta, maxDelta);
		}
		return desired;
	}

	DifferentialType diff_type_;

	double FrontRearSplit_;
	double FrontLRSplit_;
	double RearLRSplit_;

	double FrontRearBias_;
	double FrontLRBias_;
	double RearLRBias_;
};
}  // namespace mvsim
