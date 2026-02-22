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

namespace mvsim
{
/** Implementation of 4 wheels Ackermann-driven vehicles.
 * \sa class factory in VehicleBase::factory
 *  \ingroup vehicle_dynamics_module
 */
class DynamicsAckermann : public VehicleBase
{
	DECLARES_REGISTER_VEHICLE_DYNAMICS(DynamicsAckermann)
   public:
	// Wheels: [0]:rear-left, [1]:rear-right, [2]: front-left, [3]: front-right
	enum
	{
		WHEEL_RL = 0,
		WHEEL_RR = 1,
		WHEEL_FL = 2,
		WHEEL_FR = 3
	};

	DynamicsAckermann(World* parent);

	/** The maximum steering angle (rad). Determines min turning radius */
	double getMaxSteeringAngle() const { return max_steer_ang_; }
	void setMaxSteeringAngle(double val) { max_steer_ang_ = val; }
	/** @name Controllers
		@{ */

	struct TControllerInput
	{
		TSimulContext context;
	};
	struct TControllerOutput
	{
		double fl_torque, fr_torque, rl_torque, rr_torque;
		double steer_ang;  //!< Equivalent Ackermann steering angle
		TControllerOutput() : fl_torque(0), fr_torque(0), rl_torque(0), rr_torque(0), steer_ang(0)
		{
		}
	};

	/** Virtual base for controllers of vehicles of type DynamicsAckermann */
	using ControllerBase = ControllerBaseTempl<DynamicsAckermann>;

	class ControllerRawForces : public ControllerBase
	{
	   public:
		ControllerRawForces(DynamicsAckermann& veh);
		static const char* class_name() { return "raw"; }
		//!< Directly set these values to tell the controller the desired
		//! setpoints
		double setpoint_wheel_torque_l, setpoint_wheel_torque_r, setpoint_steer_ang;
		virtual void control_step(
			const DynamicsAckermann::TControllerInput& ci,
			DynamicsAckermann::TControllerOutput& co) override;
		virtual void load_config(const rapidxml::xml_node<char>& node) override;
		virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override;
	};

	/** PID controller that controls the vehicle with front traction & steering
	 * from Twist commands */
	class ControllerTwistFrontSteerPID : public ControllerBase
	{
	   public:
		ControllerTwistFrontSteerPID(DynamicsAckermann& veh);
		static const char* class_name() { return "twist_front_steer_pid"; }
		//!< Directly set these values to tell the controller the desired
		//! setpoints
		double setpoint_lin_speed,
			setpoint_ang_speed;	 //!< desired velocities (m/s) and (rad/s)
		virtual void control_step(
			const DynamicsAckermann::TControllerInput& ci,
			DynamicsAckermann::TControllerOutput& co) override;
		virtual void load_config(const rapidxml::xml_node<char>& node) override;
		virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override;

		double KP, KI, KD;	//!< PID controller parameters
		double max_torque;	//!< Maximum abs. value torque (for clamp) [Nm]
		/// setpoint magnitude below which it is treated as zero [m/s]
		double zero_threshold = 0.001;
		/// velocity magnitude below which vehicle is considered stopped [m/s]
		double stop_threshold = 0.05;

		// See base docs.
		virtual bool setTwistCommand(const mrpt::math::TTwist2D& t) override
		{
			setpoint_lin_speed = t.vx;
			setpoint_ang_speed = t.omega;
			return true;
		}

	   private:
		double dist_fWheels_, r2f_L_;
		PID_Controller PID_[2];	 //<! [0]:fl, [1]: fr

		double joyMaxLinSpeed = 1.0;
		double joyMaxAngSpeed = 0.7;
	};

	/** PID controller that controls the vehicle with front traction & steering
	 * from steer & linear speed commands */
	class ControllerFrontSteerPID : public ControllerBase
	{
	   public:
		ControllerFrontSteerPID(DynamicsAckermann& veh);
		static const char* class_name() { return "front_steer_pid"; }
		//!< Directly set these values to tell the controller the desired
		//! setpoints
		double setpoint_lin_speed, setpoint_steer_ang;	//!< desired velocities
														//!(m/s) and steering
														//! angle (rad)
		virtual void control_step(
			const DynamicsAckermann::TControllerInput& ci,
			DynamicsAckermann::TControllerOutput& co) override;
		virtual void load_config(const rapidxml::xml_node<char>& node) override;
		virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override;

		double KP, KI, KD;	//!< PID controller parameters
		double max_torque;	//!< Maximum abs. value torque (for clamp) [Nm]
	   private:
		ControllerTwistFrontSteerPID twist_control_;
		double r2f_L_;

		double joyMaxLinSpeed = 1.0;
		double joyMaxSteerAng = 0.7;
	};

	/** Ideal twist controller for Ackermann vehicles.
	 *
	 *  Works exactly like the differential ControllerTwistIdeal: the desired
	 *  (vx, omega) twist is imposed directly on the vehicle state every step,
	 *  bypassing the physical wheel torque model.  It is useful for high-level
	 *  planning tests where tyre dynamics are not of interest.
	 *
	 *  Keyboard teleop: w/s = faster/slower, a/d = turn left/right,
	 *  spacebar = stop.
	 */
	class ControllerTwistIdeal : public ControllerBase
	{
	   public:
		ControllerTwistIdeal(DynamicsAckermann& veh);
		static const char* class_name() { return "twist_ideal"; }

		/** Desired twist setpoint (vx [m/s], vy ignored, omega [rad/s]) */
		mrpt::math::TTwist2D setpoint() const
		{
			auto lck = mrpt::lockHelper(setpointMtx_);
			return setpoint_;
		}
		void setSetpoint(const mrpt::math::TTwist2D& sp)
		{
			auto lck = mrpt::lockHelper(setpointMtx_);
			setpoint_ = sp;
		}

		virtual void control_step(
			const DynamicsAckermann::TControllerInput& ci,
			DynamicsAckermann::TControllerOutput& co) override;

		virtual void on_post_step(const TSimulContext& context) override;

		virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override;

		// Accept Twist commands from ROS / mvsim-server
		virtual bool setTwistCommand(const mrpt::math::TTwist2D& t) override
		{
			setSetpoint(t);
			return true;
		}

	   private:
		mutable std::mutex setpointMtx_;
		mrpt::math::TTwist2D setpoint_{0, 0, 0};

		double r2f_L_ = 1.0;  //!< Wheelbase (rear-to-front axle distance) [m]

		double joyMaxLinSpeed = 1.0;  //!< [m/s]
		double joyMaxAngSpeed = 0.7;  //!< [rad/s]
	};

	const ControllerBase::Ptr& getController() const { return controller_; }
	ControllerBase::Ptr& getController() { return controller_; }
	virtual ControllerBaseInterface* getControllerInterface() override { return controller_.get(); }

	/** @} */  // end controllers

	virtual mrpt::math::TTwist2D getVelocityLocalOdoEstimate() const override;

	/** Computes the exact angles of the front wheels required to have an
	 * equivalent central steering angle.
	 * The method takes into account all wheels info & steering limits stored
	 * in the object.
	 */
	void computeFrontWheelAngles(
		const double desired_equiv_steer_ang, double& out_fl_ang, double& out_fr_ang) const;

   protected:
	// See base class docs
	virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char>* xml_node) override;
	// See base class doc
	virtual std::vector<double> invoke_motor_controllers(const TSimulContext& context) override;
	virtual void invoke_motor_controllers_post_step(const TSimulContext& context) override;

   private:
	ControllerBase::Ptr controller_;  //!< The installed controller

	/** The maximum steering angle (rad). Determines min turning radius */
	double max_steer_ang_ = mrpt::DEG2RAD(30);
};
}  // namespace mvsim
