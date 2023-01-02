/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
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
		double steer_ang;  //!< Equivalent ackerman steering angle
		TControllerOutput()
			: fl_torque(0),
			  fr_torque(0),
			  rl_torque(0),
			  rr_torque(0),
			  steer_ang(0)
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
		double setpoint_wheel_torque_l, setpoint_wheel_torque_r,
			setpoint_steer_ang;
		virtual void control_step(
			const DynamicsAckermann::TControllerInput& ci,
			DynamicsAckermann::TControllerOutput& co) override;
		virtual void load_config(const rapidxml::xml_node<char>& node) override;
		virtual void teleop_interface(
			const TeleopInput& in, TeleopOutput& out) override;
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
		virtual void teleop_interface(
			const TeleopInput& in, TeleopOutput& out) override;

		double KP, KI, KD;	//!< PID controller parameters
		double max_torque;	//!< Maximum abs. value torque (for clamp) [Nm]

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
		virtual void teleop_interface(
			const TeleopInput& in, TeleopOutput& out) override;

		double KP, KI, KD;	//!< PID controller parameters
		double max_torque;	//!< Maximum abs. value torque (for clamp) [Nm]
	   private:
		ControllerTwistFrontSteerPID twist_control_;
		double r2f_L_;
	};

	const ControllerBase::Ptr& getController() const { return controller_; }
	ControllerBase::Ptr& getController() { return controller_; }
	virtual ControllerBaseInterface* getControllerInterface() override
	{
		return controller_.get();
	}

	/** @} */  // end controllers

	virtual mrpt::math::TTwist2D getVelocityLocalOdoEstimate() const override;

	/** Computes the exact angles of the front wheels required to have an
	 * equivalent central steering angle.
	 * The method takes into account all wheels info & steering limits stored
	 * in the object.
	 */
	void computeFrontWheelAngles(
		const double desired_equiv_steer_ang, double& out_fl_ang,
		double& out_fr_ang) const;

   protected:
	// See base class docs
	virtual void dynamics_load_params_from_xml(
		const rapidxml::xml_node<char>* xml_node) override;
	// See base class doc
	virtual void invoke_motor_controllers(
		const TSimulContext& context,
		std::vector<double>& out_force_per_wheel) override;

   private:
	ControllerBase::Ptr controller_;  //!< The installed controller

	/** The maximum steering angle (rad). Determines min turning radius */
	double max_steer_ang_ = mrpt::DEG2RAD(30);
};
}  // namespace mvsim
