/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
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

	/** Computes differential split for Torsen-like limited slip differentials.
	 */
	void computeDiffTorqueSplit(
		const double w1, const double w2, const double diffBias, const double defaultSplitRatio,
		double& t1, double& t2);

   protected:
	// See base class docs
	virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char>* xml_node) override;
	// See base class doc
	virtual std::vector<double> invoke_motor_controllers(const TSimulContext& context) override;

   private:
	ControllerBase::Ptr controller_;  //!< The installed controller

	double max_steer_ang_;	//!< The maximum steering angle (rad). Determines
							//! min turning radius

	DifferentialType diff_type_;

	double FrontRearSplit_;
	double FrontLRSplit_;
	double RearLRSplit_;

	double FrontRearBias_;
	double FrontLRBias_;
	double RearLRBias_;
};
}  // namespace mvsim
