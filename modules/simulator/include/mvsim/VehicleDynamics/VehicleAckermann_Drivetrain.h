/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
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
	double getMaxSteeringAngle() const { return m_max_steer_ang; }
	void setMaxSteeringAngle(double val) { m_max_steer_ang = val; }
	/** @name Controllers
		@{ */

	struct TControllerInput
	{
		TSimulContext context;
	};
	struct TControllerOutput
	{
		double drive_torque;
		double steer_ang;  //!< Equivalent ackerman steering angle
		TControllerOutput() : drive_torque(0), steer_ang(0) {}
	};

	/** Virtual base for controllers of vehicles of type DynamicsAckermannLSDiff
	 */
	typedef ControllerBaseTempl<DynamicsAckermannDrivetrain> ControllerBase;
	typedef std::shared_ptr<ControllerBase> ControllerBasePtr;

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
		virtual void teleop_interface(
			const TeleopInput& in, TeleopOutput& out) override;
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
		virtual void teleop_interface(
			const TeleopInput& in, TeleopOutput& out) override;

		double KP, KI, KD;	//!< PID controller parameters
		double max_torque;	//!< Maximum abs. value torque (for clamp) [Nm]

		// See base docs.
		virtual bool setTwistCommand(const double vx, const double wz) override
		{
			setpoint_lin_speed = vx;
			setpoint_ang_speed = wz;
			return true;
		}

	   private:
		double m_dist_fWheels, m_r2f_L;
		PID_Controller m_PID;
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
		virtual void teleop_interface(
			const TeleopInput& in, TeleopOutput& out) override;

		double KP, KI, KD;	//!< PID controller parameters
		double max_torque;	//!< Maximum abs. value torque (for clamp) [Nm]
	   private:
		ControllerTwistFrontSteerPID m_twist_control;
		double m_r2f_L;
	};

	const ControllerBasePtr& getController() const { return m_controller; }
	ControllerBasePtr& getController() { return m_controller; }
	virtual ControllerBaseInterface* getControllerInterface() override
	{
		return m_controller.get();
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

	/** Computes differential split for Torsen-like limited slip differentials.
	 */
	void computeDiffTorqueSplit(
		const double w1, const double w2, const double diffBias,
		const double defaultSplitRatio, double& t1, double& t2);

   protected:
	// See base class docs
	virtual void dynamics_load_params_from_xml(
		const rapidxml::xml_node<char>* xml_node) override;
	// See base class doc
	virtual void invoke_motor_controllers(
		const TSimulContext& context,
		std::vector<double>& out_force_per_wheel) override;

   private:
	ControllerBasePtr m_controller;	 //!< The installed controller

	double m_max_steer_ang;	 //!< The maximum steering angle (rad). Determines
							 //! min turning radius

	DifferentialType m_diff_type;

	double m_FrontRearSplit;
	double m_FrontLRSplit;
	double m_RearLRSplit;

	double m_FrontRearBias;
	double m_FrontLRBias;
	double m_RearLRBias;
};
}  // namespace mvsim
