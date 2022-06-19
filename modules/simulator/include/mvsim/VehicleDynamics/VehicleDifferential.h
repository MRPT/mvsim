/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/PID_Controller.h>
#include <mvsim/VehicleBase.h>

namespace mvsim
{
/** Implementation of differential-driven vehicles.
 * \sa class factory in VehicleBase::factory
 */
class DynamicsDifferential : public VehicleBase
{
	DECLARES_REGISTER_VEHICLE_DYNAMICS(DynamicsDifferential)
   public:
	enum
	{
		WHEEL_L = 0,
		WHEEL_R = 1
	};

	DynamicsDifferential(World* parent);

	/** @name Controllers
		@{ */

	struct TControllerInput
	{
		TSimulContext context;
	};
	struct TControllerOutput
	{
		double wheel_torque_l, wheel_torque_r;
		TControllerOutput() : wheel_torque_l(0), wheel_torque_r(0) {}
	};

	/** Virtual base for controllers of vehicles of type DynamicsDifferential */
	typedef ControllerBaseTempl<DynamicsDifferential> ControllerBase;
	typedef std::shared_ptr<ControllerBase> ControllerBasePtr;

	class ControllerRawForces : public ControllerBase
	{
	   public:
		ControllerRawForces(DynamicsDifferential& veh)
			: ControllerBase(veh),
			  setpoint_wheel_torque_l(0),
			  setpoint_wheel_torque_r(0)
		{
		}
		static const char* class_name() { return "raw"; }
		//!< Directly set these values to tell the controller the desired
		//! setpoints
		double setpoint_wheel_torque_l, setpoint_wheel_torque_r;
		virtual void control_step(
			const DynamicsDifferential::TControllerInput& ci,
			DynamicsDifferential::TControllerOutput& co) override;
		virtual void teleop_interface(
			const TeleopInput& in, TeleopOutput& out) override;
	};

	/** PID controller that controls the vehicle twist: linear & angular
	 * velocities */
	class ControllerTwistPID : public ControllerBase
	{
	   public:
		ControllerTwistPID(DynamicsDifferential& veh);
		static const char* class_name() { return "twist_pid"; }
		//!< Directly set these values to tell the controller the desired
		//! setpoints
		double setpoint_lin_speed,
			setpoint_ang_speed;	 //!< desired velocities (m/s) and (rad/s)
		virtual void control_step(
			const DynamicsDifferential::TControllerInput& ci,
			DynamicsDifferential::TControllerOutput& co) override;
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
		double m_distWheels;
		PID_Controller m_PID[2];
	};

	const ControllerBasePtr& getController() const { return m_controller; }
	ControllerBasePtr& getController() { return m_controller; }
	virtual ControllerBaseInterface* getControllerInterface() override
	{
		return m_controller.get();
	}

	/** @} */  // end controllers

	virtual mrpt::math::TTwist2D getVelocityLocalOdoEstimate() const override;

   protected:
	// See base class docs
	virtual void dynamics_load_params_from_xml(
		const rapidxml::xml_node<char>* xml_node) override;
	// See base class docs
	virtual void invoke_motor_controllers(
		const TSimulContext& context,
		std::vector<double>& out_force_per_wheel) override;

   private:
	ControllerBasePtr m_controller;	 //!< The installed controller
};
}  // namespace mvsim
