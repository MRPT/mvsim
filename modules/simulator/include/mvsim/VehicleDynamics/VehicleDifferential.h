/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/math/TPoint2D.h>
#include <mvsim/PID_Controller.h>
#include <mvsim/VehicleBase.h>

namespace mvsim
{
/** @addtogroup vehicle_dynamics_module  Vehicle kinematic models
 * \ingroup mvsim_simulator_module
 */

/** Implementation of differential-driven vehicles with only two wheels.
 * Simplified model for pure planar scenarios only, do not use with ramps.
 * For that, use Differential or Ackermann models with 3 or 4 wheels.
 *
 * \sa class factory in VehicleBase::factory
 *
 * \ingroup vehicle_dynamics_module
 */
class DynamicsDifferential : public VehicleBase
{
	DECLARES_REGISTER_VEHICLE_DYNAMICS(DynamicsDifferential)
   public:
	enum
	{
		// common to all:
		WHEEL_L = 0,
		WHEEL_R = 1,
		// 3 wheels:
		WHEEL_CASTER_FRONT = 2,
		// 4 wheels:
		WHEEL_LR = 0,
		WHEEL_RR = 1,
		WHEEL_LF = 2,
		WHEEL_RF = 3
	};

	struct ConfigPerWheel
	{
		ConfigPerWheel() = default;
		ConfigPerWheel(const std::string& _name, const mrpt::math::TPoint2D& _pos)
			: name(_name), pos(_pos)
		{
		}

		std::string name;
		mrpt::math::TPoint2D pos;
	};

	DynamicsDifferential(World* parent)
		: DynamicsDifferential(
			  parent, {
						  {"l_wheel", {0.0, 0.5}},
						  {"r_wheel", {0.0, -0.5}},
					  })
	{
	}

	DynamicsDifferential(World* parent, const std::vector<ConfigPerWheel>& cfgPerWheel);

	/** @name Controllers
		@{ */

	struct TControllerInput
	{
		TSimulContext context;
	};

	struct TControllerOutput
	{
		TControllerOutput() = default;

		double wheel_torque_l = 0;
		double wheel_torque_r = 0;
	};

	/** Virtual base for controllers of vehicles of type DynamicsDifferential */
	using ControllerBase = ControllerBaseTempl<DynamicsDifferential>;

	class ControllerRawForces : public ControllerBase
	{
	   public:
		ControllerRawForces(DynamicsDifferential& veh) : ControllerBase(veh) {}

		static const char* class_name() { return "raw"; }

		//!< Directly set these values to tell the controller the desired
		//! setpoints
		double setpoint_wheel_torque_l = 0, setpoint_wheel_torque_r = 0;

		double setpoint_teleop_steps = 5e-2;

		virtual void control_step(
			const DynamicsDifferential::TControllerInput& ci,
			DynamicsDifferential::TControllerOutput& co) override;
		virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override;
	};

	/** PID controller that controls the vehicle twist: linear & angular
	 * velocities */
	class ControllerTwistPID : public ControllerBase
	{
	   public:
		ControllerTwistPID(DynamicsDifferential& veh);
		static const char* class_name() { return "twist_pid"; }

		virtual void control_step(
			const DynamicsDifferential::TControllerInput& ci,
			DynamicsDifferential::TControllerOutput& co) override;

		virtual void load_config(const rapidxml::xml_node<char>& node) override;
		virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override;

		/// PID controller parameters
		double KP = 10, KI = 0, KD = 0;

		/// Maximum abs. value torque (for clamp) [Nm]
		double max_torque = 100;

		// See base docs.
		bool setTwistCommand(const mrpt::math::TTwist2D& t) override
		{
			setpointMtx_.lock();
			setpoint_ = t;
			setpointMtx_.unlock();
			return true;
		}

		/** Returns the current setpoint of the controller */
		mrpt::math::TTwist2D setpoint() const
		{
			setpointMtx_.lock();
			auto t = setpoint_;
			setpointMtx_.unlock();
			return t;
		}

	   private:
		double distWheels_ = 0;
		std::array<PID_Controller, 2> PIDs_;
		mrpt::math::TTwist2D setpoint_{0, 0, 0};  //!< "vx" and "omega" only
		mutable std::mutex setpointMtx_;

		double joyMaxLinSpeed = 1.0;
		double joyMaxAngSpeed = 0.5;
	};

	/** Ideal ("fake") controller, which perfectly and instantaneously
	 *  sets the vehicle twist setpoint as vehicle twist.
	 */
	class ControllerTwistIdeal : public ControllerBase
	{
	   public:
		ControllerTwistIdeal(DynamicsDifferential& veh);
		static const char* class_name() { return "twist_ideal"; }

		void control_step(
			const DynamicsDifferential::TControllerInput& ci,
			DynamicsDifferential::TControllerOutput& co) override;
		void on_post_step(const TSimulContext& context) override;

		virtual void teleop_interface(const TeleopInput& in, TeleopOutput& out) override;

		// See base docs.
		bool setTwistCommand(const mrpt::math::TTwist2D& t) override
		{
			setpointMtx_.lock();
			setpoint_ = t;
			setpointMtx_.unlock();
			return true;
		}

		/** Returns the current setpoint of the controller */
		mrpt::math::TTwist2D setpoint() const
		{
			setpointMtx_.lock();
			auto t = setpoint_;
			setpointMtx_.unlock();
			return t;
		}

	   private:
		double distWheels_ = 0;
		mrpt::math::TTwist2D setpoint_{0, 0, 0};  //!< "vx" and "omega" only
		mutable std::mutex setpointMtx_;

		double joyMaxLinSpeed = 1.0;
		double joyMaxAngSpeed = 0.5;
	};

	const ControllerBase::Ptr& getController() const { return controller_; }
	ControllerBase::Ptr& getController() { return controller_; }
	virtual ControllerBaseInterface* getControllerInterface() override { return controller_.get(); }

	/** @} */  // end controllers

	virtual mrpt::math::TTwist2D getVelocityLocalOdoEstimate() const override;

   protected:
	// See base class docs
	virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char>* xml_node) override;
	// See base class docs
	virtual std::vector<double> invoke_motor_controllers(const TSimulContext& context) override;
	virtual void invoke_motor_controllers_post_step(const TSimulContext& context) override;

	/// Defined at ctor time:
	const std::vector<ConfigPerWheel> configPerWheel_;

   private:
	ControllerBase::Ptr controller_;  //!< The installed controller
};

/**
 * \ingroup vehicle_dynamics_module
 */
class DynamicsDifferential_3_wheels : public DynamicsDifferential
{
	DECLARES_REGISTER_VEHICLE_DYNAMICS(DynamicsDifferential_3_wheels)

   public:
	DynamicsDifferential_3_wheels(World* parent)
		: DynamicsDifferential(
			  parent, {
						  {"l_wheel", {0.0, 0.5}},
						  {"r_wheel", {0.0, -0.5}},
						  {"caster_wheel", {0.5, 0.0}},
					  })
	{
	}
};

/**
 * \ingroup vehicle_dynamics_module
 */
class DynamicsDifferential_4_wheels : public DynamicsDifferential
{
	DECLARES_REGISTER_VEHICLE_DYNAMICS(DynamicsDifferential_4_wheels)

   public:
	DynamicsDifferential_4_wheels(World* parent)
		: DynamicsDifferential(
			  parent, {
						  {"lr_wheel", {0.0, 0.5}},
						  {"rr_wheel", {0.0, -0.5}},
						  {"lf_wheel", {0.5, 0.5}},
						  {"rf_wheel", {0.5, -0.5}},
					  })
	{
	}
};

}  // namespace mvsim
