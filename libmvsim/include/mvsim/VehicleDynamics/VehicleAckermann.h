/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mvsim/VehicleBase.h>
#include <mvsim/PID_Controller.h>

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/TColor.h>

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
		enum {
			WHEEL_RL = 0,
			WHEEL_RR = 1,
			WHEEL_FL = 2,
			WHEEL_FR = 3
		};

		DynamicsAckermann(World *parent);

		/** The maximum steering angle (rad). Determines min turning radius */
		double getMaxSteeringAngle() const {return m_max_steer_ang;}
		void setMaxSteeringAngle(double val) {m_max_steer_ang=val;}

		/** @name Controllers
		    @{ */

		struct TControllerInput
		{
			TSimulContext context;
		};
		struct TControllerOutput
		{
			double fl_torque,fr_torque,rl_torque,rr_torque;
			double steer_ang; //!< Equivalent ackerman steering angle
			TControllerOutput() : fl_torque(0),fr_torque(0),rl_torque(0),rr_torque(0),steer_ang(0) {}
		};

		/** Virtual base for controllers of vehicles of type DynamicsAckermann */
		typedef ControllerBaseTempl<DynamicsAckermann> ControllerBase;
		typedef stlplus::smart_ptr<ControllerBase> ControllerBasePtr;

		class ControllerRawForces : public ControllerBase
		{
		public:
			ControllerRawForces(DynamicsAckermann &veh);
			static const char* class_name() { return "raw"; }
			//!< Directly set these values to tell the controller the desired setpoints
			double setpoint_wheel_torque_l, setpoint_wheel_torque_r, setpoint_steer_ang;
			virtual void control_step(const DynamicsAckermann::TControllerInput &ci, DynamicsAckermann::TControllerOutput &co); // See base class docs
			virtual void load_config(const rapidxml::xml_node<char>&node ); // See base class docs
			virtual void teleop_interface(const TeleopInput &in, TeleopOutput &out);  // See base class docs
		};

		/** PID controller that controls the vehicle with front traction & steering from Twist commands */
		class ControllerTwistFrontSteerPID : public ControllerBase
		{
		public:
			ControllerTwistFrontSteerPID(DynamicsAckermann &veh);
			static const char* class_name() { return "twist_front_steer_pid"; }
			//!< Directly set these values to tell the controller the desired setpoints
			double setpoint_lin_speed, setpoint_ang_speed;  //!< desired velocities (m/s) and (rad/s)
			virtual void control_step(const DynamicsAckermann::TControllerInput &ci, DynamicsAckermann::TControllerOutput &co); // See base class docs
			virtual void load_config(const rapidxml::xml_node<char>&node ); // See base class docs
			virtual void teleop_interface(const TeleopInput &in, TeleopOutput &out);  // See base class docs

			double KP,KI,KD; //!< PID controller parameters
			double I_MAX; //!< I part maximum value (absolute value for clamp)
			double max_torque; //!< Maximum abs. value torque (for clamp) [Nm]
		private:
			double m_dist_fWheels, m_r2f_L;
			PID_Controller m_PID[2]; //<! [0]:fl, [1]: fr
		};

		/** PID controller that controls the vehicle with front traction & steering from steer & linear speed commands */
		class ControllerFrontSteerPID : public ControllerBase
		{
		public:
			ControllerFrontSteerPID(DynamicsAckermann &veh);
			static const char* class_name() { return "front_steer_pid"; }
			//!< Directly set these values to tell the controller the desired setpoints
			double setpoint_lin_speed, setpoint_steer_ang;  //!< desired velocities (m/s) and steering angle (rad)
			virtual void control_step(const DynamicsAckermann::TControllerInput &ci, DynamicsAckermann::TControllerOutput &co); // See base class docs
			virtual void load_config(const rapidxml::xml_node<char>&node ); // See base class docs
			virtual void teleop_interface(const TeleopInput &in, TeleopOutput &out);  // See base class docs

			double KP,KI,KD; //!< PID controller parameters
			double I_MAX; //!< I part maximum value (absolute value for clamp)
			double max_torque; //!< Maximum abs. value torque (for clamp) [Nm]
		private:
			ControllerTwistFrontSteerPID m_twist_control;
			double m_r2f_L;
		};


		const ControllerBasePtr & getController() const {return m_controller;}
		ControllerBasePtr & getController() {return m_controller;}
		virtual ControllerBaseInterface * getControllerInterface() { return m_controller.pointer(); }

		/** @} */  // end controllers

		virtual vec3 getVelocityLocalOdoEstimate() const; // See docs of base class

		/** Computes the exact angles of the front wheels required to have an equivalent central steering angle.
		  * The method takes into account all wheels info & steering limits stored in the object.
		  */
		void computeFrontWheelAngles(const double desired_equiv_steer_ang, double &out_fl_ang,double &out_fr_ang) const;

	protected:
		// See base class docs
		virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node);
		// See base class doc
		virtual void invoke_motor_controllers(const TSimulContext &context, std::vector<double> &out_force_per_wheel);

	private:
		ControllerBasePtr  m_controller; //!< The installed controller

		double m_max_steer_ang; //!< The maximum steering angle (rad). Determines min turning radius



	};

}
