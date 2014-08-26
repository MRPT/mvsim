/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/VehicleBase.h>
#include <mv2dsim/ControllerBase.h>
#include <mv2dsim/PID_Controller.h>

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/TColor.h>

namespace mv2dsim
{
	/** Implementation of 4 wheels Ackermann-driven vehicles.
	  * \sa class factory in VehicleBase::factory
	  */
	class DynamicsAckermann : public VehicleBase
	{
		DECLARES_REGISTER_VEHICLE_DYNAMICS(DynamicsAckermann)
	public:
		DynamicsAckermann(World *parent);

		/** Must create a new object in the scene and/or update it according to the current state */
		virtual void gui_update( mrpt::opengl::COpenGLScene &scene);

		/** Create bodies, fixtures, etc. for the dynamical simulation */
		virtual void create_multibody_system(b2World* world);

		// See docs in base class:
		virtual float getMaxVehicleRadius() const { return m_max_radius; }
		// See docs in base class:
		virtual double getChassisMass() const { return m_chassis_mass; }

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
			ControllerRawForces(DynamicsAckermann &veh) : ControllerBase(veh),setpoint_wheel_torque_l(0), setpoint_wheel_torque_r(0) {}
			static const char* class_name() { return "raw"; }
			//!< Directly set these values to tell the controller the desired setpoints
			double setpoint_wheel_torque_l, setpoint_wheel_torque_r, setpoint_steer_ang; 
			// See base class docs
			virtual void control_step(const DynamicsAckermann::TControllerInput &ci, DynamicsAckermann::TControllerOutput &co);
			// See base class docs
			virtual void load_config(const rapidxml::xml_node<char>&node );
		};

		const ControllerBasePtr & getController() const {return m_controller;}
		ControllerBasePtr & getController() {return m_controller;}

		/** @} */  // end controllers

	protected:
		// See base class docs
		virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node);
		// See base class docs
		virtual void invoke_motor_controllers(const TSimulContext &context, std::vector<double> &out_force_per_wheel);

	private:
		mrpt::opengl::CSetOfObjectsPtr m_gl_chassis;
		mrpt::opengl::CSetOfObjectsPtr m_gl_wheels[4]; //!< [0]:rear-left, [1]:rear-right, [2]: front-left, [3]: front-right
		ControllerBasePtr  m_controller; //!< The installed controller

		// Chassis info:
		double m_chassis_mass;
		mrpt::math::TPolygon2D m_chassis_poly;
		double m_max_radius; //!< Automatically computed from m_chassis_poly upon each change via updateMaxRadiusFromPoly()
		double m_chassis_z_min,m_chassis_z_max;
		mrpt::utils::TColor   m_chassis_color;

		double m_max_steer_ang; //!< The maximum steering angle (rad). Determines min turning radius

		void updateMaxRadiusFromPoly();

		Wheel m_wheels_info[4]; //!< [0]:rear-left, [1]:rear-right, [2]: front-left, [3]: front-right

		virtual size_t getNumWheels() const { return 4; }
		virtual const Wheel & getWheelInfo(const size_t idx) const { return m_wheels_info[idx]; }
		virtual Wheel & getWheelInfo(const size_t idx) { return m_wheels_info[idx]; }

		b2Fixture* m_fixture_chassis;
		b2Fixture* m_fixture_wheels[4]; //!< Indices as in m_wheels_info

	};

}
