/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/VehicleBase.h>

#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/TColor.h>

namespace mv2dsim
{
	/** Implementation of differential-driven vehicles.
	  * \sa class factory in VehicleBase::factory
	  */
	class DynamicsDifferential : public VehicleBase
	{
		DECLARES_REGISTER_VEHICLE_DYNAMICS(DynamicsDifferential)
	public:
		DynamicsDifferential(World *parent);

		/** Must create a new object in the scene and/or update it according to the current state */
		virtual void gui_update( mrpt::opengl::COpenGLScene &scene);

		/** Create bodies, fixtures, etc. for the dynamical simulation */
		virtual void create_multibody_system(b2World* world);

		// See docs in base class:
		virtual float getMaxVehicleRadius() const { return m_max_radius; }

		/** @name Controllers 
		    @{ */

		struct TControllerInput
		{
			TSimulContext context;
		};
		struct TControllerOutput
		{
			double wheel_force_l,wheel_force_r;

			TControllerOutput() : wheel_force_l(0),wheel_force_r(0) {}
		};

		/** Virtual base for controllers of vehicles of type DynamicsDifferential */
		class ControllerBase
		{
		public:
			ControllerBase(DynamicsDifferential &veh) : m_veh(veh) {}
			virtual ~ControllerBase() {}

			/** The core of the controller: will be called at each timestep before the numeric integration of dynamical eqs */
			virtual void control_step(const DynamicsDifferential::TControllerInput &ci, DynamicsDifferential::TControllerOutput &co) =0;

			/** Override to load class-specific options from the <controller> node */
			virtual void load_config(const rapidxml::xml_node<char>&node ) {  /*default: do nothing*/ }

		protected:
			DynamicsDifferential &m_veh;
		};
		typedef stlplus::smart_ptr<ControllerBase> ControllerBasePtr;

		class ControllerRawForces : public ControllerBase
		{
		public:
			ControllerRawForces(DynamicsDifferential &veh) : ControllerBase(veh),setpoint_wheel_force_l(0), setpoint_wheel_force_r(0) {}
			static const char* class_name() { return "raw"; }
			//!< Directly set these values to tell the controller the desired setpoints
			double setpoint_wheel_force_l, setpoint_wheel_force_r; 
			// See base class docs
			virtual void control_step(const DynamicsDifferential::TControllerInput &ci, DynamicsDifferential::TControllerOutput &co);
		};


		const ControllerBasePtr & getController() const {return m_controller;}
		ControllerBasePtr & getController() {return m_controller;}

		/** @} */  // end controllers

	protected:
		// See base class docs
		virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node);
		// See base class docs
		virtual void apply_motor_forces(const TSimulContext &context);

	private:
		mrpt::opengl::CSetOfObjectsPtr m_gl_chassis;
		mrpt::opengl::CSetOfObjectsPtr m_gl_wheels[2]; //!< [0]:left, [1]:right
		ControllerBasePtr  m_controller; //!< The installed controller

		// Chassis info:
		double m_chassis_mass;
		mrpt::math::TPolygon2D m_chassis_poly;
		double m_max_radius; //!< Automatically computed from m_chassis_poly upon each change via updateMaxRadiusFromPoly()
		double m_chassis_z_min,m_chassis_z_max;
		mrpt::utils::TColor   m_chassis_color;

		void updateMaxRadiusFromPoly();

		VehicleBase::TInfoPerWheel m_wheels_info[2]; //!< [0]:left, [1]:right wheel info

		virtual size_t getNumWheels() const { return 2; }
		virtual const VehicleBase::TInfoPerWheel & getWheelInfo(const size_t idx) const { return m_wheels_info[idx]; }

		b2Fixture* m_fixture_chassis;
		b2Fixture* m_fixture_wheels[2]; //!< [0]:left, [1]:right

	};

}
