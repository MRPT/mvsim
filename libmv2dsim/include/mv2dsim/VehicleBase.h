/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/basic_types.h>
#include <mv2dsim/VisualObject.h>
#include <mv2dsim/FrictionModels/FrictionBase.h>

#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>
#include <Box2D/Dynamics/b2Fixture.h>

#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

namespace mv2dsim
{
	/** Virtual base class for each vehicle "actor" in the simulation.
	  * Derived classes implements different dynamical models (Differential, Ackermann,...)
	  */
	class VehicleBase : public VisualObject
	{
	public:
		/** Class factory: Creates a vehicle from XML description of type "<vehicle>...</vehicle>".  */
		static VehicleBase* factory(World* parent, const rapidxml::xml_node<char> *xml_node);
		/// \overload
		static VehicleBase* factory(World* parent, const std::string &xml_text);

		/** Register a new class of vehicles from XML description of type "<vehicle:class name='name'>...</vehicle:class>".  */
		static void register_vehicle_class(const rapidxml::xml_node<char> *xml_node);

		/** Loads vehicle params from input XML node of type "<vehicle>...</vehicle>".
		  * See derived classes & documentation for a list of accepted params.
		  */
		void load_params_from_xml(const rapidxml::xml_node<char> *xml_node);
		/// \overload
		void load_params_from_xml(const std::string &xml_text);

		// ------- Interface with "World" ------
		/** Process right before the integration of dynamic equations for each timestep: set action forces from motors, update friction models, etc. */
		void simul_pre_timestep(const TSimulContext &context);

		/** Override to do any required process right after the integration of dynamic equations for each timestep */
		virtual void simul_post_timestep(const TSimulContext &context) {
			/* Default: do nothing. */
		}

		/** Gets the body dynamical state into q, dot{q} */
		void simul_post_timestep_common(const TSimulContext &context);

		/** Create bodies, fixtures, etc. for the dynamical simulation */
		virtual void create_multibody_system(b2World* world) = 0;


		b2Body * getBox2DChassisBody() { return m_b2d_vehicle_body; }

		/** Common info for 2D wheels, for usage in derived classes.
		  * Wheels are modeled as a mass with a rectangular shape.
		  */
		struct TInfoPerWheel
		{
			double x,y,yaw; //!< Location of the wheel wrt the chassis ref point [m,rad] (in local coords)
			double diameter,width; //!< Length(diameter) and width of the wheel rectangle [m]
               double mass; //!< [kg]
			double color_r,color_g,color_b,color_a; //!< Color for OpenGL rendering (in range [0,1])

			TInfoPerWheel();
			void getAs3DObject(mrpt::opengl::CSetOfObjects &obj);
			void loadFromXML(const rapidxml::xml_node<char> *xml_node);
		};

		virtual size_t getNumWheels() const = 0;
		virtual const VehicleBase::TInfoPerWheel & getWheelInfo(const size_t idx) const = 0;

	protected:
		// Protected ctor for class factory
		VehicleBase(World *parent);

		/** Parse node <dynamics>: The derived-class part of load_params_from_xml(), also called in factory() */
		virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node) = 0;

		virtual void apply_motor_forces(const TSimulContext &context) = 0;

		/** Derived classes must store here the body of the vehicle main body (chassis).
		  * This is used by \a simul_post_timestep() to extract the vehicle dynamical coords (q,\dot{q}) after each simulation step.
		  */
		b2Body *m_b2d_vehicle_body;

		stlplus::smart_ptr<FrictionBase> m_friction; //!< Instance of friction model for the vehicle-to-ground interaction.

		vec3 m_q;   //!< Last time-step pose (of the ref. point, in global coords)
		vec3 m_dq;  //!< Last time-step velocity (of the ref. point, in global coords)


	};
}
