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

namespace mv2dsim
{
	/** Implementation of differential-driven vehicles.
	  * \sa class factory in VehicleBase::factory
	  */
	class DynamicsDifferential : public VehicleBase
	{
	public:
		DynamicsDifferential(World *parent);

		/** Must create a new object in the scene and/or update it according to the current state */
		virtual void gui_update( mrpt::opengl::COpenGLScene &scene);

		/** Create bodies, fixtures, etc. for the dynamical simulation */
		virtual void create_multibody_system(b2World* world);

		// See docs in base class:
		virtual float getMaxVehicleRadius() const { return m_max_radius; }

	protected:
		// See base class docs
		virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node);
		// See base class docs
		virtual void apply_motor_forces(const TSimulContext &context);

	private:
		mrpt::opengl::CSetOfObjectsPtr m_gl_chassis;
		mrpt::opengl::CSetOfObjectsPtr m_gl_wheels[2]; //!< [0]:left, [1]:right

		// Chassis info:
		double m_chassis_mass;
		mrpt::math::TPolygon2D m_chassis_poly;
		double m_max_radius; //!< Automatically computed from m_chassis_poly upon each change via updateMaxRadiusFromPoly()
		double m_chassis_z_min,m_chassis_z_max;

		void updateMaxRadiusFromPoly();

		VehicleBase::TInfoPerWheel m_wheels_info[2]; //!< [0]:left, [1]:right wheel info

		virtual size_t getNumWheels() const { return 2; }
		virtual const VehicleBase::TInfoPerWheel & getWheelInfo(const size_t idx) const { return m_wheels_info[idx]; }

		b2Fixture* m_fixture_chassis;
		b2Fixture* m_fixture_wheels[2]; //!< [0]:left, [1]:right

	};

}
