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

	protected:
		// See base class docs
		virtual void dynamics_load_params_from_xml(const rapidxml::xml_node<char> *xml_node);

	private:
		mrpt::opengl::CSetOfObjectsPtr m_gl_chassis;

	};

}