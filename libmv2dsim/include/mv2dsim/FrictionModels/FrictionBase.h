/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#include <mv2dsim/basic_types.h>  // fwrd decls.
#include <mv2dsim/ClassFactory.h>
#include <mv2dsim/Wheel.h>
#include <mrpt/otherlibs/stlplus/smart_ptr.hpp>

namespace mv2dsim
{

	/** Virtual base class for all friction models */
	class FrictionBase
	{
	public:
		FrictionBase(VehicleBase & my_vehicle);
		virtual ~FrictionBase();

		/** Class factory: Creates a friction object from XML description of type "<friction>...</friction>".  */
		static FrictionBase* factory(VehicleBase & parent, const rapidxml::xml_node<char> *xml_node);

		struct TFrictionInput
		{
			const TSimulContext &context;
			const Wheel & wheel;
			double        weight;      //!< Weight on this wheel from the car chassis (Newtons), excluding the weight of the wheel itself.
			double        motor_force; //!< The force applied by the motor, as measured in the wheel-ground contact point (Newtons). Negative means backwards, which makes the vehicle go forwards.
			mrpt::math::TPoint2D  wheel_speed; //!< Instantaneous velocity vector (in local coords) of the wheel center point.

			TFrictionInput(const TSimulContext &_context, const Wheel & _wheel) : context(_context), wheel(_wheel),weight(.0), motor_force(.0),wheel_speed(0,0) {}
		};

		/** Evaluates the net force on this wheel (in local coordinates). Refer to the manual for the theorical model. */
		virtual void evaluate_friction(const FrictionBase::TFrictionInput &input, mrpt::math::TPoint2D &out_result_force_local) const = 0;

	protected:
		World* m_world;
		VehicleBase & m_my_vehicle;
	};

	typedef stlplus::smart_ptr<FrictionBase> FrictionBasePtr;

	// Class factory:
	typedef ClassFactory<FrictionBase,VehicleBase&,const rapidxml::xml_node<char>*> TClassFactory_friction;
	extern TClassFactory_friction classFactory_friction;


	#define DECLARES_REGISTER_FRICTION(CLASS_NAME) \
		DECLARES_REGISTER_CLASS2(CLASS_NAME,FrictionBase,VehicleBase&,const rapidxml::xml_node<char> *)

	#define REGISTER_FRICTION(TEXTUAL_NAME,CLASS_NAME) \
		REGISTER_CLASS2(TClassFactory_friction,classFactory_friction,TEXTUAL_NAME,CLASS_NAME) 
	

}
