/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mvsim/ClassFactory.h>
#include <mvsim/CsvLogger.h>
#include <mvsim/Wheel.h>
#include <mvsim/basic_types.h>	// fwrd decls.

namespace mvsim
{
/** Virtual base class for all friction models */
class FrictionBase
{
   public:
	using Ptr = std::shared_ptr<FrictionBase>;

	FrictionBase(VehicleBase& my_vehicle);
	virtual ~FrictionBase();

	/** Class factory: Creates a friction object from XML description of type
	 * "<friction>...</friction>".  */
	static FrictionBase::Ptr factory(
		VehicleBase& parent, const rapidxml::xml_node<char>* xml_node);

	struct TFrictionInput
	{
		const TSimulContext& context;
		Wheel& wheel;

		/** Weight on this wheel from the car chassis (Newtons), excluding the
		 * weight of the wheel itself.
		 */
		double weight = 0;

		/** The force applied by the motor to the wheel (Nm). Negative means
		 * backwards, which makes the vehicle go forwards. */
		double motorTorque = 0;

		/** Instantaneous velocity vector (in local coordinates) of the wheel
		 *  center of gravity (cog) point. */
		mrpt::math::TVector2D wheelCogLocalVel{0, 0};

		TFrictionInput(const TSimulContext& _context, Wheel& _wheel)
			: context(_context), wheel(_wheel)
		{
		}
	};

	/** Evaluates the net force on this wheel (in vehicle local coordinates).
	 * Refer to the manual for the theorical model. */
	virtual mrpt::math::TVector2D evaluate_friction(
		const FrictionBase::TFrictionInput& input) const = 0;

	void setLogger(const std::weak_ptr<CSVLogger>& logger);

   protected:
	World* world_;
	VehicleBase& myVehicle_;

	std::weak_ptr<CSVLogger> logger_;
};

using FrictionBasePtr = std::shared_ptr<FrictionBase>;

// Class factory:
typedef ClassFactory<
	FrictionBase, VehicleBase&, const rapidxml::xml_node<char>*>
	TClassFactory_friction;
extern TClassFactory_friction classFactory_friction;

#define DECLARES_REGISTER_FRICTION(CLASS_NAME)  \
	DECLARES_REGISTER_CLASS2(                   \
		CLASS_NAME, FrictionBase, VehicleBase&, \
		const rapidxml::xml_node<char>*)

#define REGISTER_FRICTION(TEXTUAL_NAME, CLASS_NAME)                  \
	REGISTER_CLASS2(                                                 \
		TClassFactory_friction, classFactory_friction, TEXTUAL_NAME, \
		CLASS_NAME)
}  // namespace mvsim
