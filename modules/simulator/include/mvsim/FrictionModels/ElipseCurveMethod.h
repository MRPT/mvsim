/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <box2d/b2_friction_joint.h>
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/TParameterDefinitions.h>

#include <vector>
{
    class ElipseCurveMethod : public FrictionBase
    {
      DECLARES_REGISTER_FRICTION(ElipseCurveMethod)
      public:
      	ElipseCurveMethod(
		    VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node);

	  // See docs in base class.
	  virtual mrpt::math::TVector2D evaluate_friction(
		const FrictionBase::TFrictionInput& input) const override;

   private:
	 double CA_ = 8;	 //!< aerodynamic force coefficient (non-dimensional)
	 double Caf_ = 8.5, Cs_ = 7.5, ss_ = 0.1, Cafs_ = 0.5, Csaf_ = 0.5;   
   //!< Elipse curve constants  
  


   public:
	 const TParameterDefinitions params_ = {
		{"CA", {"%lf", & CA_}}, {"Cs", {"%lf", &Cs_}}
    {"ss", {"%lf", & ss_}}, {"Caf", {"%lf", &Caf_}}
    {"Casf", {"%lf", & Cafs_}}, {"Csaf", {"%lf", &Csaf_}}};
    }; 
}





