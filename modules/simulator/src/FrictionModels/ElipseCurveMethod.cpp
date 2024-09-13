/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */


#include <mvsim/FrictionModels/ElipseCurveMethod.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <cmath>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;

ElipseCurveMethod::ElipseCurveMethod(
	VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node)
	: FrictionBase(my_vehicle), CA_(8), Caf_(8.5), Cs_(7.5), 
  ss_(0.1), Cafs_(0.5), Csaf_(0.5)
{
	// Sanity: we can tolerate node==nullptr (=> means use default params).
	if (node && 0 != strcmp(node->name(), "friction"))
		throw std::runtime_error(
			"<friction>...</friction> XML node was expected!!");

	// Parse XML params:
	if (node)
		parse_xmlnode_children_as_param(
			*node, params_, world_->user_defined_variables());
}

// See docs in base class.
mrpt::math::TVector2D ElipseCurveMethod::evaluate_friction(
	const FrictionBase::TFrictionInput& input) const
{
	// Rotate wheel velocity vector from veh. frame => wheel frame
	const mrpt::poses::CPose2D wRot(0, 0, input.wheel.yaw);

	// Velocity of the wheel cog in the frame of the wheel itself:
	const mrpt::math::TVector2D vel_w =
		wRot.inverseComposePoint(input.wheelCogLocalVel);

	// Action/Reaction, slippage, etc:
	// --------------------------------------
	const double mu = mu_;
	const double gravity = myVehicle_.parent()->get_gravity();
	const double partial_mass = input.weight / gravity + input.wheel.mass;
	const double max_friction = mu * partial_mass * gravity;

    // Heaviside function
	double miH(double x, double x0) {
    if (x > x0)
        return 1.0;
    else 
        return 0.0;
    }
    // Saturation function
    double miS(double x, double x0) {
    return x * miH(x0, std::abs(x)) + x0 * miH(std::abs(x), x0);
    }

void fcn(double delta1, double delta2, 
double vx, double vy, double r, double &vxdot, double &vydot, double &rdot, double &w1dot, 
double &w2dot, double &w3dot, double &w4dot, double &Fx1, double &Fx2, 
double &Fx3, double &Fx4, double &Fy1, double &Fy2, double &Fy3, 
double &Fy4) { 

    const double m = chassiss_mass_; // masa del conjunto

    const double Caf = Caf_; 
    const double Cs = Cs_; 
    const double afs = 5.0 * M_PI / 180.0; 
    const double ss = ss_; 
    const double Cafs = Cafs_; 
    const double Csaf = Csaf_; 
    const double CA = CA_; 
    const double gravity = myVehicle_.parent()->get_gravity(); 
    
    
    
    double h = 0.9;  
    const double R = 0.35; // radio rueda
    
    double Iz = 2000.0, I = 30.0; // inercias 
    double T1 = 0.0, T2 = 0.0; 
    double delta3 = 0.0, delta4 = 0.0; 

    

     // 1) Vertical forces (decoupled sub-problem)
	// --------------------------------------------
    // obtener posiciones y distancias de ejes
    mrpt::math::TVector2D Center_of_mass = getChassisCenterOfMass();
 	const size_t nW = this->getNumWheels();    //esta linea ya existe en VehicleBase.ccp linea 568
	mrpt::math::TVector2D> pos_(nW);

    for (size_t i = 0; i < nW; i++)  // quiza seria mejor sacar este codigo de la función para que solo se ejecute 1 vez
	{
    double pos_[i].x = Wheels_info_[i].x - Center_of_mass.x; 
    double pos_[i].y = Wheels_info_[i].y - Center_of_mass.y;  
    }


   // const double a1 = std::abs(pos_[0].x), a2 = std::abs(pos_[3].x); // distancia centro de gravedad a ejes
    const double l = std::abs(pos_[0].x) + std::abs(pos_[3].x); // distancia entre ejes
    const double Axf = std::abs(pos_[0].y)+std::abs(pos_[1].y), Axr = std::abs(pos_[2].y)+std::abs(pos_[3].y); // longitud ejes
    const double w = veh_vel_local.omega;


    //// crear un if para cada rueda
    double Fz1 = (m / (l * Axf * gravity)) * (a2 * gravity - h * (vxdot - w * veh_vel_local.vy)) * (b2f * gravity - h * (vydot + w * veh_vel_local.vx)); 
    double Fz2 = (m / (l * Axf * gravity)) * (a2 * gravity - h * (vxdot - w * veh_vel_local.vy)) * (b1f * gravity + h * (vydot + w * veh_vel_local.vx)); 
    double Fz3 = (m / (l * Axr * gravity)) * (a1 * gravity + h * (vxdot - w * veh_vel_local.vy)) * (b2r * gravity - h * (vydot + w * veh_vel_local.vx)); 
    double Fz4 = (m / (l * Axr * gravity)) * (a1 * gravity + h * (vxdot - w * veh_vel_local.vy)) * (b1r * gravity + h * (vydot + w * veh_vel_local.vx)); 
    double max_friction = Fz;
 
	// 2) Wheels velocity at Tire SR (decoupled sub-problem)
	// -------------------------------------------------
    //duda de cambiar el codigo o no) VehicleBase.cpp line 575 calcula esto pero distinto    
    double vxT = (veh_vel_local.vx - w * pos_[i].y) * cos(delta1) + (veh_vel_local.vy + w * pos_[i].x) * sin(delta1); 
    
 	// 3) Longitudinal slip (decoupled sub-problem)
	// -------------------------------------------------
    
    // w= velocidad angular
    double s = (R * input.wheel.getW() - vxT) / (R * input.wheel.getW() * miH(R * input.wheel.getW(), vxT) + vxT * miH(vxT, R * input.wheel.getW())); 

    if (std::isnan(s)) s = 0; 

 
	// 4) Sideslip angle (decoupled sub-problem)
	// -------------------------------------------------

    double af = atan2((veh_vel_local.vy + pos_x * r), (veh_vel_local.vx - pos_y * r)) - delta1; 

	// 5) Longitudinal friction (decoupled sub-problem)
	// -------------------------------------------------
    double wheel_long_friction = max_friction * Cs * miS(s, ss) * sqrt(1 - Csaf * pow((miS(af, afs) / afs), 2)); 
    
    // 6) Lateral friction (decoupled sub-problem)
	// --------------------------------------------
    double wheel_lat_friction = -max_friction * Caf * miS(af, afs) * sqrt(1 - Cafs * pow((miS(s, ss) / ss), 2)); 

 
	// 6) Forces and moments (decoupled sub-problem)
	// -------------------------------------------------

    // double Fx = Fx1 * cos(delta1) + Fx2 * cos(delta2) + Fx3 + Fx4 - Fy1 * sin(delta1) - Fy2 * sin(delta2); 
    //double Fy = Fy1 * cos(delta1) + Fy2 * cos(delta2) + Fy3 + Fy4 + Fx1 * sin(delta1) + Fx2 * sin(delta2); 
    //double Mz = a1 * Fx1 * sin(delta1) + a1 * Fy1 * cos(delta1) + a1 * Fx2 * sin(delta2) + a1 * Fy2 * cos(delta2) - a2 * Fy3 - a2 * Fy4 - b1f * Fx1 * cos(delta1) + b1f * Fy1 * sin(delta1) - b2f * Fy2 * sin(delta2) + b2f * Fx2 * cos(delta2) - b1r * Fx3 + b2r * Fx4; 

 
	// 7) EOMs (decoupled sub-problem)
	// -------------------------------------------------

    // vxdot = ((Fx - CA * pow(vx, 2)) / m) + r * vy; 
    // vydot = (Fy / m) - r * vx; 
    // rdot = Mz / Iz; 

    // Recalc wheel ang. velocity impulse with this reduced force:
    const double I_yy = input.wheel.Iyy;
    const double actual_wheel_alpha = (input.motorTorque / I_yy) - R * wheel_long_friction / I_yy; 


    input.wheel.setW(
		input.wheel.getW() + actual_wheel_alpha * input.context.dt);



	// Resultant force: In local (x,y) coordinates (Newtons) wrt the Wheel
	// -----------------------------------------------------------------------
	const mrpt::math::TPoint2D result_force_wrt_wheel(
		wheel_long_friction, wheel_lat_friction);

	// Rotate to put: Wheel frame ==> vehicle local framework:
	mrpt::math::TVector2D res;
	wRot.composePoint(result_force_wrt_wheel, res);
	return res;



} 
}









