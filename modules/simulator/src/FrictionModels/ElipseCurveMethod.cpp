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

void fcn(double T3, double T4, double delta1, double delta2, 
double vx, double vy, double r, double w1, double w2, double w3, 
double w4, double &vxdot, double &vydot, double &rdot, double &w1dot, 
double &w2dot, double &w3dot, double &w4dot, double &Fx1, double &Fx2, 
double &Fx3, double &Fx4, double &Fy1, double &Fy2, double &Fy3, 
double &Fy4) { 

    double m = 1000.0;
    double Iz = 2000.0, I1 = 30.0, I2 = I1, I3 = I1, I4 = I1; 
    double Caf1 = 8.5, Caf2 = Caf1, Caf3 = Caf1, Caf4 = Caf1; 
    double Cs1 = 7.5, Cs2 = Cs1, Cs3 = Cs1, Cs4 = Cs1; 
    double afs = 5.0 * M_PI / 180.0; / 
    double ss = 0.1; 
    double Cafs = 0.5; 
    double Csaf = 0.5; 
    const double a1 = 1.35, a2 = 1.5; 
    double l = a1 + a2; 
    double h = 0.9;  
    const double b1f = 0.9, b1r = b1f, b2f = b1f, b2r = b1f; 
    const double Rg = 0.35; 

    const double Wf = b1f + b2f, Wr = b1r + b2r;
    double x1 = a1, y1 = b1f, x2 = a1, y2 = -b2f, x3 = -a2, y3 = b1r,
	 x4 = -a2, y4 = -b2r; 
    double CA = 0.8; 
    double g = 9.81; 
    double T1 = 0.0, T2 = 0.0; 
    double delta3 = 0.0, delta4 = 0.0; 

    

     // 1) Vertical forces (decoupled sub-problem)
	// --------------------------------------------

    double Fz1 = (m / (l * Wf * g)) * (a2 * g - h * (vxdot - r * vy)) * (b2f * g - h * (vydot + r * vx)); 
    double Fz2 = (m / (l * Wf * g)) * (a2 * g - h * (vxdot - r * vy)) * (b1f * g + h * (vydot + r * vx)); 
    double Fz3 = (m / (l * Wr * g)) * (a1 * g + h * (vxdot - r * vy)) * (b2r * g - h * (vydot + r * vx)); 
    double Fz4 = (m / (l * Wr * g)) * (a1 * g + h * (vxdot - r * vy)) * (b1r * g + h * (vydot + r * vx)); 

 
	// 2) Wheels velocity at Tire SR (decoupled sub-problem)
	// -------------------------------------------------
    
    double vxT1 = (vx - r * b1f) * cos(delta1) + (vy + r * a1) * sin(delta1); 
    double vxT2 = (vx + r * b1f) * cos(delta2) + (vy + r * a1) * sin(delta2); 
    double vxT3 = vx - r * b2r; 
    double vxT4 = vx + r * b2r; 


 	// 3) Longitudinal slip (decoupled sub-problem)
	// -------------------------------------------------

    double s1 = (Rg * w1 - vxT1) / (Rg * w1 * miH(Rg * w1, vxT1) + vxT1 * miH(vxT1, Rg * w1)); 
    double s2 = (Rg * w2 - vxT2) / (Rg * w2 * miH(Rg * w2, vxT2) + vxT2 * miH(vxT2, Rg * w2)); 
    double s3 = (Rg * w3 - vxT3) / (Rg * w3 * miH(Rg * w3, vxT3) + vxT3 * miH(vxT3, Rg * w3)); 
    double s4 = (Rg * w4 - vxT4) / (Rg * w4 * miH(Rg * w4, vxT4) + vxT4 * miH(vxT4, Rg * w4)); 

 

    if (std::isnan(s1)) s1 = 0; 
    if (std::isnan(s2)) s2 = 0; 
    if (std::isnan(s3)) s3 = 0; 
    if (std::isnan(s4)) s4 = 0; 

 
	// 4) Sideslip angle (decoupled sub-problem)
	// -------------------------------------------------

    double af1 = atan2((vy + x1 * r), (vx - y1 * r)) - delta1; 
    double af2 = atan2((vy + x2 * r), (vx - y2 * r)) - delta2; 
    double af3 = atan2((vy + x3 * r), (vx - y3 * r)) - delta3; 
    double af4 = atan2((vy + x4 * r), (vx - y4 * r)) - delta4; 

 

    Fx1 = Fz1 * Cs1 * miS(s1, ss) * sqrt(1 - Csaf * pow((miS(af1, afs) / afs), 2)); 
    Fx2 = Fz2 * Cs2 * miS(s2, ss) * sqrt(1 - Csaf * pow((miS(af2, afs) / afs), 2)); 
    Fx3 = Fz3 * Cs3 * miS(s3, ss) * sqrt(1 - Csaf * pow((miS(af3, afs) / afs), 2)); 
    Fx4 = Fz4 * Cs4 * miS(s4, ss) * sqrt(1 - Csaf * pow((miS(af4, afs) / afs), 2)); 

    Fy1 = -Fz1 * Caf1 * miS(af1, afs) * sqrt(1 - Cafs * pow((miS(s1, ss) / ss), 2)); 
    Fy2 = -Fz2 * Caf2 * miS(af2, afs) * sqrt(1 - Cafs * pow((miS(s2, ss) / ss), 2)); 
    Fy3 = -Fz3 * Caf3 * miS(af3, afs) * sqrt(1 - Cafs * pow((miS(s3, ss) / ss), 2)); 
    Fy4 = -Fz4 * Caf4 * miS(af4, afs) * sqrt(1 - Cafs * pow((miS(s4, ss) / ss), 2)); 

 

    double Fx = Fx1 * cos(delta1) + Fx2 * cos(delta2) + Fx3 + Fx4 - Fy1 * sin(delta1) - Fy2 * sin(delta2); 
    double Fy = Fy1 * cos(delta1) + Fy2 * cos(delta2) + Fy3 + Fy4 + Fx1 * sin(delta1) + Fx2 * sin(delta2); 
    double Mz = a1 * Fx1 * sin(delta1) + a1 * Fy1 * cos(delta1) + a1 * Fx2 * sin(delta2) + a1 * Fy2 * cos(delta2) - a2 * Fy3 - a2 * Fy4 - b1f * Fx1 * cos(delta1) + b1f * Fy1 * sin(delta1) - b2f * Fy2 * sin(delta2) + b2f * Fx2 * cos(delta2) - b1r * Fx3 + b2r * Fx4; 

 

    vxdot = ((Fx - CA * pow(vx, 2)) / m) + r * vy; 
    vydot = (Fy / m) - r * vx; 
    rdot = Mz / Iz; 

    w1dot = (T1 / I1) - Rg * Fx1 / I1; 
    w2dot = (T2 / I2) - Rg * Fx2 / I2; 
    w3dot = (T3 / I3) - Rg * Fx3 / I3; 
    w4dot = (T4 / I4) - Rg * Fx4 / I4; 

} 









	// 5) Tire forces (decoupled sub-problem)
	// -------------------------------------------------

	// 6) Forces and moments (decoupled sub-problem)
	// -------------------------------------------------

	// 7) EOMs (decoupled sub-problem)
	// -------------------------------------------------

