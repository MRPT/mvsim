/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/opengl/CPolyhedron.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/system/filesystem.h>
#include <mvsim/FrictionModels/DefaultFriction.h>  // For use as default model
#include <mvsim/FrictionModels/EllipseCurveMethod.h>
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/Simulable.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/VehicleDynamics/VehicleAckermann.h>
#include <mvsim/VehicleDynamics/VehicleAckermann_Drivetrain.h>
#include <mvsim/VehicleDynamics/VehicleDifferential.h>
#include <mvsim/Wheel.h>
#include <mvsim/World.h>

#include <cmath>
#include <rapidxml.hpp>

#include "parse_utils.h"
#include "xml_utils.h"

using namespace mvsim;

EllipseCurveMethod::EllipseCurveMethod(
	VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node)
	: FrictionBase(my_vehicle), CA_(8), Caf_(8.5), Cs_(7.5), ss_(0.1), Cafs_(0.5), Csaf_(0.5)
{
	// Sanity: we can tolerate node==nullptr (=> means use default params).
	if (node && 0 != strcmp(node->name(), "friction"))
	{
		throw std::runtime_error("<friction>...</friction> XML node was expected!!");
	}

	// Parse XML params:
	if (node)
	{
		parse_xmlnode_children_as_param(*node, params_, world_->user_defined_variables());
	}
}

// aqui ya cambia el codigo
namespace
{
// Heaviside function
double miH(double x, double x0) { return (x > x0) ? 1.0 : 0.0; }
// Saturation function
double miS(double x, double x0) { return x * miH(x0, std::abs(x)) + x0 * miH(std::abs(x), x0); }
}  // namespace

// See docs in base class.
mrpt::math::TVector2D EllipseCurveMethod::evaluate_friction(
	const FrictionBase::TFrictionInput& input) const
{
	// This function is called once for each wheel of the vehicle, for each simulation time step.

	// vehicle center of mass, wrt local vehicle frame:
	const auto com = myVehicle_.getChassisCenterOfMass();
	const auto wheel_pos_wrt_com = mrpt::math::TPoint2D(input.wheel.pose().translation()) - com;

	// pasar a local la velocidad
	// const mrpt::math::TVector3Df linAccGlobal = myVehicle_.getLinearAcceleration();
	// const mrpt::poses::CPose3D vehiclePose3D(myVehicle_.getPose());
	// mrpt::math::TVector3Df linAccLocal = vehiclePose3D.inverseComposePoint(linAccGlobal);

	const auto linAccLocal = myVehicle_.getLinearAcceleration();

	// const mrpt::math::TVector2D linAccLocal = getAcc();
	//  ¿Está bien? no se si se corresponde con la aceleración que quiero
	const mrpt::math::TTwist2D& vel = myVehicle_.getVelocityLocal();
	const double w = vel.omega;

	// wheel angle around the vertical axis:
	const double delta = input.wheel.yaw;

	// Rotate wheel velocity vector from veh. frame => wheel frame
	const mrpt::poses::CPose2D wRot(pos[wheel_index].x, pos[wheel_index].y, delta);

	// Velocity of the wheel cog in the frame of the wheel itself: == vxT
	// const mrpt::math::TVector2D vel_w = wRot.inverseComposePoint(input.wheelCogLocalVel);

	const double h = 0.40;	// altura del centro de gravedad provisional
	//--------------------------------------------------------------------------

	const double m = myVehicle_.getChassisMass();  // masa del conjunto
	const double afs = 5.0 * M_PI / 180.0;
	// const double CA = CA_;
	const double gravity = myVehicle_.parent()->get_gravity();
	const double R = 0.5 * input.wheel.diameter;  // Wheel radius

	// distancia centro de gravedad a ejes
	const double a1 = std::abs(pos[3].x), a2 = std::abs(pos[0].x);
	const double l = a1 + a2;  // distancia entre ejes
	// longitud ejes
	const double Axf = std::abs(pos[2].y) + std::abs(pos[3].y),
				 Axr = std::abs(pos[0].y) + std::abs(pos[1].y);
	ASSERT_(Axf > 0);  // comprobar si la distancia de los ejes son mayores a 0
	ASSERT_(Axr > 0);

	// 1) Vertical forces (decoupled sub-problem)
	// --------------------------------------------
	//// crear un if para cada rueda
	// Wheels: [0]:rear-left, [1]:rear-right, [2]: front-left, [3]: front-right
	double Fz = 0.0;  // Declaración antes del if

	if (wheel_index == 3)  //(Wpos.x > 0 && Wpos.y > 0)
	{
		Fz = std::abs(
			(m / (l * Axf * gravity)) * (a2 * gravity - h * (linAccLocal.x - w * vel.vy)) *
			(std::abs(pos[1].y) * gravity - h * (linAccLocal.y + w * vel.vx)));
	}
	else if (wheel_index == 2)	//(Wpos.x < 0 && Wpos.y > 0)
	{
		Fz = std::abs(
			(m / (l * Axf * gravity)) * (a2 * gravity - h * (linAccLocal.x - w * vel.vy)) *
			(std::abs(pos[0].y) * gravity + h * (linAccLocal.y + w * vel.vx)));
	}
	else if (wheel_index == 1)	//(Wpos.x > 0 && Wpos.y < 0)
	{
		Fz = std::abs(
			(m / (l * Axr * gravity)) * (a1 * gravity + h * (linAccLocal.x - w * vel.vy)) *
			(std::abs(pos[3].y) * gravity - h * (linAccLocal.y + w * vel.vx)));
	}
	else if (wheel_index == 0)	//(Wpos.x < 0 && Wpos.y < 0)
	{
		Fz = std::abs(
			(m / (l * Axr * gravity)) * (a1 * gravity + h * (linAccLocal.x - w * vel.vy)) *
			(std::abs(pos[2].y) * gravity + h * (linAccLocal.y + w * vel.vx)));
	}
	else
	{
		throw std::runtime_error("Invalid wheel index");  // Sin es mas de 4 ruedas generar error
	}

	const double max_friction = Fz;

	// 2) Wheels velocity at Tire SR (decoupled sub-problem)
	// -------------------------------------------------
	// duda de cambiar el codigo o no) VehicleBase.cpp line 575 calcula esto pero distinto
	const double vxT = (vel.vx - w * pos[wheel_index].y) * cos(delta) +
					   (vel.vy + w * pos[wheel_index].x) * sin(delta);

	// 3) Longitudinal slip (decoupled sub-problem)
	// -------------------------------------------------
	// w= velocidad angular
	double s = (R * input.wheel.getW() - vxT) /
			   (R * input.wheel.getW() * miH(R * input.wheel.getW(), vxT) +
				vxT * miH(vxT, R * input.wheel.getW()));

	// 4) Sideslip angle (decoupled sub-problem)
	// -------------------------------------------------
	double af = atan2((vel.vy + pos[wheel_index].x * w), (vel.vx - pos[wheel_index].y * w)) - delta;

	// 5) Longitudinal friction (decoupled sub-problem)
	// -------------------------------------------------
	double wheel_long_friction = 0.0;
	wheel_long_friction =
		max_friction * Cs_ * miS(s, ss_) * sqrt(1 - Csaf_ * pow((miS(af, afs) / afs), 2));
	// wheel_long_friction = b2Clamp(wheel_long_friction, -1.0, 1.0);

	// 6) Lateral friction (decoupled sub-problem)
	// --------------------------------------------
	double wheel_lat_friction = 0.0;
	wheel_lat_friction =
		-max_friction * Caf_ * miS(af, afs) * sqrt(1 - Cafs_ * pow((miS(s, ss_) / ss_), 2));
	// wheel_lat_friction = b2Clamp(wheel_lat_friction, -1.0, 1.0);

	// Recalc wheel ang. velocity impulse with this reduced force:
	const double I_yy = input.wheel.Iyy;
	const double actual_wheel_alpha = (input.motorTorque - R * wheel_long_friction) / I_yy;

	// Apply impulse to wheel's spinning:
	input.wheel.setW(input.wheel.getW() + actual_wheel_alpha * input.context.dt);

	// Resultant force: In local (x,y) coordinates (Newtons) wrt the Wheel
	// -----------------------------------------------------------------------
	const mrpt::math::TPoint2D result_force_wrt_wheel(wheel_long_friction, wheel_lat_friction);

	// recalcular aceleración

	// mostrar en pantalla los resultados
	const double Fx = wheel_long_friction;
	const double Fy = wheel_lat_friction;
	const double Vx = vel.vx;
	const double Vy = vel.vy;
	const double Acx = linAccLocal.x;
	const double Acy = linAccLocal.y;

	// Rotate to put: Wheel frame ==> vehicle local framework:
	mrpt::math::TVector2D res;
	wRot.composePoint(result_force_wrt_wheel, res);

	// Logger:
	if (logger_ && !logger_->expired())
	{
		auto logger = logger_->lock();

		logger->updateColumn("actual_wheel_alpha", actual_wheel_alpha);
		logger->updateColumn("motorTorque", input.motorTorque);
		logger->updateColumn("wheel_long_friction", wheel_long_friction);
		logger->updateColumn("wheel_lat_friction", wheel_lat_friction);
	}

	return res;
}
