// Fernando Cañadas Aránega, 5 Nov 2025

#include <mvsim/FrictionModels/AdaptativeFriction.h>
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>

#include <rapidxml.hpp>

#include "xml_utils.h"

using namespace mvsim;

AdaptativeFriction::AdaptativeFriction(
	VehicleBase& my_vehicle, const rapidxml::xml_node<char>* node)
	: FrictionBase(my_vehicle), mu_(0.8), C_damping_(1.0), Crr_(0.02)
{
	if (node && 0 != strcmp(node->name(), "friction"))
		throw std::runtime_error("<friction>...</friction> XML node was expected!!");

	if (!node) return;

	const rapidxml::xml_node<char>* zonesNode = node->first_node("zones");
	if (!zonesNode) return;

	for (auto zoneNode = zonesNode->first_node("zone"); zoneNode;
		 zoneNode = zoneNode->next_sibling("zone"))
	{
		ZoneParams z;
		z.name = zoneNode->first_attribute("name") ? zoneNode->first_attribute("name")->value()
												   : "unknown";

		z.mu =
			zoneNode->first_attribute("mu") ? atof(zoneNode->first_attribute("mu")->value()) : mu_;
		z.C_damping = zoneNode->first_attribute("C_damping")
						  ? atof(zoneNode->first_attribute("C_damping")->value())
						  : C_damping_;
		z.Crr = zoneNode->first_attribute("Crr") ? atof(zoneNode->first_attribute("Crr")->value())
												 : Crr_;

		z.x_min = zoneNode->first_attribute("x_min")
					  ? atof(zoneNode->first_attribute("x_min")->value())
					  : -1e9;
		z.x_max = zoneNode->first_attribute("x_max")
					  ? atof(zoneNode->first_attribute("x_max")->value())
					  : 1e9;
		z.y_min = zoneNode->first_attribute("y_min")
					  ? atof(zoneNode->first_attribute("y_min")->value())
					  : -1e9;
		z.y_max = zoneNode->first_attribute("y_max")
					  ? atof(zoneNode->first_attribute("y_max")->value())
					  : 1e9;

		zones_.push_back(z);
	}
}

// See docs in base class.
mrpt::math::TVector2D AdaptativeFriction::evaluate_friction(
	const FrictionBase::TFrictionInput& input) const
{
	// ------------------------------------------------------------
	// 1. Obtener posición actual del vehículo
	// ------------------------------------------------------------
	const mrpt::math::TPose2D pose(myVehicle_.getPose());

	double mu_local = mu_;
	double C_damping_local = C_damping_;
	double Crr_local = Crr_;

	// ------------------------------------------------------------
	// 2. Buscar zona activa según (x,y)
	// ------------------------------------------------------------
	// Buscar zona activa según (x,y)
	for (const auto& z : zones_)
	{
		if (pose.x >= z.x_min && pose.x <= z.x_max && pose.y >= z.y_min && pose.y <= z.y_max)
		{
			mu_local = z.mu;
			C_damping_local = z.C_damping;
			Crr_local = z.Crr;
			break;
		}
	}

	// 💡 ACTUALIZAR los miembros internos para que getMu(), getCdamping(), getCrr()
	// reflejen la fricción actual:
	const_cast<AdaptativeFriction*>(this)->mu_ = mu_local;
	const_cast<AdaptativeFriction*>(this)->C_damping_ = C_damping_local;
	const_cast<AdaptativeFriction*>(this)->Crr_ = Crr_local;

	// Rotate wheel velocity vector from veh. frame => wheel frame
	const mrpt::poses::CPose2D wRot(0, 0, input.wheel.yaw);

	// Velocity of the wheel cog in the frame of the wheel itself:
	const mrpt::math::TVector2D vel_w = wRot.inverseComposePoint(input.wheelCogLocalVel);

	// Action/Reaction, slippage, etc:
	// DUDA PARA JL; EL MODELO NO TIENE EN CUENTA LA MASA DEL VEHICULO?
	// --------------------------------------
	const double mu = mu_;
	const double gravity = myVehicle_.parent()->get_gravity();
	const double partial_mass = input.Fz / gravity + input.wheel.mass;
	const double max_friction = mu * partial_mass * gravity;

	// 1) Lateral friction (decoupled sub-problem)
	// --------------------------------------------
	double wheel_lateral_friction = 0.0;  // direction: +y local wrt the wheel
	{
		// Impulse required to step the lateral slippage:
		wheel_lateral_friction = -vel_w.y * partial_mass / input.context.dt;

		wheel_lateral_friction = std::clamp(wheel_lateral_friction, -max_friction, max_friction);
	}

	// 2) Longitudinal friction (decoupled sub-problem)
	// -------------------------------------------------
	double wheel_long_friction = 0.0;  // direction: +x local wrt the wheel

	// (eq. 1)==> desired impulse in wheel spinning speed.
	// wheel_C_lon_vel = vel_w.x - input.wheel.w * 0.5*input.wheel.diameter

	// It should be = 0 for no slippage (nonholonomic constraint): find out
	// required wheel \omega:case '4':
	const double R = 0.5 * input.wheel.diameter;  // Wheel radius
	const double lon_constraint_desired_wheel_w = vel_w.x / R;

	const double desired_wheel_w_impulse = (lon_constraint_desired_wheel_w - input.wheel.getW());

	const double desired_wheel_alpha = desired_wheel_w_impulse / input.context.dt;

	// (eq. 3)==> Find out F_r
	// Iyy_w * \Delta\omega_w = dt*\tau-  R*dt*Fri    -C_damp * \omega_w * dt
	// "Damping" / internal friction of the wheel's shaft, etc.
	const double C_damping = C_damping_;
	// const mrpt::math::TPoint2D wheel_damping(- C_damping *
	// input.wheel_speed.x, 0.0);

	const double I_yy = input.wheel.Iyy;
	double F_friction_lon =
		(input.motorTorque - I_yy * desired_wheel_alpha - C_damping * input.wheel.getW()) / R;

	// Slippage: The friction with the ground is not infinite:
	F_friction_lon = std::clamp(F_friction_lon, -max_friction, max_friction);

	// Recalc wheel ang. velocity impulse with this reduced force:
	const double actual_wheel_alpha =
		(input.motorTorque - R * F_friction_lon - C_damping * input.wheel.getW()) / I_yy;

	// std::cout << "[AdaptativeFriction] Evaluating friction for pose: ("
	// 	<< pose.x << "," << pose.y << ")  mu=" << mu_local << std::endl;

	// Apply impulse to wheel's spinning:
	input.wheel.setW(input.wheel.getW() + actual_wheel_alpha * input.context.dt);

	wheel_long_friction = F_friction_lon;

	// Resultant force: In local (x,y) coordinates (Newtons) wrt the Wheel
	// -----------------------------------------------------------------------
	const mrpt::math::TPoint2D result_force_wrt_wheel(wheel_long_friction, wheel_lateral_friction);

	// Rotate to put: Wheel frame ==> vehicle local framework:
	mrpt::math::TVector2D res;
	wRot.composePoint(result_force_wrt_wheel, res);

	if (logger_ && !logger_->expired())
	{
		auto logger = logger_->lock();

		logger->updateColumn("desired_wheel_alpha", desired_wheel_alpha);
		logger->updateColumn("wheel_lateral_friction", wheel_lateral_friction);
		logger->updateColumn("desired_wheel_w_impulse", desired_wheel_w_impulse);
		logger->updateColumn("F_friction_lon", F_friction_lon);
		logger->updateColumn("actual_wheel_alpha", actual_wheel_alpha);
		logger->updateColumn("motor_torque", input.motorTorque);
	}

	return res;
}