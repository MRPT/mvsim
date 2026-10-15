/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/FrictionModels/DefaultFriction.h>
#include <mvsim/FrictionModels/EllipseCurveMethod.h>
#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/FrictionModels/WardIagnemmaFriction.h>
#include <mvsim/VehicleBase.h>

#include <algorithm>
#include <cmath>
#include <rapidxml.hpp>

using namespace mvsim;

TClassFactory_friction mvsim::classFactory_friction;

// Explicit registration calls seem to be one (the unique?) way to assure
// registration takes place:
void register_all_friction()
{
	static bool done = false;
	if (done)
	{
		return;
	}

	done = true;

	REGISTER_FRICTION("default", DefaultFriction)
	REGISTER_FRICTION("wardiagnemma", WardIagnemmaFriction)
	REGISTER_FRICTION("ellipse", EllipseCurveMethod)
}

FrictionBase::FrictionBase(VehicleBase& my_vehicle)
	: world_(my_vehicle.parent()), myVehicle_(my_vehicle)
{
}

FrictionBase::~FrictionBase() {}
FrictionBase::Ptr FrictionBase::factory(
	VehicleBase& parent, const rapidxml::xml_node<char>* xml_node)
{
	register_all_friction();

	using namespace std;
	using namespace rapidxml;

	if (!xml_node || 0 != strcmp(xml_node->name(), "friction"))
	{
		throw runtime_error("[FrictionBase::factory] Expected XML node <friction>");
	}

	// Parse:
	const xml_attribute<>* frict_class = xml_node->first_attribute("class");
	if (!frict_class || !frict_class->value())
	{
		throw runtime_error(
			"[FrictionBase::factory] Missing mandatory attribute 'class' in "
			"node <friction>");
	}

	return classFactory_friction.create(frict_class->value(), parent, xml_node);
}

void FrictionBase::setLogger(const std::weak_ptr<CSVLogger>& logger) { logger_ = logger; }

void FrictionBase::solveWheelSpin(
	const TFrictionInput& input, double otherTorque, double desiredAlpha, double R, double maxF,
	double& F, double& alpha)
{
	const double dt = input.context.dt;
	const auto solve = [&](double motorTorque, double motorDamping)
	{
		// Effective inertia includes the motor reaction to the spin change:
		const double I = input.wheel.Iyy + dt * motorDamping;
		const double torque = motorTorque + otherTorque;
		// No-slip force, reduced by the chassis reaction within the step: the
		// ground point speed changes by F*dt/longitudinalMass while the wheel
		// spins up by -R*F*dt/I.
		const double k =
			input.longitudinalMass > 0 ? 1.0 / (1.0 + I / (input.longitudinalMass * R * R)) : 1.0;
		F = std::clamp(k * (torque - I * desiredAlpha) / R, -maxF, maxF);
		alpha = (torque - R * F) / I;
	};

	const auto& m = input.motor;
	if (m.damping <= 0)
	{
		solve(input.motorTorque, 0);
		return;
	}

	solve(m.unsaturated, m.damping);

	// If the motor would end up saturated, it applies its limit all along the
	// step (exact, since the torque law is monotonic):
	const double endTorque = m.unsaturated - m.damping * dt * alpha;
	if (m.limit > 0 && std::abs(endTorque) > m.limit)
	{
		solve(endTorque > 0 ? m.limit : -m.limit, 0);
	}
}
