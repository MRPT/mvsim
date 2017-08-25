/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+ */

#include <mvsim/FrictionModels/FrictionBase.h>
#include <mvsim/FrictionModels/DefaultFriction.h>
#include <mvsim/FrictionModels/WardIagnemmaFriction.h>
#include <mvsim/VehicleBase.h>
#include <rapidxml.hpp>

using namespace mvsim;

TClassFactory_friction mvsim::classFactory_friction;

// Explicit registration calls seem to be one (the unique?) way to assure
// registration takes place:
void register_all_friction()
{
	static bool done = false;
	if (done)
		return;
	else
		done = true;

	REGISTER_FRICTION("default", DefaultFriction)
	REGISTER_FRICTION("wardiagnemma", WardIagnemmaFriction)
}

FrictionBase::FrictionBase(VehicleBase& my_vehicle)
	: m_world(my_vehicle.getWorldObject()), m_my_vehicle(my_vehicle)
{
}

FrictionBase::~FrictionBase() {}
FrictionBase* FrictionBase::factory(
	VehicleBase& parent, const rapidxml::xml_node<char>* xml_node)
{
	register_all_friction();

	using namespace std;
	using namespace rapidxml;

	if (!xml_node || 0 != strcmp(xml_node->name(), "friction"))
		throw runtime_error(
			"[FrictionBase::factory] Expected XML node <friction>");

	// Parse:
	const xml_attribute<>* frict_class = xml_node->first_attribute("class");
	if (!frict_class || !frict_class->value())
		throw runtime_error(
			"[FrictionBase::factory] Missing mandatory attribute 'class' in "
			"node <friction>");

	return classFactory_friction.create(frict_class->value(), parent, xml_node);
}

void FrictionBase::setLogger(const std::weak_ptr<CSVLogger>& logger)
{
	m_logger = logger;
}
