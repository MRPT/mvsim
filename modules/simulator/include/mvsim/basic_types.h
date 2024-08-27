/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

// Misc. types & forwards declarations

#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/TTwist2D.h>

#include <cstdint>	// uintptr_t
#include <string>
#include <vector>

class b2World;
class b2Body;
class b2Fixture;

namespace rapidxml
{
// Forward declarations
template <class Ch>
class xml_node;
template <class Ch>
class xml_attribute;
template <class Ch>
class xml_document;
}  // namespace rapidxml

namespace mrpt
{
namespace opengl
{
class CSetOfObjects;
}  // namespace opengl
namespace slam
{
class CObservation;
}
}  // namespace mrpt

namespace mvsim
{
class World;
class VehicleBase;
template <typename Ch = char>
class JointXMLnode;

/** Simulation context for simulable objects updates */
struct TSimulContext
{
	b2World* b2_world = nullptr;
	World* world = nullptr;
	double simul_time = 0;	//!< Current time in the simulated world
	double dt = 0;	//!< timestep
};

/// Used to signal a Box2D fixture as "invisible" to sensors.
constexpr uintptr_t INVISIBLE_FIXTURE_USER_DATA = 1;

struct TJoyStickEvent
{
	std::vector<float> axes;
	std::vector<bool> buttons;

	TJoyStickEvent() = default;
};

}  // namespace mvsim
