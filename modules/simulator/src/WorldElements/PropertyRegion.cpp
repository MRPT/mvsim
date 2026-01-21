/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/World.h>
#include <mvsim/WorldElements/PropertyRegion.h>

#include <rapidxml.hpp>

#include "../parse_utils.h"
#include "xml_utils.h"

using namespace mvsim;

PropertyRegion::PropertyRegion(World* parent, const rapidxml::xml_node<char>* root)
	: WorldElementBase(parent)
{
	PropertyRegion::loadConfigFrom(root);
}

void PropertyRegion::loadConfigFrom(const rapidxml::xml_node<char>* root)
{
	TParameterDefinitions params;

	// Bounding Box
	params["x_min"] = TParamEntry("%lf", &boxMin_.x);
	params["y_min"] = TParamEntry("%lf", &boxMin_.y);
	params["z_min"] = TParamEntry("%lf", &boxMin_.z);
	params["x_max"] = TParamEntry("%lf", &boxMax_.x);
	params["y_max"] = TParamEntry("%lf", &boxMax_.y);
	params["z_max"] = TParamEntry("%lf", &boxMax_.z);

	// Property Metadata
	params["property_name"] = TParamEntry("%s", &propertyName_);

	// Values (we parse all, but only the last one defined in XML effectively sets the type)
	double valDouble = 0;
	bool valBool = false;
	std::string valString;

	params["value_double"] = TParamEntry("%lf", &valDouble);
	params["value_bool"] = TParamEntry("%bool", &valBool);
	params["value_string"] = TParamEntry("%s", &valString);

	parse_xmlnode_children_as_param(*root, params, world_->user_defined_variables());

	// Determine which value was actually provided in the XML to store in std::any
	if (root->first_node("value_double"))
	{
		propertyValue_ = valDouble;
	}
	else if (root->first_node("value_bool"))
	{
		propertyValue_ = valBool;
	}
	else if (root->first_node("value_string"))
	{
		propertyValue_ = valString;
	}
}

std::optional<std::any> PropertyRegion::queryProperty(
	const std::string& propertyName, const mrpt::math::TPoint3D& worldXYZ) const
{
	// 1. Check if name matches
	if (propertyName != propertyName_)
	{
		return {};
	}

	// 2. Check AABB bounds
	const bool inside =
		(worldXYZ.x >= boxMin_.x && worldXYZ.x <= boxMax_.x && worldXYZ.y >= boxMin_.y &&
		 worldXYZ.y <= boxMax_.y && worldXYZ.z >= boxMin_.z && worldXYZ.z <= boxMax_.z);

	if (inside)
	{
		return propertyValue_;
	}

	return {};
}