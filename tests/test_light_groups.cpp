/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

// Tests for switchable lights (<light_group> XML tags): parsing, switching
// them through the World API, and their effect on the 3D scenes (lights and
// emissive models), without rendering.

#include <mrpt/viz/CAssimpModel.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/Scene.h>
#include <mrpt/viz/TLightParameters.h>	// MRPT_VIZ_HAS_CLIGHT
#include <mvsim/VehicleBase.h>
#include <mvsim/World.h>
#if defined(MRPT_VIZ_HAS_CLIGHT)
#include <mrpt/viz/CLight.h>
#endif

#include <iostream>
#include <string>
#include <vector>

#include "test_utils.h"

int g_failures = 0;

namespace
{
const std::string kHeadlightModel = std::string(MVSIM_TEST_DIR) + "/../models/headlight.obj";

mvsim::VehicleBase::Ptr make_vehicle(
	mvsim::World& world, const std::string& name, const std::string& lightGroups)
{
	const std::string xml = "<vehicle name=\"" + name +
							"\">"
							"  <dynamics class=\"differential\">"
							"    <l_wheel pos=\"0  0.5\" />"
							"    <r_wheel pos=\"0 -0.5\" />"
							"  </dynamics>"
							"  <init_pose>0 0 0</init_pose>" +
							lightGroups + "</vehicle>";

	auto veh = mvsim::VehicleBase::factory(&world, xml);
	world.insert_vehicle(veh);
	return veh;
}

const std::string kLightGroups = R"(
	<light_group name="headlights" initially_on="false">
		<spot_light> <position>0.5 0.2 0.3</position> <direction>1 0 -0.2</direction> </spot_light>
		<spot_light> <position>0.5 -0.2 0.3</position> <direction>1 0 -0.2</direction> </spot_light>
	</light_group>
	<light_group name="beacon">
		<point_light> <position>0 0 0.8</position> <range>3</range> </point_light>
	</light_group>
	<visual light_group="headlights"> <model_uri>)" +
								 kHeadlightModel +
								 R"(</model_uri> </visual>
)";

struct SceneContents
{
	int lights = 0;	 //!< CLight objects
	int lightsOn = 0;  //!< ...visible, along with all their parents
	std::vector<mrpt::viz::CAssimpModel::Ptr> models;
};

void collect(mrpt::viz::CSetOfObjects& objs, bool parentVisible, SceneContents& out)
{
	for (const auto& o : objs)
	{
		if (!o)
		{
			continue;
		}
		const bool visible = parentVisible && o->isVisible();
		if (auto so = std::dynamic_pointer_cast<mrpt::viz::CSetOfObjects>(o); so)
		{
			collect(*so, visible, out);
		}
		if (auto m = std::dynamic_pointer_cast<mrpt::viz::CAssimpModel>(o); m)
		{
			out.models.push_back(m);
		}
#if defined(MRPT_VIZ_HAS_CLIGHT)
		if (std::dynamic_pointer_cast<mrpt::viz::CLight>(o))
		{
			out.lights++;
			out.lightsOn += visible ? 1 : 0;
		}
#endif
	}
}

SceneContents contentsOf(mrpt::viz::Scene& scene)
{
	auto all = mrpt::viz::CSetOfObjects::Create();
	for (const auto& o : *scene.getViewport())
	{
		all->insert(o);
	}
	SceneContents c;
	collect(*all, true, c);
	return c;
}

/** Brightest red emissive component among the parts of a model */
float maxEmissive(const mrpt::viz::CAssimpModel& m)
{
	float e = 0;
	for (const auto& part : m)
	{
		if (part)
		{
			e = std::max(e, part->materialEmissive().R);
		}
	}
	return e;
}

void test_parse_and_switch()
{
	std::cout << "[TEST] light groups: parse and switch\n";

	mvsim::World world;
	world.headless(true);
	world.internal_initialize();

	auto v1 = make_vehicle(world, "v1", kLightGroups);
	auto v2 = make_vehicle(world, "v2", kLightGroups);

	const auto names = v1->lightGroupNames();
	EXPECT_TRUE(names.size() == 2);
	EXPECT_TRUE(v1->lightGroupState("headlights") == std::optional<bool>(false));
	EXPECT_TRUE(v1->lightGroupState("beacon") == std::optional<bool>(true));
	EXPECT_FALSE(v1->lightGroupState("none").has_value());

	// World API:
	EXPECT_TRUE(world.setLightGroupState("v1", "headlights", true));
	EXPECT_TRUE(world.lightGroupState("v1", "headlights") == std::optional<bool>(true));
	EXPECT_TRUE(world.lightGroupState("v2", "headlights") == std::optional<bool>(false));
	EXPECT_FALSE(world.setLightGroupState("v1", "none", true));
	EXPECT_FALSE(world.setLightGroupState("none", "headlights", true));
	EXPECT_FALSE(world.lightGroupState("none", "headlights").has_value());

	// Effect on the 3D scenes:
	mrpt::viz::Scene viz;
	mrpt::viz::Scene physical;
	v1->guiUpdate(viz, physical);
	v2->guiUpdate(viz, physical);

#if defined(MRPT_VIZ_HAS_CLIGHT)
	// v1: 2 headlights + beacon on; v2: only its beacon. In both scenes, so
	// camera sensors also see them:
	for (auto* scene : {&viz, &physical})
	{
		const auto c = contentsOf(*scene);
		EXPECT_TRUE(c.lights == 6);
		EXPECT_TRUE(c.lightsOn == 4);
	}
	world.setLightGroupState("v1", "beacon", false);
	world.setLightGroupState("v2", "headlights", true);
	EXPECT_TRUE(contentsOf(physical).lightsOn == 5);
#endif

	// The lamp models glow only while their group is on, each vehicle with
	// its own model instance:
	const auto models = contentsOf(viz).models;
	EXPECT_TRUE(models.size() == 2);
	if (models.size() == 2)
	{
		EXPECT_TRUE(models[0] != models[1]);
		EXPECT_GT(maxEmissive(*models[0]), 0.5);
		EXPECT_GT(maxEmissive(*models[1]), 0.5);

		world.setLightGroupState("v2", "headlights", false);
		const int nOn =
			(maxEmissive(*models[0]) > 0.5 ? 1 : 0) + (maxEmissive(*models[1]) > 0.5 ? 1 : 0);
		EXPECT_TRUE(nOn == 1);
	}
}

void test_parse_errors()
{
	std::cout << "[TEST] light groups: XML errors\n";

	mvsim::World world;
	world.headless(true);
	world.internal_initialize();

	const auto throws = [&](const std::string& groups)
	{
		try
		{
			make_vehicle(world, "bad", groups);
		}
		catch (const std::exception&)
		{
			return true;
		}
		return false;
	};

	EXPECT_FALSE(throws("<light_group name='a'> <point_light/> </light_group>"));
	EXPECT_TRUE(throws("<light_group> <point_light/> </light_group>"));	 // no name
	EXPECT_TRUE(throws("<light_group name='a'> <lamp/> </light_group>"));  // unknown tag
	EXPECT_TRUE(throws(
		"<light_group name='a'> <point_light> <range>far</range> </point_light> </light_group>"));
}
}  // namespace

int main()
{
	try
	{
		test_parse_and_switch();
		test_parse_errors();
	}
	catch (const std::exception& e)
	{
		std::cerr << "Unexpected exception: " << e.what() << "\n";
		g_failures++;
	}

	if (g_failures == 0)
	{
		std::cout << "ALL TESTS PASSED\n";
	}
	else
	{
		std::cerr << g_failures << " TEST(S) FAILED\n";
	}
	return g_failures == 0 ? 0 : 1;
}
