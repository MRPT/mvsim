/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/system/os.h>
#include <mvsim/CollisionShapeCache.h>

#include <functional>
#include <iostream>
#include <vector>

#include "test_utils.h"

using namespace mvsim;

void shape_test_sphere()
{
	auto& csc = CollisionShapeCache::Instance();
	const float radius = 0.5f;
	auto glSphere = mrpt::opengl::CSphere::Create(radius);

	const Shape2p5 shape = csc.get(*glSphere, -radius, +radius, {}, 1.0f);

	std::cout << shape.getContour() << std::endl;
}

void shape_test_cylinder()
{
	auto& csc = CollisionShapeCache::Instance();

	const float radius = 0.5f, L = 2.0f;
	auto glCyl = mrpt::opengl::CCylinder::Create(radius, radius, L);

	const Shape2p5 shape = csc.get(*glCyl, 0, L, {}, 1.0f);

	std::cout << shape.getContour() << std::endl;
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
	std::vector<std::pair<std::function<void(void)>, std::string>> lst = {
		{&shape_test_sphere, "shape_test_sphere"},
		{&shape_test_cylinder, "shape_test_cylinder"},
	};

	bool anyFail = false;

	for (const auto& kv : lst)
	{
		try
		{
			setConsoleBlueColor();
			printf("[%20s] Running...\n", kv.second.c_str());
			setConsoleNormalColor();

			kv.first();

			setConsoleBlueColor();
			printf("[%20s] Success\n", kv.second.c_str());
			setConsoleNormalColor();
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what();
			setConsoleErrorColor();
			printf("[%20s] FAIL\n", kv.second.c_str());
			setConsoleNormalColor();
			anyFail = true;
		}
	}

	return anyFail ? 1 : 0;
}
