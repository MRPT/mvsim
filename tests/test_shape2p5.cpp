/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/system/filesystem.h>	 // mrpt::system::pathJoin()
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

	std::cout << "Sphere:\n" << shape.getContour() << std::endl;
}

void shape_test_cylinder()
{
	auto& csc = CollisionShapeCache::Instance();

	const float radius = 0.5f, L = 2.0f;
	auto glCyl = mrpt::opengl::CCylinder::Create(radius, radius, L);

	const Shape2p5 shape = csc.get(*glCyl, 0, L, {}, 1.0f);

	std::cout << "Cylinder:\n" << shape.getContour() << std::endl;
}

void shape_test_merge()
{
	auto& csc = CollisionShapeCache::Instance();

	const float radius = 0.5f, L = 2.0f;
	auto glCyl = mrpt::opengl::CCylinder::Create(radius, radius, L);

	const Shape2p5 s1 =
		csc.get(*glCyl, 0, L, mrpt::poses::CPose3D::FromTranslation(-0.15, 0, 0), 1.0f);

	const Shape2p5 s2 =
		csc.get(*glCyl, 0, L, mrpt::poses::CPose3D::FromTranslation(0.15, 0, 0), 1.0f);

	Shape2p5 s = s1;
	s.mergeWith(s2);

	std::cout << "Cylinder 1:\n" << s1.getContour() << std::endl;
	std::cout << "Cylinder 2:\n" << s2.getContour() << std::endl;
	std::cout << "Cylinder 1+2:\n" << s.getContour() << std::endl;
}

void shape_test_simplecamera()
{
	auto& csc = CollisionShapeCache::Instance();

	auto glModel = mrpt::opengl::CAssimpModel::Create();
	glModel->loadScene(mrpt::system::pathJoin({MVSIM_TEST_DIR, "../models/simple_camera.dae"}));

	const Shape2p5 shape = csc.get(*glModel, 0, 1.0, {}, 1.0f);

	std::cout << "SimpleCamera .DAE:\n" << shape.getContour() << std::endl;
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
	std::vector<std::pair<std::function<void(void)>, std::string>> lst = {
		{&shape_test_sphere, "shape_test_sphere"},
		{&shape_test_cylinder, "shape_test_cylinder"},
		{&shape_test_merge, "shape_test_merge"},
		{&shape_test_simplecamera, "shape_test_simplecamera"},
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
