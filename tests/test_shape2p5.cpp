/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/opengl/CSphere.h>
#include <mvsim/CollisionShapeCache.h>

#include <iostream>

using namespace mvsim;

void test_shape1()
{
	auto& csc = CollisionShapeCache::Instance();
	const float radius = 0.5f;
	auto glSphere = mrpt::opengl::CSphere::Create(radius);

	const Shape2p5 shape = csc.get(*glSphere, -radius, +radius, {}, 1.0f);

	std::cout << shape.getContour() << std::endl;
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
	try
	{
		test_shape1();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cerr << e.what();
		return 1;
	}
}
