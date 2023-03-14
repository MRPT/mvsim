/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/math/geometry.h>
#include <mvsim/Shape2p5.h>

#include <cmath>
//#include <iostream>

using namespace mvsim;

Shape2p5 Shape2p5::CreateConvexHullFromPoints(
	const std::vector<mrpt::math::TPoint3Df>& pts)
{
	ASSERT_GE_(pts.size(), 3UL);

	Shape2p5 ret;

	// zMin,zMax:
	// --------------------------
	ret.zMax = -std::numeric_limits<float>::max();
	ret.zMin = +std::numeric_limits<float>::max();

	for (const auto& p : pts)
	{
		mrpt::keep_max(ret.zMax, p.z);
		mrpt::keep_min(ret.zMin, p.z);
	}

	// Convex hull:
	// --------------------------
	ret.contour.emplace_back(-0.25, -0.25);
	ret.contour.emplace_back(-0.25, +0.25);
	ret.contour.emplace_back(+0.25, +0.25);
	ret.contour.emplace_back(+0.25, -0.25);

	// Polygon pruning to make it fit into b2Box maximum edge count:
	// ------------------------------------------------------------------
	MRPT_TODO("todo ");

#if 0
	std::cout << "[CreateConvexHullFromPoints] #pts=" << pts.size()
			  << " : zMin=" << ret.zMin << " zMax=" << ret.zMax
			  << " contour: " << ret.contour << "\n";
#endif

	return ret;
}

double Shape2p5::volume() const
{
	return std::abs(mrpt::math::signedArea(contour)) * std::abs(zMin - zMax);
}

void Shape2p5::mergeWith(const Shape2p5& s)
{
	ASSERT_(!s.contour.empty());

	std::vector<mrpt::math::TPoint3Df> pts;
	for (const auto& p : s.contour) pts.emplace_back(p.x, p.y, s.zMin);
	pts.emplace_back(pts.front().x, pts.front().y, s.zMax);

	mergeWith(pts);
}

void Shape2p5::mergeWith(const std::vector<mrpt::math::TPoint3Df>& pts)
{
	ASSERT_(!contour.empty());

	std::vector<mrpt::math::TPoint3Df> allPts;
	for (const auto& p : contour) allPts.emplace_back(p.x, p.y, zMin);
	allPts.emplace_back(allPts.front().x, allPts.front().y, zMax);

	for (const auto& p : pts) allPts.emplace_back(p);

	*this = CreateConvexHullFromPoints(allPts);
}
