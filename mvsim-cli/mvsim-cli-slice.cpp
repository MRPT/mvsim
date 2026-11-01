/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/typemeta/TEnumType.h>
#include <mrpt/viz/CSetOfObjects.h>
#include <mrpt/viz/CVisualObject.h>
#include <mrpt/viz/Scene.h>
#include <mvsim/World.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <utility>
#include <vector>

#include "mvsim-cli.h"

namespace
{
using mrpt::math::TPoint3D;

/** Affine transform p' = A*p + t, with A = rotation x scale, stored as the
 * rows of the 3x4 matrix [A|t]. */
struct Affine3
{
	std::array<std::array<double, 4>, 3> m = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}}};

	template <typename POINT>
	TPoint3D apply(const POINT& p) const
	{
		TPoint3D r;
		for (int i = 0; i < 3; i++)
		{
			r[i] = m[i][0] * p.x + m[i][1] * p.y + m[i][2] * p.z + m[i][3];
		}
		return r;
	}

	/** Returns this * [R(pose)*diag(scale) | t(pose)] */
	Affine3 compose(const mrpt::poses::CPose3D& pose, const std::array<double, 3>& scale) const
	{
		const auto& R = pose.getRotationMatrix();
		const std::array<double, 3> t = {pose.x(), pose.y(), pose.z()};

		Affine3 out;
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				double v = (j == 3) ? m[i][3] : 0.0;
				for (int k = 0; k < 3; k++)
				{
					v += m[i][k] * (j == 3 ? t[k] : R(k, j) * scale[j]);
				}
				out.m[i][j] = v;
			}
		}
		return out;
	}
};

/** Occupied cells of a 2D grid, filled with the parts of the scene geometry
 * within a height band [zMin, zMax]. */
class SliceGrid
{
   public:
	SliceGrid(double resolution, double zMin, double zMax)
		: resolution_(resolution), zMin_(zMin), zMax_(zMax)
	{
		// A zero-thickness band (a single slice) still needs some thickness
		// for the clipped triangles to have area:
		constexpr double minHalfThickness = 1e-4;
		if (zMax_ - zMin_ < 2 * minHalfThickness)
		{
			const double zMid = 0.5 * (zMin_ + zMax_);
			zMin_ = zMid - minHalfThickness;
			zMax_ = zMid + minHalfThickness;
		}
	}

	void addPoint(const TPoint3D& p)
	{
		if (p.z >= zMin_ && p.z <= zMax_)
		{
			mark(p.x, p.y);
		}
	}

	/** Marks the cells covered by the part of the triangle within the band. */
	void addTriangle(const TPoint3D& p0, const TPoint3D& p1, const TPoint3D& p2)
	{
		const double triZMin = std::min({p0.z, p1.z, p2.z});
		const double triZMax = std::max({p0.z, p1.z, p2.z});
		if (triZMax < zMin_ || triZMin > zMax_)
		{
			return;
		}

		std::vector<TPoint3D> poly = {p0, p1, p2};
		poly = clipZ(poly, zMin_, true);
		poly = clipZ(poly, zMax_, false);

		// Fan triangulation of the clipped convex polygon:
		for (size_t i = 1; i + 1 < poly.size(); i++)
		{
			sampleTriangle(poly[0], poly[i], poly[i + 1]);
		}
	}

	void markCell(double x, double y) { mark(x, y); }

	const std::set<std::pair<int, int>>& cells() const { return cells_; }

	TPoint3D cellCenter(const std::pair<int, int>& c) const
	{
		return {(c.first + 0.5) * resolution_, (c.second + 0.5) * resolution_, 0.0};
	}

   private:
	double resolution_;
	double zMin_;
	double zMax_;
	std::set<std::pair<int, int>> cells_;

	void mark(double x, double y)
	{
		cells_.emplace(
			static_cast<int>(std::floor(x / resolution_)),
			static_cast<int>(std::floor(y / resolution_)));
	}

	/** Sutherland-Hodgman clipping of a convex polygon by the plane z=zLim,
	 * keeping the side above (keepAbove) or below it. */
	static std::vector<TPoint3D> clipZ(
		const std::vector<TPoint3D>& poly, double zLim, bool keepAbove)
	{
		std::vector<TPoint3D> out;
		const size_t n = poly.size();
		for (size_t i = 0; i < n; i++)
		{
			const auto& a = poly[i];
			const auto& b = poly[(i + 1) % n];
			const bool aIn = keepAbove ? (a.z >= zLim) : (a.z <= zLim);
			const bool bIn = keepAbove ? (b.z >= zLim) : (b.z <= zLim);
			if (aIn)
			{
				out.push_back(a);
			}
			if (aIn != bIn)
			{
				const double s = (zLim - a.z) / (b.z - a.z);
				out.emplace_back(a.x + s * (b.x - a.x), a.y + s * (b.y - a.y), zLim);
			}
		}
		return out;
	}

	/** Marks the cells of points sampled over the triangle, finer than the
	 * grid resolution so no cell under it is missed. */
	void sampleTriangle(const TPoint3D& p0, const TPoint3D& p1, const TPoint3D& p2)
	{
		const double maxEdge = std::max({p0.distanceTo(p1), p1.distanceTo(p2), p2.distanceTo(p0)});
		const int n = std::max(1, static_cast<int>(std::ceil(maxEdge / (0.5 * resolution_))));
		for (int i = 0; i <= n; i++)
		{
			for (int j = 0; j <= n - i; j++)
			{
				const double u = static_cast<double>(i) / n;
				const double v = static_cast<double>(j) / n;
				mark(
					p0.x + u * (p1.x - p0.x) + v * (p2.x - p0.x),
					p0.y + u * (p1.y - p0.y) + v * (p2.y - p0.y));
			}
		}
	}
};

/** Adds the geometry of a visual object (and its children) to the grid. */
void sliceObject(const mrpt::viz::CVisualObject& obj, const Affine3& parentXf, SliceGrid& grid)
{
	const auto ps = obj.getPoseAndScale();
	if (!ps.visible)
	{
		return;	 // sensors do not see it either
	}
	const Affine3 xf = parentXf.compose(ps.pose, {ps.scaleX, ps.scaleY, ps.scaleZ});

	// Make sure the CPU-side buffers are up to date:
	obj.updateBuffers();

	const auto lambdaTriangles = [&](const auto& tris)
	{
		for (const auto& t : tris)
		{
			grid.addTriangle(xf.apply(t.vertex(0)), xf.apply(t.vertex(1)), xf.apply(t.vertex(2)));
		}
	};

	if (const auto* o = dynamic_cast<const mrpt::viz::VisualObjectParams_Triangles*>(&obj); o)
	{
		auto lck = mrpt::lockHelper(o->shaderTrianglesBufferMutex().data);
		lambdaTriangles(o->shaderTrianglesBuffer());
	}
	if (const auto* o = dynamic_cast<const mrpt::viz::VisualObjectParams_TexturedTriangles*>(&obj);
		o)
	{
		auto lck = mrpt::lockHelper(o->shaderTexturedTrianglesBufferMutex().data);
		lambdaTriangles(o->shaderTexturedTrianglesBuffer());
	}
	if (const auto* o = dynamic_cast<const mrpt::viz::VisualObjectParams_Points*>(&obj); o)
	{
		auto lck = mrpt::lockHelper(o->shaderPointsBuffersMutex().data);
		for (const auto& pt : o->shaderPointsVertexPointBuffer())
		{
			grid.addPoint(xf.apply(pt));
		}
	}
	// Composite objects (including CAssimpModel meshes):
	if (const auto* o = dynamic_cast<const mrpt::viz::CSetOfObjects*>(&obj); o)
	{
		for (const auto& child : *o)
		{
			if (child)
			{
				sliceObject(*child, xf, grid);
			}
		}
	}
}

/** Adds the occupied cells of an occupancy grid world element */
void sliceOccupancyGrid(const mvsim::OccupancyGridMap& e, SliceGrid& grid)
{
	const auto& g = e.getOccGrid();
	for (unsigned int cy = 0; cy < g.getSizeY(); cy++)
	{
		for (unsigned int cx = 0; cx < g.getSizeX(); cx++)
		{
			if (g.getCell(cx, cy) < 0.5f)
			{
				grid.markCell(g.idx2x(cx), g.idx2y(cy));
			}
		}
	}
}

int printSliceUsage()
{
	fprintf(
		stdout,
		R"XXX(Usage: mvsim slice <WORLD_MODEL.xml> -o <OUTPUT.txt> [options]

Loads a world and writes the 2D occupied cells of its obstacles within a
height band (or at a single height), as seen by sensors at t=0, e.g. to build
a static obstacle map for navigation. Vehicles, actors and animated blocks are
skipped, since they move.

Output: a text file with one "x y z" line per occupied cell center (world
frame; z is --slice-z for all cells).

Available options:
 -o, --output <FILE>          Output .txt file (mandatory).
 --slice-z <0.5>              Height of the slice [m].
 --slice-z-max <z>            If given, the cells of all geometry between
                              --slice-z and this height [m] are reported.
 --slice-resolution <0.05>    Cell size [m].
 --slice-static-only          Skip blocks that are not <static> (movable).
 -v, --verbosity              Set verbosity level: DEBUG, INFO (default), WARN, ERROR
)XXX");
	return 0;
}

}  // namespace

int commandSlice()
{
	const auto& unlabeledArgs = cli->argCmd;
	const bool badArgs = unlabeledArgs.size() != 2 || cli->argOutput.empty();
	if (cli->argHelp || badArgs)
	{
		printSliceUsage();
		return badArgs && !cli->argHelp ? 1 : 0;
	}

	const double zMin = cli->argSliceZ;
	const double zMax = cli->argSliceZMax.value_or(cli->argSliceZ);
	ASSERT_GE_(zMax, zMin);
	ASSERT_GT_(cli->argSliceResolution, 0.0);

	mvsim::World world;
	world.setMinLoggingLevel(
		mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(cli->argVerbosity));
	world.headless(true);
	world.load_from_XML_file(unlabeledArgs.at(1));

	// Build the scene sensors see, only with the entities that do not move:
	mrpt::viz::Scene viz;
	mrpt::viz::Scene physical;
	SliceGrid grid(cli->argSliceResolution, zMin, zMax);

	for (const auto& e : world.getListOfWorldElements())
	{
		if (const auto* og = dynamic_cast<const mvsim::OccupancyGridMap*>(e.get()); og)
		{
			sliceOccupancyGrid(*og, grid);
			continue;
		}
		e->guiUpdate(viz, physical);
	}
	size_t numBlocks = 0;
	for (const auto& [name, b] : world.getListOfBlocks())
	{
		if (b->isAnimated() || (cli->argSliceStaticOnly && !b->isStatic()))
		{
			continue;
		}
		b->guiUpdate(viz, physical);
		numBlocks++;
	}

	// Geometry is rendered with an offset in large (e.g. georeferenced) worlds:
	const auto renderOffset = world.worldRenderOffset();
	Affine3 xf;
	for (int i = 0; i < 3; i++)
	{
		xf.m[i][3] = -renderOffset[i];
	}
	for (const auto& obj : *physical.getViewport())
	{
		if (obj)
		{
			sliceObject(*obj, xf, grid);
		}
	}

	std::ofstream f(cli->argOutput);
	if (!f.is_open())
	{
		std::cerr << "Error: cannot write to: " << cli->argOutput << "\n";
		return 1;
	}
	f << "# x y z: occupied cells of mvsim world '" << unlabeledArgs.at(1) << "' for z in [" << zMin
	  << ", " << zMax << "], resolution " << cli->argSliceResolution << "\n";
	for (const auto& c : grid.cells())
	{
		const auto p = grid.cellCenter(c);
		f << p.x << " " << p.y << " " << zMin << "\n";
	}

	std::cout << "Wrote " << grid.cells().size() << " occupied cells (" << numBlocks << " blocks, "
			  << world.getListOfWorldElements().size() << " world elements) to: " << cli->argOutput
			  << "\n";
	return 0;
}
