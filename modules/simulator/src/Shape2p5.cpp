/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <box2d/b2_settings.h>	// b2_maxPolygonVertices
#include <mrpt/containers/yaml.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/math/TLine2D.h>
#include <mrpt/math/TObject2D.h>
#include <mrpt/math/geometry.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfTriangles.h>
#include <mrpt/opengl/CTexturedPlane.h>
#include <mrpt/opengl/stock_objects.h>
#include <mvsim/Shape2p5.h>

#include <cmath>
#include <iostream>
#include <queue>

// Uncomment only for development debugging
// #define DEBUG_DUMP_ALL_TEMPORARY_GRIDS
// #define DEBUG_DUMP_TRIANGLES

using namespace mvsim;

using mrpt::math::TPoint2Df;
using mrpt::math::TPoint3Df;

constexpr uint8_t CELL_UNDEFINED = 0x80;
constexpr uint8_t CELL_OCCUPIED = 0x00;
constexpr uint8_t CELL_FREE = 0xff;
constexpr uint8_t CELL_VISITED = 0x40;

double Shape2p5::volume() const
{
	return std::abs(mrpt::math::signedArea(getContour())) * std::abs(zMin_ - zMax_);
}

void Shape2p5::mergeWith(const Shape2p5& s)
{
	using mrpt::math::TPoint3Df;

	const auto& ca = this->getContour();
	const auto& cb = s.getContour();

	// gross BB:
	auto bb = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
	for (const auto& p : ca) bb.updateWithPoint(TPoint3Df(p.x, p.y, zMin()));
	for (const auto& p : cb) bb.updateWithPoint(TPoint3Df(p.x, p.y, s.zMin()));

	bb.updateWithPoint(TPoint3Df(bb.min.x, bb.min.y, s.zMax()));
	bb.updateWithPoint(TPoint3Df(bb.max.x, bb.max.y, this->zMax()));

	// convert to triangles:
	Shape2p5 newShape;
	newShape.buildInit({bb.min.x, bb.min.y}, {bb.max.x, bb.max.y});
	for (size_t i = 0; i < ca.size(); i++)
	{
		size_t im1 = i == 0 ? (ca.size() - 1) : i - 1;
		const auto p0 = ca.at(im1);
		const auto p1 = ca.at(i);

		mrpt::opengl::TTriangle t;
		t.vertex(0) = TPoint3Df(p0.x, p0.y, bb.min.z);
		t.vertex(1) = TPoint3Df(p1.x, p1.y, bb.min.z);
		t.vertex(2) = TPoint3Df(p1.x, p1.y, bb.max.z);
		newShape.buildAddTriangle(t);
	}
	for (size_t i = 0; i < cb.size(); i++)
	{
		size_t im1 = i == 0 ? (cb.size() - 1) : i - 1;
		const auto p0 = cb.at(im1);
		const auto p1 = cb.at(i);

		mrpt::opengl::TTriangle t;
		t.vertex(0) = TPoint3Df(p0.x, p0.y, bb.min.z);
		t.vertex(1) = TPoint3Df(p1.x, p1.y, bb.min.z);
		t.vertex(2) = TPoint3Df(p1.x, p1.y, bb.max.z);
		newShape.buildAddTriangle(t);
	}

	// re-generate again:
	*this = newShape;
}

const mrpt::math::TPolygon2D& Shape2p5::getContour() const
{
	if (!contour_) computeShape();
	return *contour_;
}

// For debugging only:
#ifdef DEBUG_DUMP_TRIANGLES
static auto glDebugTriangles = mrpt::opengl::CSetOfTriangles::Create();
#endif

void Shape2p5::buildInit(
	const mrpt::math::TPoint2Df& bbMin, const mrpt::math::TPoint2Df& bbMax, int numCells)
{
	contour_.reset();  // start from scratch

	// grid resolution:
	const float r = (bbMax - bbMin).norm() / numCells;
	// border to ensure we have a free row/col all around the shape
	const float b = r * 1.5f;
	grid_.emplace(bbMin.x - b, bbMax.x + b, bbMin.y - b, bbMax.y + b, r);

	zMin_ = std::numeric_limits<float>::max();
	zMax_ = -std::numeric_limits<float>::max();
	grid_->fill(CELL_UNDEFINED);

#ifdef DEBUG_DUMP_TRIANGLES
	glDebugTriangles->clearTriangles();
#endif
}

void Shape2p5::buildAddPoint(const mrpt::math::TPoint3Df& pt)
{
	mrpt::keep_max(zMax_, pt.z);
	mrpt::keep_min(zMin_, pt.z);
	uint8_t* c = grid_->cellByPos(pt.x, pt.y);
	ASSERT_(c);
	*c = CELL_OCCUPIED;
}

void Shape2p5::buildAddTriangle(const mrpt::opengl::TTriangle& t)
{
	const float step = grid_->getResolution();

	for (int i0 = 0; i0 < 3; i0++)
	{
		const int i1 = (i0 + 1) % 3;
		const auto& v0 = t.vertex(i0);
		const auto& v1 = t.vertex(i1);

		const auto v01 = v1 - v0;

		const int nSteps = static_cast<int>(std::ceil(v01.norm() / step));

		mrpt::math::TPoint3Df p = v0;
		const mrpt::math::TVector3Df Ap = v01 * (1.0f / nSteps);

		for (int s = 0; s < nSteps; s++, p += Ap)
		{
			uint8_t* c = grid_->cellByPos(p.x, p.y);
			if (!c) continue;

			*c = CELL_OCCUPIED;
			mrpt::keep_max(zMax_, p.z);
			mrpt::keep_min(zMin_, p.z);
		}
	}

#ifdef DEBUG_DUMP_TRIANGLES
	glDebugTriangles->insertTriangle(t);
#endif
}

// Computes contour_ from the contents in grid_
void Shape2p5::computeShape() const
{
	ASSERT_(grid_);
	ASSERT_(!contour_);

#ifdef DEBUG_DUMP_ALL_TEMPORARY_GRIDS
	// Debug save initial grid:
	debugSaveGridTo3DSceneFile({});
#endif

	// 0) Filter spurious occupied cells, due to (rare) lost points in the 3D
	// model:
	internalGridFilterSpurious();

	// 1) Flood-fill the grid with "FREE color" to allow detecting the outer
	// shape:
	internalGridFloodFill();

	// 2) Detect the outer contour with full grid resolution:
	const mrpt::math::TPolygon2D rawGridContour = internalGridContour();

	// 3) convex hull:
	// Eventually, b2Box library will use convex hull anyway, so let's use it
	// here too:

	// 4) Polygon pruning until edge count is <= b2_maxPolygonVertices
	const auto finalPoly = internalPrunePolygon(rawGridContour);

	// 5) Save result if output structure:
	contour_.emplace(finalPoly);

// DEBUG:
#ifdef DEBUG_DUMP_ALL_TEMPORARY_GRIDS
	debugSaveGridTo3DSceneFile(rawGridContour);
#endif

	grid_.reset();

#ifdef DEBUG_DUMP_TRIANGLES
	{
		static int cnt = 0;
		mrpt::opengl::COpenGLScene scene;
		scene.insert(glDebugTriangles);
		scene.saveToFile(mrpt::format("debug_shape2p5_triangles_%04i.3Dscene", cnt++));
	}
#endif
}

void Shape2p5::setShapeManual(
	const mrpt::math::TPolygon2D& contour, const float zMin, const float zMax)
{
	grid_.reset();
	contour_ = contour;
	zMin_ = zMin;
	zMax_ = zMax;
}

void Shape2p5::internalGridFilterSpurious() const
{
	ASSERT_(grid_);

	const int cxMax = grid_->getSizeX() - 1;
	const int cyMax = grid_->getSizeY() - 1;

	for (int cx = 1; cx < cxMax; cx++)
	{
		for (int cy = 1; cy < cyMax; cy++)
		{
			auto* thisCell = grid_->cellByIndex(cx, cy);
			if (*thisCell != CELL_OCCUPIED) continue;
			// it's occupied:
			// reset to unknown if no other neighbors is occupied:
			bool anyNN = (*grid_->cellByIndex(cx - 1, cy - 1) == CELL_OCCUPIED) ||
						 (*grid_->cellByIndex(cx - 1, cy + 0) == CELL_OCCUPIED) ||
						 (*grid_->cellByIndex(cx - 1, cy + 1) == CELL_OCCUPIED) ||
						 (*grid_->cellByIndex(cx + 0, cy - 1) == CELL_OCCUPIED) ||
						 (*grid_->cellByIndex(cx + 0, cy + 1) == CELL_OCCUPIED) ||
						 (*grid_->cellByIndex(cx + 1, cy - 1) == CELL_OCCUPIED) ||
						 (*grid_->cellByIndex(cx + 1, cy + 0) == CELL_OCCUPIED) ||
						 (*grid_->cellByIndex(cx + 1, cy + 1) == CELL_OCCUPIED);

			if (!anyNN) *thisCell = CELL_UNDEFINED;
		}
	}
}

void Shape2p5::internalGridFloodFill() const
{
	ASSERT_(grid_);

	// Algorithm:
	// Heckbert, Paul S (1990). "IV.10: A Seed Fill Algorithm". In Glassner,
	// Andrew S (ed.). Graphics Gems. Academic Press. pp. 275–277.
	// https://en.wikipedia.org/wiki/Flood_fill

	const int cxMax = grid_->getSizeX() - 1;
	const int cyMax = grid_->getSizeY() - 1;

	const auto Inside = [&](int x, int y)
	{
		if (x < 0 || y < 0) return false;
		if (x > cxMax || y > cyMax) return false;
		uint8_t* c = grid_->cellByIndex(x, y);
		if (!c) return false;

		return *c == CELL_UNDEFINED;
	};

	const auto Set = [&](int x, int y)
	{
		if (x < 0 || y < 0)
		{
			return;
		}
		if (x > cxMax || y > cyMax)
		{
			return;
		}
		uint8_t* c = grid_->cellByIndex(x, y);
		if (!c)
		{
			return;
		}
		*c = CELL_FREE;
	};

	const int x0 = 0, y0 = 0;  // start pixel for flood fill.

	/*
	fn fill(x, y):
		if not Inside(x, y) then return
		let s = new empty stack or queue
		Add (x, y) to s
		while s is not empty:
			Remove an (x, y) from s
			let lx = x
			while Inside(lx - 1, y):
				Set(lx - 1, y)
				lx = lx - 1
			while Inside(x, y):
				Set(x, y)
				x = x + 1
		  scan(lx, x - 1, y + 1, s)
		  scan(lx, x - 1, y - 1, s)

	fn scan(lx, rx, y, s):
		let span_added = false
		for x in lx .. rx:
			if not Inside(x, y):
				span_added = false
			else if not span_added:
				Add (x, y) to s
				span_added = true
	*/

	if (!Inside(x0, y0))
	{
		return;
	}
	struct Coord
	{
		Coord() = default;
		Coord(int X, int Y) : x_(X), y_(Y) {}

		int x_ = 0, y_ = 0;
	};

	std::queue<Coord> s;

	const auto lambdaScan = [&s, &Inside](int lx, int rx, int y)
	{
		bool spanAdded = false;
		for (int x = lx; x <= rx; x++)
		{
			if (!Inside(x, y))
			{
				spanAdded = false;
			}
			else if (!spanAdded)
			{
				s.emplace(x, y);
				spanAdded = true;
			}
		}
	};

	s.emplace(x0, y0);
	while (!s.empty())
	{
		auto [x, y] = s.front();
		s.pop();
		int lx = x;
		while (Inside(lx - 1, y))
		{
			Set(lx - 1, y);
			lx--;
		}
		while (Inside(x, y))
		{
			Set(x, y);
			x++;
		}
		lambdaScan(lx, x - 1, y + 1);
		lambdaScan(lx, x - 1, y - 1);
	}
}

// Detects the outter polygon of the grid, after having been flood filled.
mrpt::math::TPolygon2D Shape2p5::internalGridContour() const
{
	ASSERT_(grid_);

	mrpt::math::TPolygon2D p;

	const int nx = grid_->getSizeX();
	const int ny = grid_->getSizeY();

	const std::vector<std::pair<int, int>> dirs = {
		// first, straight directions (important!)
		{+1, 0},
		{-1, 0},
		{0, +1},
		{0, -1},
		// second, diagonals:
		{+1, +1},
		{+1, -1},
		{-1, +1},
		{-1, -1},
	};

	auto lambdaCellIsBorderSimple = [&](int cx, int cy)
	{
		auto* c = grid_->cellByIndex(cx, cy);
		if (!c) return false;

		if (*c != CELL_OCCUPIED) return false;

		// check 4 neighbors:
		if (auto* cS = grid_->cellByIndex(cx, cy - 1); cS && *cS == CELL_FREE) return true;
		if (auto* cN = grid_->cellByIndex(cx, cy + 1); cN && *cN == CELL_FREE) return true;
		if (auto* cE = grid_->cellByIndex(cx + 1, cy); cE && *cE == CELL_FREE) return true;
		if (auto* cW = grid_->cellByIndex(cx - 1, cy); cW && *cW == CELL_FREE) return true;

		return false;
	};

	auto lambdaStillHasUnexploredNeighbors = [&](int cx, int cy)
	{
		// precondition: (cx,cy) is VISITED.
		// We check 8-neighbors:

		for (const auto& dir : dirs)
		{
			const int ix = dir.first, iy = dir.second;
			const bool isBorder = lambdaCellIsBorderSimple(cx + ix, cy + iy);
			if (isBorder) return true;
		}
		return false;
	};

	auto lambdaCellIsBorder = [&](int cx, int cy, bool considerRevisits)
	{
		auto* c = grid_->cellByIndex(cx, cy);
		if (!c) return false;

		if (*c == CELL_UNDEFINED) return false;
		if (*c == CELL_FREE) return false;
		if (*c == CELL_VISITED)
		{
			// only consider it if it still has possible free ways to move
			// around:
			if (considerRevisits && lambdaStillHasUnexploredNeighbors(cx, cy))
				return true;
			else
				return false;
		}

		// check 4 neighbors:
		if (auto* cS = grid_->cellByIndex(cx, cy - 1); cS && *cS == CELL_FREE) return true;
		if (auto* cN = grid_->cellByIndex(cx, cy + 1); cN && *cN == CELL_FREE) return true;
		if (auto* cE = grid_->cellByIndex(cx + 1, cy); cE && *cE == CELL_FREE) return true;
		if (auto* cW = grid_->cellByIndex(cx - 1, cy); cW && *cW == CELL_FREE) return true;

		return false;
	};

	// 1) Look for the first CELL_OCCUPIED cell:
	int cx = 0, cy = 0;
	while (*grid_->cellByIndex(cx, cy) != CELL_OCCUPIED)
	{
		cx++;
		if (cx >= nx)
		{
			cx = 0;
			cy++;
			ASSERT_(cy < ny);
		}
	}

	// 2) Iterate:
	//    - mark current cell as CELL_VISITED, add to polygon.
	//    - Look in 8 neighbors for a CELL_OCCUPIED with a CELL_FREE cell in
	//    one of its 4 main directions.
	for (;;)
	{
		auto* c = grid_->cellByIndex(cx, cy);
		ASSERT_(c);
		*c = CELL_VISITED;

		// save into polygon too:
		p.emplace_back(grid_->idx2x(cx), grid_->idx2y(cy));

		bool cellDone = false;

		for (int pass = 0; pass < 2 && !cellDone; pass++)
		{
			for (const auto& dir : dirs)
			{
				const int ix = dir.first, iy = dir.second;
				const bool isBorder = lambdaCellIsBorder(cx + ix, cy + iy, pass == 1);

				if (isBorder)
				{
#ifdef DEBUG_DUMP_ALL_TEMPORARY_GRIDS
					debugSaveGridTo3DSceneFile(p);
#endif
					// Save for next iter:
					cellDone = true;
					cx = cx + ix;
					cy = cy + iy;
					break;
				}
			}
		}
		if (!cellDone) break;
	}

	return p;
}

void Shape2p5::debugSaveGridTo3DSceneFile(
	const mrpt::math::TPolygon2D& rawGridContour, const std::string& debugStr) const
{
	mrpt::opengl::COpenGLScene scene;

	auto glGrid = mrpt::opengl::CTexturedPlane::Create();
	glGrid->setPlaneCorners(grid_->getXMin(), grid_->getXMax(), grid_->getYMin(), grid_->getYMax());

	mrpt::math::CMatrixDouble mat;
	grid_->getAsMatrix(mat);

	mrpt::img::CImage im;
	im.setFromMatrix(mat, false /* matrix is [0,255]*/);

	glGrid->assignImage(im);

	scene.insert(mrpt::opengl::stock_objects::CornerXYZSimple());
	scene.insert(glGrid);

	auto lambdaRenderPoly =
		[&scene](const mrpt::math::TPolygon2D& p, const mrpt::img::TColor& color, double z)
	{
		auto glPts = mrpt::opengl::CPointCloud::Create();
		auto glPoly = mrpt::opengl::CSetOfLines::Create();
		glPoly->setColor_u8(color);
		glPts->setColor_u8(color);
		glPts->setPointSize(4.0f);
		const auto N = p.size();
		for (size_t j = 0; j < N; j++)
		{
			const size_t j1 = (j + 1) % N;
			const auto& p0 = p.at(j);
			const auto& p1 = p.at(j1);
			glPoly->appendLine(p0.x, p0.y, z + 1e-4 * j, p1.x, p1.y, z + 1e-4 * (j + 1));
			glPts->insertPoint(p0.x, p0.y, z + 1e-4 * j);
		}
		scene.insert(glPoly);
		scene.insert(glPts);
	};

	lambdaRenderPoly(*contour_, {0xff, 0x00, 0x00}, 0.10);
	lambdaRenderPoly(rawGridContour, {0x00, 0xff, 0x00}, 0.05);

	if (!debugStr.empty()) scene.getViewport()->addTextMessage(5, 5, debugStr);

	static int i = 0;
	scene.saveToFile(mrpt::format("collision_grid_%05i.3Dscene", i++));
}

std::optional<Shape2p5::RemovalCandidate> Shape2p5::lossOfRemovingVertex(
	size_t i, const mrpt::math::TPolygon2D& p, bool allowApproxEdges) const
{
	// 1st: check if removing that vertex leads to edges crossing
	// CELL_UNDEFINED or CELL_OCCUPIED cells:

	size_t im1 = i > 0 ? i - 1 : (p.size() - 1);
	size_t ip1 = i == (p.size() - 1) ? 0 : i + 1;

	const auto& pt_im1 = p[im1];
	const auto& pt_ip1 = p[ip1];
	const auto delta = pt_ip1 - pt_im1;
	const size_t nSteps = static_cast<size_t>(ceil(delta.norm() / grid_->getResolution()));
	const auto d = delta * (1.0 / nSteps);
	for (size_t k = 0; k < nSteps; k++)
	{
		const auto pt = pt_im1 + d * k;
		const auto* c = grid_->cellByPos(pt.x, pt.y);
		if (!c) return {};	// should never happen (!)

		if (!allowApproxEdges)
		{
			// removing this vertex leads to unacceptable approximation:
			if (*c == CELL_UNDEFINED || *c == CELL_OCCUPIED) return {};
		}
	}

	// ok, removing the vertex is ok.
	// now, let's quantify the increase in area ("loss"):
	Shape2p5::RemovalCandidate rc;
	rc.next = p;
	rc.next.erase(rc.next.begin() + i);

	const double originalArea = std::abs(mrpt::math::signedArea(p));
	const double newArea = std::abs(mrpt::math::signedArea(rc.next));
	rc.loss = newArea - originalArea;

	if (allowApproxEdges) rc.loss = -rc.loss;

	return rc;
}

mrpt::math::TPolygon2D Shape2p5::internalPrunePolygon(const mrpt::math::TPolygon2D& poly) const
{
	using namespace std::string_literals;

	mrpt::math::TPolygon2D p = poly;

	// Algorithm:
	// Pass #1: go thru all vertices, and pick the one that minimizes
	// the increase of polygon area while not crossing through any grid cell
	// that is either CELL_UNDEFINED or CELL_OCCUPIED.
	// Pass #2: idem, but allow crossing cells.
	for (int pass = 0; pass < 2; pass++)
	{
		while (p.size() > b2_maxPolygonVertices)
		{
			std::optional<RemovalCandidate> best;

			for (size_t i = 0; i < p.size(); i++)
			{
				std::optional<RemovalCandidate> rc = lossOfRemovingVertex(i, p, pass == 1);
				if (rc && (!best || rc->loss < best->loss)) best = *rc;
			}

			if (!best) break;  // No more vertices found to remove

			p = best->next;

#ifdef DEBUG_DUMP_ALL_TEMPORARY_GRIDS
			debugSaveGridTo3DSceneFile(p, mrpt::format("pass #%i loss=%f", pass, best->loss));
#endif
		}
	}

	return p;
}

void Shape2p5::clipZMin(float v)
{
	if (zMin_ < v) zMin_ = v;
}

void Shape2p5::clipZMax(float v)
{
	if (zMax_ > v) zMax_ = v;
}

std::string Shape2p5::asString() const
{
	std::stringstream s;
	s << getContour().asYAML();
	return s.str();
}
