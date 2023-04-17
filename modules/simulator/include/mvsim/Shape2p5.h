/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/containers/CDynamicGrid.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/opengl/TTriangle.h>

#include <cstdint>
#include <optional>
#include <vector>

namespace mvsim
{
/** A "2.5" shape: a 2D polygon + a [zMin,zMax] height range
 *
 *  The actual 2.5 shape is evaluated the first time getContour() is called,
 *  given all former calls to buildAddPoint() or buildAddTriangle() and a
 * ray-tracing algorithm.
 *
 */
class Shape2p5
{
   public:
	Shape2p5() = default;

	void buildInit(
		const mrpt::math::TPoint2Df& bbMin, const mrpt::math::TPoint2Df& bbMax,
		int numCells = 100);
	void buildAddPoint(const mrpt::math::TPoint3Df& pt);
	void buildAddTriangle(const mrpt::opengl::TTriangle& t);

	const mrpt::math::TPolygon2D& getContour() const;

	double volume() const;

	void mergeWith(const Shape2p5& s);

	void setShapeManual(
		const mrpt::math::TPolygon2D& contour, const float zMin,
		const float zMax);

	float zMin() const { return zMin_; }
	float zMax() const { return zMax_; }

	void clipZMin(float v);
	void clipZMax(float v);

   private:
	mutable std::optional<mrpt::math::TPolygon2D> contour_;
	mutable float zMin_ = 0, zMax_ = 0;

	class SimpleOccGrid : public mrpt::containers::CDynamicGrid<uint8_t>
	{
	   public:
		template <typename... Args>
		SimpleOccGrid(Args&&... args)
			: mrpt::containers::CDynamicGrid<uint8_t>(
				  std::forward<Args>(args)...)
		{
		}

		// Used to debug (save grid to txt file)
		float cell2float(const uint8_t& v) const override { return v; }
	};

	mutable std::optional<SimpleOccGrid> grid_;

	/// Computes contour_ from the contents in grid_
	void computeShape() const;

	void internalGridFilterSpurious() const;
	void internalGridFloodFill() const;
	mrpt::math::TPolygon2D internalGridContour() const;
	mrpt::math::TPolygon2D internalPrunePolygon(
		const mrpt::math::TPolygon2D& poly) const;

	struct RemovalCandidate
	{
		double loss = 0;
		mrpt::math::TPolygon2D next;
	};

	std::optional<RemovalCandidate> lossOfRemovingVertex(
		size_t i, const mrpt::math::TPolygon2D& p, bool allowApproxEdges) const;

	void debugSaveGridTo3DSceneFile(
		const mrpt::math::TPolygon2D& rawGridContour,
		const std::string& debugStr = {}) const;
};

}  // namespace mvsim
