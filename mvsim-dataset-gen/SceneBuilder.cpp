/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "SceneBuilder.h"

#include <mrpt/core/exceptions.h>
#include <mvsim/Block.h>
#include <mvsim/WorldElements/ElevationMap.h>
#include <mvsim/WorldElements/GroundGrid.h>
#include <mvsim/WorldElements/HorizontalPlane.h>
#include <mvsim/WorldElements/OccupancyGridMap.h>
#include <mvsim/WorldElements/PointCloud.h>
#include <mvsim/WorldElements/PropertyRegion.h>
#include <mvsim/WorldElements/SkyBox.h>
#include <mvsim/WorldElements/VerticalPlane.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

using namespace mvsim_dataset_gen;
using mrpt::math::TPoint2D;
using mrpt::math::TPoint3D;

namespace
{
/** Transforms a local-frame 2D point (z=0) into world XY through `pose`.
 * Valid for blocks/walls, which never pitch or roll (2.5D). */
TPoint2D toWorldXY(const mrpt::poses::CPose3D& pose, double lx, double ly)
{
	const TPoint3D p = pose.composePoint(TPoint3D(lx, ly, 0));
	return {p.x, p.y};
}

void warnOrThrow(const SceneBuilder::Options& opts, const std::string& msg)
{
	if (opts.allowUnsupported)
	{
		std::cerr << "[SceneBuilder] WARNING: " << msg << " -- skipped.\n";
		return;
	}
	THROW_EXCEPTION(msg + " (pass --allow-unsupported to skip it instead)");
}

void addBlock(mvsim::rt::RayScene& scene, const mvsim::Block& block, const SceneBuilder::Options&)
{
	using mvsim::GeometryType;
	const auto pose = block.getCPose3D();
	const std::string name = block.getName();

	switch (block.geometryType())
	{
		case GeometryType::Cylinder:
		{
			mvsim::rt::Cylinder cyl;
			cyl.pose = pose;
			cyl.radius = block.geometryRadius();
			cyl.length = block.geometryLength();
			cyl.cappedBottom = true;
			cyl.cappedTop = true;
			scene.addPrimitive(mvsim::rt::Primitive{cyl, name});
			return;
		}
		case GeometryType::Sphere:
		{
			mvsim::rt::Sphere sp;
			sp.center = {pose.x(), pose.y(), pose.z()};
			sp.radius = block.geometryRadius();
			scene.addPrimitive(mvsim::rt::Primitive{sp, name});
			return;
		}
		case GeometryType::Box:
		{
			const double lx = block.geometryLx(), ly = block.geometryLy(), lz = block.geometryLz();
			mvsim::rt::Prism prism;
			prism.contour = mrpt::math::TPolygon2D(
				{toWorldXY(pose, 0, 0), toWorldXY(pose, lx, 0), toWorldXY(pose, lx, ly),
				 toWorldXY(pose, 0, ly)});
			prism.zMin = pose.z();
			prism.zMax = pose.z() + lz;
			scene.addPrimitive(mvsim::rt::Primitive{prism, name});
			return;
		}
		case GeometryType::Ramp:
		{
			// Exactly mirrors the 6 triangles built in
			// Block::internal_parseGeometry() for GeometryType::Ramp.
			const double lx = block.geometryLx(), ly = block.geometryLy(), lz = block.geometryLz();
			const auto w = [&](double x, double y, double z)
			{ return pose.composePoint(TPoint3D(x, y, z)); };
			const TPoint3D p0 = w(0, -ly * 0.5, 0);
			const TPoint3D p1 = w(lx, -ly * 0.5, lz);
			const TPoint3D p2 = w(0, ly * 0.5, 0);
			const TPoint3D p3 = w(lx, ly * 0.5, lz);
			const TPoint3D p4 = w(lx, -ly * 0.5, 0);
			const TPoint3D p5 = w(lx, ly * 0.5, 0);
			for (const auto& tri :
				 {std::array<TPoint3D, 3>{p0, p1, p2}, std::array<TPoint3D, 3>{p2, p1, p3},
				  std::array<TPoint3D, 3>{p0, p4, p1}, std::array<TPoint3D, 3>{p5, p3, p2},
				  std::array<TPoint3D, 3>{p4, p5, p1}, std::array<TPoint3D, 3>{p3, p1, p5}})
			{
				scene.addPrimitive(
					mvsim::rt::Primitive{mvsim::rt::Triangle{tri[0], tri[1], tri[2]}, name});
			}
			return;
		}
		case GeometryType::SemiCylinderBump:
		{
			// A half-ellipse cross section (width ly, height lz) extruded
			// along local X in [0,lx], matching the same
			// z(y) = lz*sqrt(max(0,1-(2y/ly)^2)) profile used by
			// Block::getElevationAt() for this geometry type. mvsim's own
			// visual model reaches this shape via a scaled+rotated
			// CCylinder tied to <shape_from_visual/>'s bounding box, whose
			// exact local-X placement is not otherwise pinned down; this
			// is a faithful, self-consistent reconstruction of the same
			// profile rather than a byte-exact replay of that pipeline.
			const double lx = block.geometryLx(), ly = block.geometryLy(), lz = block.geometryLz();
			constexpr int kSlices = 16;
			const auto profile = [&](int i)
			{
				const double y = -ly * 0.5 + ly * i / kSlices;
				const double f = std::sqrt(std::max(0.0, 1.0 - std::pow(2 * y / ly, 2)));
				return std::pair<double, double>{y, lz * f};
			};
			for (int i = 0; i < kSlices; i++)
			{
				const auto [y0, z0] = profile(i);
				const auto [y1, z1] = profile(i + 1);
				const TPoint3D a0 = pose.composePoint(TPoint3D(0, y0, z0));
				const TPoint3D a1 = pose.composePoint(TPoint3D(0, y1, z1));
				const TPoint3D b0 = pose.composePoint(TPoint3D(lx, y0, z0));
				const TPoint3D b1 = pose.composePoint(TPoint3D(lx, y1, z1));
				scene.addPrimitive(mvsim::rt::Primitive{mvsim::rt::Triangle{a0, b0, b1}, name});
				scene.addPrimitive(mvsim::rt::Primitive{mvsim::rt::Triangle{a0, b1, a1}, name});
			}
			return;
		}
		case GeometryType::Invalid:
		default:
			break;	// Fall through: use the block's 2D footprint instead.
	}

	// No <geometry> tag (or unrecognized type): approximate with the
	// block's own 2D collision footprint (an explicit <shape>, the default
	// unit square, or <shape_from_visual/>'s bounding box) extruded in
	// [block_z_min, block_z_max]. Note this is an approximation whenever
	// the footprint came from <shape_from_visual/>: the block's exact
	// visual mesh is not ray-traced, only its footprint's bounding prism.
	const auto& poly = block.blockShape();
	if (poly.size() < 3)
	{
		return;
	}
	mvsim::rt::Prism prism;
	std::vector<TPoint2D> worldPoly;
	worldPoly.reserve(poly.size());
	for (const auto& v : poly)
	{
		worldPoly.push_back(toWorldXY(pose, v.x, v.y));
	}
	prism.contour = mrpt::math::TPolygon2D(worldPoly);
	prism.zMin = pose.z() + block.block_z_min();
	prism.zMax = pose.z() + block.block_z_max();
	scene.addPrimitive(mvsim::rt::Primitive{prism, name});
}

void addHorizontalPlane(mvsim::rt::RayScene& scene, const mvsim::HorizontalPlane& hp)
{
	mvsim::rt::Plane pl;
	pl.center = {(hp.xMin() + hp.xMax()) * 0.5, (hp.yMin() + hp.yMax()) * 0.5, hp.z()};
	pl.normal = {0, 0, 1};
	pl.uAxis = {1, 0, 0};
	pl.vAxis = {0, 1, 0};
	pl.halfU = (hp.xMax() - hp.xMin()) * 0.5;
	pl.halfV = (hp.yMax() - hp.yMin()) * 0.5;
	scene.addPrimitive(mvsim::rt::Primitive{pl, hp.getName()});
}

void addVerticalPlane(mvsim::rt::RayScene& scene, const mvsim::VerticalPlane& vp)
{
	const double dx = vp.x1() - vp.x0();
	const double dy = vp.y1() - vp.y0();
	const double length = std::sqrt(dx * dx + dy * dy);
	if (length < 1e-9)
	{
		return;
	}
	const double dirX = dx / length, dirY = dy / length;
	const double nX = -dirY, nY = dirX;
	const double halfT = vp.thickness() * 0.5;

	// Breakpoints along the wall's length axis, from every opening's span:
	std::vector<double> breaks = {0.0, length};
	for (const auto& o : vp.openings())
	{
		const double c = o.position * length;
		breaks.push_back(std::clamp(c - o.width * 0.5, 0.0, length));
		breaks.push_back(std::clamp(c + o.width * 0.5, 0.0, length));
	}
	std::sort(breaks.begin(), breaks.end());

	auto footprintAt = [&](double a, double b)
	{
		const TPoint2D pa{vp.x0() + a * dirX, vp.y0() + a * dirY};
		const TPoint2D pb{vp.x0() + b * dirX, vp.y0() + b * dirY};
		return mrpt::math::TPolygon2D(
			{TPoint2D(pa.x - halfT * nX, pa.y - halfT * nY),
			 TPoint2D(pb.x - halfT * nX, pb.y - halfT * nY),
			 TPoint2D(pb.x + halfT * nX, pb.y + halfT * nY),
			 TPoint2D(pa.x + halfT * nX, pa.y + halfT * nY)});
	};

	for (size_t i = 0; i + 1 < breaks.size(); i++)
	{
		const double a = breaks[i], b = breaks[i + 1];
		if (b - a < 1e-6)
		{
			continue;
		}
		const double mid = 0.5 * (a + b);

		const mvsim::VerticalPlane::Opening* active = nullptr;
		for (const auto& o : vp.openings())
		{
			const double c = o.position * length;
			if (mid >= c - o.width * 0.5 && mid <= c + o.width * 0.5)
			{
				active = &o;
				break;
			}
		}

		mvsim::rt::Prism prism;
		prism.contour = footprintAt(a, b);
		if (!active)
		{
			prism.zMin = vp.z();
			prism.zMax = vp.z() + vp.height();
			scene.addPrimitive(mvsim::rt::Primitive{prism, vp.getName()});
			continue;
		}
		// Both doors and windows are modeled as full through-gaps (mvsim
		// has no glass optical model); only the solid material above/below
		// the opening remains.
		if (active->z_min > 1e-4)
		{
			mvsim::rt::Prism below = prism;
			below.zMin = vp.z();
			below.zMax = vp.z() + active->z_min;
			scene.addPrimitive(mvsim::rt::Primitive{below, vp.getName()});
		}
		if (active->z_max < vp.height() - 1e-4)
		{
			mvsim::rt::Prism above = prism;
			above.zMin = vp.z() + active->z_max;
			above.zMax = vp.z() + vp.height();
			scene.addPrimitive(mvsim::rt::Primitive{above, vp.getName()});
		}
	}
}

void addElevationMap(mvsim::rt::RayScene& scene, const mvsim::ElevationMap& em)
{
	mvsim::rt::HeightField hf;
	hf.z = em.meshZ();
	hf.minX = em.meshMinX();
	hf.maxX = em.meshMaxX();
	hf.minY = em.meshMinY();
	hf.maxY = em.meshMaxY();
	scene.addPrimitive(mvsim::rt::Primitive{hf, em.getName()});
}

}  // namespace

mvsim::rt::RayScene SceneBuilder::build(
	const mvsim::World& world, const std::string& egoVehicleName, const Options& opts)
{
	mvsim::rt::RayScene scene;

	for (const auto& [name, block] : world.getListOfBlocks())
	{
		if (!block)
		{
			continue;
		}
		addBlock(scene, *block, opts);
	}

	for (const auto& elem : world.getListOfWorldElements())
	{
		if (!elem)
		{
			continue;
		}
		if (auto* hp = dynamic_cast<mvsim::HorizontalPlane*>(elem.get()); hp)
		{
			addHorizontalPlane(scene, *hp);
		}
		else if (auto* vp = dynamic_cast<mvsim::VerticalPlane*>(elem.get()); vp)
		{
			addVerticalPlane(scene, *vp);
		}
		else if (auto* em = dynamic_cast<mvsim::ElevationMap*>(elem.get()); em)
		{
			addElevationMap(scene, *em);
		}
		else if (
			dynamic_cast<mvsim::GroundGrid*>(elem.get()) ||
			dynamic_cast<mvsim::SkyBox*>(elem.get()) ||
			dynamic_cast<mvsim::PropertyRegion*>(elem.get()))
		{
			// Purely visual or non-geometric: not an obstacle, skip silently.
		}
		else if (dynamic_cast<mvsim::OccupancyGridMap*>(elem.get()))
		{
			warnOrThrow(
				opts, "Unsupported world element '" + elem->getName() +
						  "': occupancy grids are not ray-traceable geometry");
		}
		else if (dynamic_cast<mvsim::PointCloud*>(elem.get()))
		{
			warnOrThrow(
				opts, "Unsupported world element '" + elem->getName() +
						  "': point clouds are not ray-traceable geometry");
		}
		else
		{
			warnOrThrow(
				opts, "Unsupported world element '" + elem->getName() + "' of unrecognized type");
		}
	}

	for (const auto& [name, veh] : world.getListOfVehicles())
	{
		if (name == egoVehicleName)
		{
			continue;
		}
		warnOrThrow(
			opts, "Additional vehicle '" + name +
					  "' found besides the ego vehicle; only a single vehicle is supported and "
					  "extra vehicles are not ray-traced as obstacles");
	}

	scene.build();
	return scene;
}
