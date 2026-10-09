/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/img/CImage.h>
#include <mrpt/viz/CBox.h>
#include <mrpt/viz/CCylinder.h>
#include <mrpt/viz/CDisk.h>
#include <mrpt/viz/CSetOfLines.h>
#include <mrpt/viz/CSetOfTriangles.h>
#include <mrpt/viz/CSphere.h>
#include <mrpt/viz/CTexturedPlane.h>
#include <mvsim/RuntimeObjects.h>
#include <mvsim/World.h>

using namespace mvsim;

namespace
{
using Shape = RuntimeObjectDescription::Shape;

// Height from which to look down for the ground, for "on_ground" objects:
constexpr float kGroundQueryHeight = 1e4f;

void validate(const RuntimeObjectDescription& d)
{
	ASSERTMSG_(!d.name.empty(), "Runtime object name cannot be empty");
	switch (d.shape)
	{
		case Shape::Polygon:
			ASSERTMSG_(
				d.polygon.size() >= 3,
				mrpt::format("Polygon object '%s' needs >=3 vertices", d.name.c_str()));
			break;
		case Shape::Triangles:
			ASSERTMSG_(
				!d.points.empty() && d.points.size() % 3 == 0,
				mrpt::format("Triangles object '%s' needs a multiple of 3 points", d.name.c_str()));
			ASSERTMSG_(
				d.point_colors.empty() || d.point_colors.size() == d.points.size(),
				mrpt::format(
					"Triangles object '%s': point_colors length mismatch", d.name.c_str()));
			break;
		case Shape::Lines:
			ASSERTMSG_(
				!d.points.empty() && d.points.size() % 2 == 0,
				mrpt::format("Lines object '%s' needs a multiple of 2 points", d.name.c_str()));
			break;
		default:
			break;
	};
}

mrpt::viz::TTriangle makeTriangle(
	const mrpt::math::TPoint3D& a, const mrpt::math::TPoint3D& b, const mrpt::math::TPoint3D& c,
	const mrpt::img::TColor& ca, const mrpt::img::TColor& cb, const mrpt::img::TColor& cc)
{
	mrpt::viz::TTriangle t(a.cast<float>(), b.cast<float>(), c.cast<float>());
	t.vertices[0].setColor(ca);
	t.vertices[1].setColor(cb);
	t.vertices[2].setColor(cc);
	return t;
}

mrpt::viz::CVisualObject::Ptr buildGlObject(const RuntimeObjectDescription& d, const World& world)
{
	const auto& s = d.size;
	const auto& col = d.color;

	switch (d.shape)
	{
		case Shape::Box:
		{
			auto o = mrpt::viz::CBox::Create(
				mrpt::math::TPoint3D(-0.5 * s.x, -0.5 * s.y, -0.5 * s.z),
				mrpt::math::TPoint3D(0.5 * s.x, 0.5 * s.y, 0.5 * s.z));
			o->setColor_u8(col);
			return o;
		}
		case Shape::Cylinder:
		{
			auto o = mrpt::viz::CCylinder::Create(
				static_cast<float>(0.5 * s.x), static_cast<float>(0.5 * s.x),
				static_cast<float>(s.z));
			o->setColor_u8(col);
			return o;
		}
		case Shape::Sphere:
		{
			auto o = mrpt::viz::CSphere::Create(static_cast<float>(0.5 * s.x));
			o->setColor_u8(col);
			return o;
		}
		case Shape::Rectangle:
		{
			const auto hx = static_cast<float>(0.5 * s.x);
			const auto hy = static_cast<float>(0.5 * s.y);
			if (!d.texture.empty())
			{
				auto o = mrpt::viz::CTexturedPlane::Create(-hx, hx, -hy, hy);
				mrpt::img::CImage img;
				const auto path = world.xmlPathToActualPath(d.texture);
				ASSERTMSG_(
					img.loadFromFile(path),
					mrpt::format("Cannot load texture image '%s'", path.c_str()));
				o->assignImage(img);
				return o;
			}
			auto o = mrpt::viz::CSetOfTriangles::Create();
			const mrpt::math::TPoint3D p0(-hx, -hy, 0);
			const mrpt::math::TPoint3D p1(hx, -hy, 0);
			const mrpt::math::TPoint3D p2(hx, hy, 0);
			const mrpt::math::TPoint3D p3(-hx, hy, 0);
			o->insertTriangle(makeTriangle(p0, p1, p2, col, col, col));
			o->insertTriangle(makeTriangle(p0, p2, p3, col, col, col));
			return o;
		}
		case Shape::Disk:
		{
			auto o = mrpt::viz::CDisk::Create(static_cast<float>(0.5 * s.x), 0.0f);
			o->setColor_u8(col);
			return o;
		}
		case Shape::Polygon:
		{
			// Convex polygon: triangle fan from the first vertex.
			auto o = mrpt::viz::CSetOfTriangles::Create();
			const auto& pl = d.polygon;
			const mrpt::math::TPoint3D p0(pl[0].x, pl[0].y, 0);
			for (size_t i = 1; i + 1 < pl.size(); i++)
			{
				o->insertTriangle(makeTriangle(
					p0, {pl[i].x, pl[i].y, 0}, {pl[i + 1].x, pl[i + 1].y, 0}, col, col, col));
			}
			return o;
		}
		case Shape::Triangles:
		{
			auto o = mrpt::viz::CSetOfTriangles::Create();
			const auto& pts = d.points;
			const bool perPointColor = !d.point_colors.empty();
			for (size_t i = 0; i + 2 < pts.size(); i += 3)
			{
				o->insertTriangle(makeTriangle(
					pts[i], pts[i + 1], pts[i + 2], perPointColor ? d.point_colors[i] : col,
					perPointColor ? d.point_colors[i + 1] : col,
					perPointColor ? d.point_colors[i + 2] : col));
			}
			return o;
		}
		case Shape::Lines:
		{
			auto o = mrpt::viz::CSetOfLines::Create();
			for (size_t i = 0; i + 1 < d.points.size(); i += 2)
			{
				o->appendLine(d.points[i], d.points[i + 1]);
			}
			o->setLineWidth(s.x > 0 ? static_cast<float>(s.x) : 1.0f);
			o->setColor_u8(col);
			return o;
		}
	};
	THROW_EXCEPTION("Unknown runtime object shape");
}

}  // namespace

void RuntimeObjects::spawn(const std::vector<RuntimeObjectDescription>& objs)
{
	for (const auto& d : objs)
	{
		validate(d);
	}

	// Resolve ground heights outside of our mutex:
	std::vector<RuntimeObjectDescription> resolved = objs;
	for (auto& d : resolved)
	{
		if (d.on_ground)
		{
			const float zGround = parent_.getHighestElevationUnder(mrpt::math::TPoint3Df(
				static_cast<float>(d.pose.x), static_cast<float>(d.pose.y), kGroundQueryHeight));
			d.pose.z += zGround;
			d.on_ground = false;
		}
	}

	auto lck = std::lock_guard(mtx_);
	for (auto& d : resolved)
	{
		if (auto it = objects_.find(d.name); it != objects_.end())
		{
			removeEntry(it);
		}
		Entry e;
		e.desc = std::move(d);
		objects_.emplace(e.desc.name, std::move(e));
	}
	anyChange_ = true;
}

bool RuntimeObjects::setPose(const std::string& name, const mrpt::math::TPose3D& pose)
{
	auto lck = std::lock_guard(mtx_);
	auto it = objects_.find(name);
	if (it == objects_.end())
	{
		return false;
	}
	it->second.desc.pose = pose;
	it->second.needsPose = true;
	anyChange_ = true;
	return true;
}

std::optional<mrpt::math::TPose3D> RuntimeObjects::getPose(const std::string& name) const
{
	auto lck = std::lock_guard(mtx_);
	auto it = objects_.find(name);
	if (it == objects_.end())
	{
		return {};
	}
	return it->second.desc.pose;
}

void RuntimeObjects::removeEntry(std::map<std::string, Entry>::iterator it)
{
	// mtx_ must be locked by the caller.
	if (it->second.gl)
	{
		pendingRemovals_.push_back(it->second.gl);
	}
	objects_.erase(it);
	anyChange_ = true;
}

size_t RuntimeObjects::remove(const std::vector<std::string>& names)
{
	auto lck = std::lock_guard(mtx_);
	size_t n = 0;
	for (const auto& name : names)
	{
		if (auto it = objects_.find(name); it != objects_.end())
		{
			removeEntry(it);
			n++;
		}
	}
	return n;
}

size_t RuntimeObjects::removeByPrefix(const std::string& prefix)
{
	auto lck = std::lock_guard(mtx_);
	size_t n = 0;
	for (auto it = objects_.lower_bound(prefix);
		 it != objects_.end() && it->first.compare(0, prefix.size(), prefix) == 0;)
	{
		auto toRemove = it++;
		removeEntry(toRemove);
		n++;
	}
	return n;
}

size_t RuntimeObjects::size() const
{
	auto lck = std::lock_guard(mtx_);
	return objects_.size();
}

std::vector<std::pair<std::string, mrpt::math::TPose3D>> RuntimeObjects::poses() const
{
	auto lck = std::lock_guard(mtx_);
	std::vector<std::pair<std::string, mrpt::math::TPose3D>> ret;
	ret.reserve(objects_.size());
	for (const auto& [name, e] : objects_)
	{
		ret.emplace_back(name, e.desc.pose);
	}
	return ret;
}

void RuntimeObjects::guiUpdate(mrpt::viz::Scene& viz, mrpt::viz::Scene& physical)
{
	auto lck = std::lock_guard(mtx_);

	if (!glContainerViz_)
	{
		glContainerViz_ = mrpt::viz::CSetOfObjects::Create();
		glContainerViz_->setName("__runtime_objects");
		glContainerPhysical_ = mrpt::viz::CSetOfObjects::Create();
		glContainerPhysical_->setName("__runtime_objects");
		viz.insert(glContainerViz_);
		physical.insert(glContainerPhysical_);
	}

	if (!anyChange_)
	{
		return;
	}
	anyChange_ = false;

	for (const auto& gl : pendingRemovals_)
	{
		glContainerViz_->removeObject(gl);
		glContainerPhysical_->removeObject(gl);
	}
	pendingRemovals_.clear();

	for (auto& [name, e] : objects_)
	{
		if (e.needsRebuild)
		{
			e.needsRebuild = false;
			e.needsPose = true;
			try
			{
				e.gl = buildGlObject(e.desc, parent_);
			}
			catch (const std::exception& ex)
			{
				parent_.logStr(
					mrpt::system::LVL_ERROR,
					mrpt::format("Cannot create runtime object '%s': %s", name.c_str(), ex.what()));
				e.gl.reset();
				continue;
			}
			e.gl->setName(name);
			glContainerViz_->insert(e.gl);
			if (e.desc.visible_to_sensors)
			{
				glContainerPhysical_->insert(e.gl);
			}
		}
		if (e.needsPose && e.gl)
		{
			e.needsPose = false;
			e.gl->setPose(parent_.applyWorldRenderOffset(e.desc.pose));
		}
	}
}
