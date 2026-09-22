/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "TrajectorySource.h"

#include <mrpt/core/exceptions.h>
#include <mvsim/World.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <regex>
#include <sstream>

using namespace mvsim_dataset_gen;

namespace
{
struct Waypoint2D
{
	double t, x, y;
};

/** Parses either the `<waypoint t="" x="" y=""/>` XML form (order of
 * attributes within a tag does not matter) or a plain "t x y" text file
 * (# comments and blank lines ignored). Auto-detected from the first
 * non-whitespace character. */
std::vector<Waypoint2D> parseWaypoints2D(const std::string& path)
{
	std::ifstream f(path);
	if (!f)
	{
		THROW_EXCEPTION("Cannot open trajectory file: " + path);
	}
	std::stringstream ss;
	ss << f.rdbuf();
	const std::string content = ss.str();

	const size_t firstNonWs = content.find_first_not_of(" \t\r\n");
	const bool isXml = firstNonWs != std::string::npos && content[firstNonWs] == '<';

	std::vector<Waypoint2D> wps;

	if (isXml)
	{
		// Attributes may appear in any order within the tag, so match each
		// attribute independently rather than assuming t,x,y ordering.
		static const std::regex tagRe(R"re(<waypoint\b([^>]*)/?>)re");
		static const std::regex attrRe(R"re((\w+)\s*=\s*"([^"]*)")re");
		for (auto it = std::sregex_iterator(content.begin(), content.end(), tagRe);
			 it != std::sregex_iterator(); ++it)
		{
			const std::string attrs = (*it)[1].str();
			std::optional<double> t, x, y;
			for (auto ait = std::sregex_iterator(attrs.begin(), attrs.end(), attrRe);
				 ait != std::sregex_iterator(); ++ait)
			{
				const std::string name = (*ait)[1].str();
				const double val = std::stod((*ait)[2].str());
				if (name == "t")
				{
					t = val;
				}
				else if (name == "x")
				{
					x = val;
				}
				else if (name == "y")
				{
					y = val;
				}
			}
			if (t && x && y)
			{
				wps.push_back({*t, *x, *y});
			}
		}
	}
	else
	{
		std::istringstream is(content);
		std::string line;
		while (std::getline(is, line))
		{
			const size_t h = line.find('#');
			if (h != std::string::npos)
			{
				line = line.substr(0, h);
			}
			std::istringstream ls(line);
			double t, x, y;
			if (ls >> t >> x >> y)
			{
				wps.push_back({t, x, y});
			}
		}
	}

	std::sort(
		wps.begin(), wps.end(), [](const Waypoint2D& a, const Waypoint2D& b) { return a.t < b.t; });
	return wps;
}

}  // namespace

void TrajectorySource::loadTum(const std::string& path)
{
	path_.clear();
	path_.setInterpolationMethod(mrpt::poses::imLinearSlerp);
	if (!path_.loadFromTextFile_TUM(path))
	{
		THROW_EXCEPTION("Failed to load TUM trajectory file: " + path);
	}
	if (path_.size() < 2)
	{
		THROW_EXCEPTION("TUM trajectory file must have at least 2 waypoints: " + path);
	}
}

void TrajectorySource::load2DWithTerrain(
	const std::string& path, const mvsim::World& world, double footprintLx, double footprintLy)
{
	const auto wps = parseWaypoints2D(path);
	if (wps.size() < 2)
	{
		THROW_EXCEPTION("2D trajectory file must have at least 2 waypoints: " + path);
	}

	path_.clear();
	path_.setInterpolationMethod(mrpt::poses::imLinearSlerp);

	const double hx = footprintLx * 0.5;
	const double hy = footprintLy * 0.5;
	size_t nProbeMisses = 0;

	for (size_t i = 0; i < wps.size(); i++)
	{
		// Yaw from the path tangent: central difference, one-sided at the
		// endpoints.
		double dx, dy;
		if (i == 0)
		{
			dx = wps[1].x - wps[0].x;
			dy = wps[1].y - wps[0].y;
		}
		else if (i + 1 == wps.size())
		{
			dx = wps[i].x - wps[i - 1].x;
			dy = wps[i].y - wps[i - 1].y;
		}
		else
		{
			dx = wps[i + 1].x - wps[i - 1].x;
			dy = wps[i + 1].y - wps[i - 1].y;
		}
		const double yaw = std::atan2(dy, dx);
		const double cy = std::cos(yaw), sy = std::sin(yaw);

		// 4 probes in the body frame, rotated+translated to world XY:
		auto probeZ = [&](double lx, double ly) -> double
		{
			const double wx = wps[i].x + lx * cy - ly * sy;
			const double wy = wps[i].y + lx * sy + ly * cy;
			const auto elevs = world.getElevationsAt({wx, wy});
			if (elevs.empty())
			{
				nProbeMisses++;
				return 0.0;
			}
			// The *lowest* candidate: the true supporting ground surface,
			// immune to a nearby block's top face reading as "higher
			// terrain" underneath the vehicle.
			return *elevs.begin();
		};
		const double zFL = probeZ(hx, hy);	// front-left
		const double zFR = probeZ(hx, -hy);	 // front-right
		const double zBL = probeZ(-hx, hy);	 // back-left
		const double zBR = probeZ(-hx, -hy);  // back-right

		const double zCenter = 0.25 * (zFL + zFR + zBL + zBR);
		// Forward (X) and lateral (Y) slopes, body frame, small-angle plane fit:
		const double slopeX = ((zFL + zFR) - (zBL + zBR)) / (2 * footprintLx);
		const double slopeY = ((zFL + zBL) - (zFR + zBR)) / (2 * footprintLy);
		const double pitch = std::atan(-slopeX);
		const double roll = std::atan(slopeY);

		const mrpt::math::TPose3D p6{wps[i].x, wps[i].y, zCenter, yaw, pitch, roll};
		path_.insert(mrpt::Clock::fromDouble(wps[i].t), p6);
	}

	if (nProbeMisses > 0)
	{
		std::cerr << "[TrajectorySource] WARNING: " << nProbeMisses
				  << " terrain probe(s) found no ground; used z=0 as a fallback there.\n";
	}
}

void TrajectorySource::setInterpolationMethod(mrpt::poses::TInterpolatorMethod m)
{
	path_.setInterpolationMethod(m);
}

double TrajectorySource::startTime() const { return mrpt::Clock::toDouble(path_.begin()->first); }

double TrajectorySource::endTime() const { return mrpt::Clock::toDouble(path_.rbegin()->first); }

mrpt::poses::CPose3D TrajectorySource::poseAt(double tEpochSeconds) const
{
	const double t = std::clamp(tEpochSeconds, startTime(), endTime());
	mrpt::poses::CPose3D out;
	bool valid = false;
	path_.interpolate(mrpt::Clock::fromDouble(t), out, valid);
	if (!valid)
	{
		THROW_EXCEPTION_FMT("TrajectorySource: could not interpolate at t=%.6f", tEpochSeconds);
	}
	return out;
}
