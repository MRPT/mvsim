/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "TrajectorySource.h"

#include <mrpt/core/exceptions.h>
#include <mrpt/math/CQuaternion.h>
#include <mvsim/World.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <regex>
#include <sstream>

#include "TimeUtils.h"

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
			const std::string tagText = (*it)[0].str();
			const std::string attrs = (*it)[1].str();
			std::optional<double> t, x, y;
			for (auto ait = std::sregex_iterator(attrs.begin(), attrs.end(), attrRe);
				 ait != std::sregex_iterator(); ++ait)
			{
				const std::string name = (*ait)[1].str();
				const std::string valStr = (*ait)[2].str();
				double val;
				try
				{
					val = std::stod(valStr);
				}
				catch (const std::exception&)
				{
					THROW_EXCEPTION(
						"Malformed waypoint tag (non-numeric '" + name +
						"' attribute): " + tagText);
				}
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
			if (!t || !x || !y)
			{
				THROW_EXCEPTION("Malformed waypoint tag (missing t/x/y attribute): " + tagText);
			}
			wps.push_back({*t, *x, *y});
		}
	}
	else
	{
		std::istringstream is(content);
		std::string line;
		size_t lineNo = 0;
		while (std::getline(is, line))
		{
			lineNo++;
			const size_t h = line.find('#');
			const std::string dataPart = h != std::string::npos ? line.substr(0, h) : line;
			if (dataPart.find_first_not_of(" \t\r\n") == std::string::npos)
			{
				continue;  // Blank or comment-only line.
			}
			std::istringstream ls(dataPart);
			double t, x, y;
			if (!(ls >> t >> x >> y))
			{
				THROW_EXCEPTION_FMT(
					"Malformed waypoint at %s:%zu: '%s'", path.c_str(), lineNo, line.c_str());
			}
			wps.push_back({t, x, y});
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
	// Parsed here rather than with CPose3DInterpolator::loadFromTextFile_TUM()
	// so that timestamps are rounded to the nearest clock tick (see
	// timeFromSeconds()): dense trajectories need exactly regular knots.
	std::ifstream f(path);
	if (!f)
	{
		THROW_EXCEPTION("Failed to load TUM trajectory file: " + path);
	}
	std::string line;
	size_t lineNo = 0;
	while (std::getline(f, line))
	{
		lineNo++;
		const size_t h = line.find('#');
		const std::string dataPart = h != std::string::npos ? line.substr(0, h) : line;
		if (dataPart.find_first_not_of(" \t\r\n") == std::string::npos)
		{
			continue;  // Blank or comment-only line.
		}
		std::istringstream ls(dataPart);
		double t = 0;
		double x = 0;
		double y = 0;
		double z = 0;
		double qx = 0;
		double qy = 0;
		double qz = 0;
		double qw = 0;
		if (!(ls >> t >> x >> y >> z >> qx >> qy >> qz >> qw))
		{
			THROW_EXCEPTION_FMT(
				"Malformed TUM line at %s:%zu: '%s'", path.c_str(), lineNo, line.c_str());
		}
		const mrpt::math::CQuaternionDouble q(qw, qx, qy, qz);
		const mrpt::poses::CPose3D p(q, x, y, z);
		insertKnot(timeFromSeconds(t), p.asTPose());
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
		insertKnot(timeFromSeconds(wps[i].t), p6);
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

void TrajectorySource::insertKnot(const mrpt::Clock::time_point& t, const mrpt::math::TPose3D& p)
{
	if (path_.empty())
	{
		t0_ = t;
	}
	path_.insert(mrpt::Clock::time_point(t - t0_), p);
}

double TrajectorySource::startTime() const
{
	return mrpt::Clock::toDouble(t0_ + path_.begin()->first.time_since_epoch());
}

double TrajectorySource::endTime() const
{
	return mrpt::Clock::toDouble(t0_ + path_.rbegin()->first.time_since_epoch());
}

mrpt::poses::CPose3D TrajectorySource::poseAt(double tEpochSeconds) const
{
	return poseAt(timeFromSeconds(tEpochSeconds));
}

mrpt::poses::CPose3D TrajectorySource::poseAt(const mrpt::Clock::time_point& tQuery) const
{
	const mrpt::Clock::time_point tRel(tQuery - t0_);
	const auto t = std::clamp(tRel, path_.begin()->first, path_.rbegin()->first);
	mrpt::poses::CPose3D out;
	bool valid = false;
	path_.interpolate(t, out, valid);
	if (!valid)
	{
		THROW_EXCEPTION_FMT(
			"TrajectorySource: could not interpolate at t=%.6f", mrpt::Clock::toDouble(tQuery));
	}
	return out;
}
