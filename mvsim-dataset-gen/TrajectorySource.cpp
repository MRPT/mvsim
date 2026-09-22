/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include "TrajectorySource.h"

#include <mrpt/core/exceptions.h>

#include <algorithm>

using namespace mvsim_dataset_gen;

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
