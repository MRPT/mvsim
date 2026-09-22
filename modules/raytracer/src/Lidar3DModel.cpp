/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/raytracer/Lidar3DModel.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

using namespace mvsim::rt;

namespace
{
constexpr double kDeg2Rad = M_PI / 180.0;
}

Lidar3DModel::Lidar3DModel(Params params) : params_(std::move(params))
{
	if (params_.horzNumRays < 1)
	{
		throw std::invalid_argument("Lidar3DModel: horzNumRays must be >= 1");
	}
	if (params_.sensorPeriod <= 0)
	{
		throw std::invalid_argument("Lidar3DModel: sensorPeriod must be > 0");
	}

	if (!params_.verticalRayAnglesDeg.empty())
	{
		// Mirrors Lidar3D::loadConfigFrom(): re-sort ascending, regardless
		// of the order given in the XML.
		std::set<double> angles(
			params_.verticalRayAnglesDeg.begin(), params_.verticalRayAnglesDeg.end());
		ringElevationsRad_.assign(angles.begin(), angles.end());
		for (double& a : ringElevationsRad_)
		{
			a *= kDeg2Rad;
		}
	}
	else
	{
		if (params_.vertNumRays < 1)
		{
			throw std::invalid_argument("Lidar3DModel: vertNumRays must be >= 1");
		}
		ringElevationsRad_.resize(params_.vertNumRays);
		if (params_.vertNumRays == 1)
		{
			ringElevationsRad_[0] = 0;
		}
		else
		{
			for (int i = 0; i < params_.vertNumRays; i++)
			{
				const double frac = -0.5 + i * 1.0 / (params_.vertNumRays - 1);
				ringElevationsRad_[i] = params_.vertFovDegrees * frac * kDeg2Rad;
			}
		}
	}
}

double Lidar3DModel::columnAzimuthRad(int col) const
{
	return -M_PI + col * (2.0 * M_PI / params_.horzNumRays);
}

double Lidar3DModel::columnFireTime(int col) const
{
	return col * (params_.sensorPeriod / params_.horzNumRays);
}

mrpt::math::TPoint3D Lidar3DModel::rayDirection(int ring, int col) const
{
	const double elev = ringElevationRad(ring);
	const double az = columnAzimuthRad(col);
	const double cosElev = std::cos(elev);
	return {cosElev * std::cos(az), cosElev * std::sin(az), std::sin(elev)};
}
