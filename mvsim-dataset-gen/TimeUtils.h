/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/Clock.h>

#include <cmath>
#include <cstdint>

namespace mvsim_dataset_gen
{
/** Like mrpt::Clock::fromDouble(), but rounding to the nearest 100 ns tick
 * instead of truncating. With truncation, times such as 0.0045 s land one
 * tick early, and the IMU finite differences (a stencil of a fraction of a
 * millisecond) amplify that jitter into large spurious accelerations. */
inline mrpt::Clock::time_point timeFromSeconds(double t)
{
	// Offset between the 1601-01-01 epoch of mrpt::Clock and the UNIX
	// epoch, in 100-nanosecond ticks.
	constexpr int64_t kUnixEpochOffset = INT64_C(116444736) * INT64_C(1000000000);
	const int64_t ticks = static_cast<int64_t>(std::llround(t * 1e7)) + kUnixEpochOffset;
	return mrpt::Clock::time_point(mrpt::Clock::duration(ticks));
}
}  // namespace mvsim_dataset_gen
