/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/os.h>
#include <mrpt/version.h>

#include <cmath>
#include <cstdio>

// Minimal test framework (no external deps required):

static inline void setConsoleErrorColor()
{
	mrpt::system::consoleColorAndStyle(mrpt::system::ConsoleForegroundColor::RED);
}

static inline void setConsoleNormalColor()
{
	mrpt::system::consoleColorAndStyle(mrpt::system::ConsoleForegroundColor::DEFAULT);
}

static inline void setConsoleBlueColor()
{
	mrpt::system::consoleColorAndStyle(mrpt::system::ConsoleForegroundColor::BLUE);
}

extern int g_failures;

#define EXPECT_NEAR(a, b, tol)                                                      \
	do                                                                              \
	{                                                                               \
		const double va = static_cast<double>(a);                                   \
		const double vb = static_cast<double>(b);                                   \
		if (std::abs(va - vb) > static_cast<double>(tol))                           \
		{                                                                           \
			std::fprintf(                                                           \
				stderr, "FAIL %s:%d  |%f - %f| > %f\n", __FILE__, __LINE__, va, vb, \
				static_cast<double>(tol));                                          \
			++g_failures;                                                           \
		}                                                                           \
	} while (false)

#define EXPECT_GT(a, b)                                                                    \
	do                                                                                     \
	{                                                                                      \
		const double va = static_cast<double>(a);                                          \
		const double vb = static_cast<double>(b);                                          \
		if (!(va > vb))                                                                    \
		{                                                                                  \
			std::fprintf(stderr, "FAIL %s:%d  %f not > %f\n", __FILE__, __LINE__, va, vb); \
			++g_failures;                                                                  \
		}                                                                                  \
	} while (false)

#define EXPECT_LT(a, b)                                                                    \
	do                                                                                     \
	{                                                                                      \
		const double va = static_cast<double>(a);                                          \
		const double vb = static_cast<double>(b);                                          \
		if (!(va < vb))                                                                    \
		{                                                                                  \
			std::fprintf(stderr, "FAIL %s:%d  %f not < %f\n", __FILE__, __LINE__, va, vb); \
			++g_failures;                                                                  \
		}                                                                                  \
	} while (false)
