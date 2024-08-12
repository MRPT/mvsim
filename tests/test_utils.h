/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/os.h>
#include <mrpt/version.h>

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
