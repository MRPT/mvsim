/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#define MVSIM_MAJOR_VERSION 0
#define MVSIM_MINOR_VERSION 3
#define MVSIM_PATCH_VERSION 0

#define MVSIM_STR_EXP(__A) #__A
#define MVSIM_STR(__A) MVSIM_STR_EXP(__A)
#define MVSIM_VERSION              \
	MVSIM_STR(MVSIM_MAJOR_VERSION) \
	"." MVSIM_STR(MVSIM_MINOR_VERSION) "." MVSIM_STR(MVSIM_PATCH_VERSION)
