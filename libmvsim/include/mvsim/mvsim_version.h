/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#define MVSIM_MAJOR_VERSION    0
#define MVSIM_MINOR_VERSION    9
#define MVSIM_PATCH_VERSION    0

#define MVSIM_STR_EXP(__A)  #__A
#define MVSIM_STR(__A)      MVSIM_STR_EXP(__A)
#define MVSIM_VERSION       MVSIM_STR(MVSIM_MAJOR_VERSION) "." MVSIM_STR(MVSIM_MINOR_VERSION) "." MVSIM_STR(MVSIM_PATCH_VERSION)
