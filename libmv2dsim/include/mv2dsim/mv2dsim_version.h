/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */

#pragma once

#define MV2DSIM_MAJOR_VERSION    0
#define MV2DSIM_MINOR_VERSION    9
#define MV2DSIM_PATCH_VERSION    0

#define MV2DSIM_STR_EXP(__A)  #__A
#define MV2DSIM_STR(__A)      MV2DSIM_STR_EXP(__A)
#define MV2DSIM_VERSION       MV2DSIM_STR(MV2DSIM_MAJOR_VERSION) "." MV2DSIM_STR(MV2DSIM_MINOR_VERSION) "." MV2DSIM_STR(MV2DSIM_PATCH_VERSION)
