/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mvsim/World.h>

using namespace mvsim;

#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF
//#include <mrpt/serialization/zmq_serialization.h>

#include <zmq.hpp>

#include "TimeStampedPose.pb.h"
#endif

#if MVSIM_HAS_ZMQ && MVSIM_HAS_PROTOBUF

static void test()
{
	mvsim_msgs::TimeStampedPose t;
	std::string s = t.SerializeAsString();
}

#endif
