/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <google/protobuf/message.h>
#include <mvsim/Comms/common.h>

#include <zmq.hpp>

using namespace mvsim;

void mvsim::sendMessage(
	const google::protobuf::MessageLite& m, zmq::socket_t& socket)
{
	mrpt::io::CMemoryStream buf;
	auto arch = mrpt::serialization::archiveFrom(buf);

	arch << m.GetTypeName();
	arch << m.SerializeAsString();

	zmq::message_t msg(buf.getRawBufferData(), buf.getTotalBytesCount());
#if ZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 4, 0)
	socket.send(msg, zmq::send_flags::none);
#else
	socket.send(msg);
#endif
}

#endif
