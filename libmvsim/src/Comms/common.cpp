/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
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

void mvsim::parseMessage(
	const zmq::message_t& msg, google::protobuf::MessageLite& out)
{
	mrpt::io::CMemoryStream buf;
	buf.assignMemoryNotOwn(msg.data(), msg.size());

	auto arch = mrpt::serialization::archiveFrom(buf);

	std::string typeName, serializedData;
	arch >> typeName >> serializedData;

	ASSERT_EQUAL_(typeName, out.GetTypeName());

	bool ok = out.ParseFromString(serializedData);
	if (!ok)
		THROW_EXCEPTION_FMT(
			"Format error: protobuf could not decode binary message of type "
			"'%s'",
			typeName.c_str());
}

#endif
