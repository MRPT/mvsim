/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <google/protobuf/message.h>
#include <mvsim/Comms/common.h>

#include <zmq.hpp>

using namespace mvsim;

void mvsim::sendMessage(const google::protobuf::MessageLite& m, zmq::socket_t& socket)
{
	mrpt::io::CMemoryStream buf;
	auto arch = mrpt::serialization::archiveFrom(buf);

	arch << m.GetTypeName();
	arch << m.SerializeAsString();

	zmq::message_t msg(buf.getRawBufferData(), buf.getTotalBytesCount());
#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 3, 1)
	socket.send(msg, zmq::send_flags::none);
#else
	socket.send(msg);
#endif
}

std::tuple<std::string, std::string> mvsim::internal::parseMessageToParts(const zmq::message_t& msg)
{
	mrpt::io::CMemoryStream buf;
	buf.assignMemoryNotOwn(msg.data(), msg.size());

	auto arch = mrpt::serialization::archiveFrom(buf);

	std::string typeName, serializedData;
	arch >> typeName >> serializedData;
	return {typeName, serializedData};
}

void mvsim::parseMessage(const zmq::message_t& msg, google::protobuf::MessageLite& out)
{
	const auto [typeName, serializedData] = internal::parseMessageToParts(msg);

	ASSERT_EQUAL_(typeName, out.GetTypeName());

	bool ok = out.ParseFromString(serializedData);
	if (!ok)
		THROW_EXCEPTION_FMT(
			"Format error: protobuf could not decode binary message of "
			"type "
			"'%s'",
			typeName.c_str());
}

zmq::message_t mvsim::receiveMessage(zmq::socket_t& s)
{
	zmq::message_t m;
#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 3, 1)
	std::optional<size_t> msgSize = s.recv(m);
	ASSERT_(msgSize.has_value());
#else
	s.recv(&m);
#endif
	return m;
}

std::string mvsim::get_zmq_endpoint(const zmq::socket_t& s)
{
#if CPPZMQ_VERSION > ZMQ_MAKE_VERSION(4, 7, 0)
	return s.get(zmq::sockopt::last_endpoint);
#else
	char assignedPort[200];
	size_t assignedPortLen = sizeof(assignedPort);
	s.getsockopt(ZMQ_LAST_ENDPOINT, &assignedPort, &assignedPortLen);
	assignedPort[assignedPortLen] = '\0';
	return {assignedPort};
#endif
}

#endif
