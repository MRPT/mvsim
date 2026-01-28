/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
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
	std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);

	// typename:
	const std::string typeName = m.GetTypeName();
	const uint32_t typeNameLen = static_cast<uint32_t>(typeName.size());
	ss.write(reinterpret_cast<const char*>(&typeNameLen), sizeof(typeNameLen));
	ss.write(typeName.data(), static_cast<std::streamsize>(typeName.size()));

	// serialized data:
	const auto sData = m.SerializeAsString();
	const uint32_t sDataLen = static_cast<uint32_t>(sData.size());
	ss.write(reinterpret_cast<const char*>(&sDataLen), sizeof(sDataLen));
	ss.write(sData.data(), static_cast<std::streamsize>(sData.size()));

	// Send the message:
	const auto& s = ss.str();
	zmq::message_t msg(s.data(), s.size());

#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 3, 1)
	socket.send(msg, zmq::send_flags::none);
#else
	socket.send(msg);
#endif
}

std::tuple<std::string, std::string> mvsim::internal::parseMessageToParts(const zmq::message_t& msg)
{
	std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
	ss.write(static_cast<const char*>(msg.data()), static_cast<std::streamsize>(msg.size()));

	// type:
	uint32_t typeNameLen;
	ss.read(reinterpret_cast<char*>(&typeNameLen), sizeof(typeNameLen));
	char typeNameBuf[256];
	ASSERT_(typeNameLen < sizeof(typeNameBuf));
	ss.read(typeNameBuf, static_cast<std::streamsize>(typeNameLen));
	std::string typeName(typeNameBuf, typeNameLen);

	// Data:
	uint32_t sDataLen;
	ss.read(reinterpret_cast<char*>(&sDataLen), sizeof(sDataLen));
	std::string serializedData;
	serializedData.resize(sDataLen);
	ss.read(reinterpret_cast<char*>(serializedData.data()), static_cast<std::streamsize>(sDataLen));

	return {typeName, serializedData};
}

void mvsim::parseMessage(const zmq::message_t& msg, google::protobuf::MessageLite& out)
{
	const auto [typeName, serializedData] = internal::parseMessageToParts(msg);

	ASSERT_EQUAL_(typeName, out.GetTypeName());

	// convert vector<uint8_t> to string:
	const std::string sData(
		reinterpret_cast<const char*>(serializedData.data()), serializedData.size());

	bool ok = out.ParseFromString(sData);
	if (!ok)
	{
		THROW_EXCEPTION_FMT(
			"Format error: protobuf could not decode binary message of type '%s'",
			typeName.c_str());
	}
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
