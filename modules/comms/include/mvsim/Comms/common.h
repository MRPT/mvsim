/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

#include <mrpt/io/CMemoryStream.h>
#include <mrpt/serialization/CArchive.h>

#include <variant>

#include "zmq_fwrds.h"

namespace google::protobuf
{
class MessageLite;
}

namespace mvsim
{
/** \addtogroup mvsim_comms_module
 * @{ */

/** Sends a ZMQ message comprising:
 * - std::string with the protobuf message type name.
 * - std::string with the binary serialization of the message itself.
 */
void sendMessage(const google::protobuf::MessageLite& m, zmq::socket_t& socket);

/** Receives a message from the socket. */
zmq::message_t receiveMessage(zmq::socket_t& s);

/** Parses a ZMQ message received as sent by sendMessage(), and decodes it into
 * the provided google::protobuf message. Use this signature when the expected
 * type of the received message is known before hand.
 *
 * \exception std::runtime_error If the message type does not match with out.
 */
void parseMessage(const zmq::message_t& msg, google::protobuf::MessageLite& out);

class UnexpectedMessageException : public std::runtime_error
{
   public:
	UnexpectedMessageException(const char* reason) : std::runtime_error(reason) {}
};
namespace internal
{
std::tuple<std::string, std::string> parseMessageToParts(const zmq::message_t& msg);

template <typename variant_t, size_t IDX = 0>
variant_t recursiveParse(const std::string& typeName, const std::string& serializedData)
{
	if constexpr (IDX < std::variant_size_v<variant_t>)
	{
		using this_t = std::variant_alternative_t<IDX, variant_t>;
		this_t v;
		const std::string expectedName = v.GetTypeName();
		if (expectedName == typeName)
		{
			bool ok = v.ParseFromString(serializedData);
			if (!ok)
				THROW_EXCEPTION_FMT(
					"Format error: protobuf could not decode binary message of "
					"type '%s'",
					typeName.c_str());
			return {v};
		}
		else
			return recursiveParse<variant_t, IDX + 1>(typeName, serializedData);
	}
	throw UnexpectedMessageException(
		mrpt::format("Type '%s' not found in expected list of variant arguments.", typeName.c_str())
			.c_str());
}
}  // namespace internal

std::string get_zmq_endpoint(const zmq::socket_t& s);

/** Parses a ZMQ message into one of a set of possible protobuf message types,
 * passed as a std::variant<...>.
 *
 * \exception UnexpectedMessageException If none of the types coincide.
 */
template <typename variant_t>
variant_t parseMessageVariant(const zmq::message_t& msg)
{
	const auto [typeName, serializedData] = internal::parseMessageToParts(msg);
	return internal::recursiveParse<variant_t>(typeName, serializedData);
}

/** Based on https://en.cppreference.com/w/cpp/utility/variant/visit */
template <class... Ts>
struct overloaded : Ts...
{
	using Ts::operator()...;
};

/** Based on https://en.cppreference.com/w/cpp/utility/variant/visit */
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

/** @} */

}  // namespace mvsim
#endif
