/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

#include "zmq_fwrds.h"

namespace google::protobuf
{
class MessageLite;
}

namespace mvsim
{
/** Sends a ZMQ message comprising:
 * - std::string with the protobuf message type name.
 * - std::string with the binary serialization of the message itself.
 */
void sendMessage(const google::protobuf::MessageLite& m, zmq::socket_t& socket);

/** Parses a ZMQ message received as sent by sendMessage(), and decodes it into
 * the provided google::protobuf message. Use this signature when the expected
 * type of the received message is known before hand.
 *
 * \exception std::runtime_error If the message type does not match with out.
 */
void parseMessage(
	const zmq::message_t& msg, google::protobuf::MessageLite& out);

}  // namespace mvsim
#endif
