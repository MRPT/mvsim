/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/version.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/Comms/common.h>
#include <mvsim/Comms/ports.h>
#if MRPT_VERSION >= 0x204
#include <mrpt/system/thread_name.h>
#endif

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

#include <google/protobuf/text_format.h>

#include <zmq.hpp>

#include "ListNodesAnswer.pb.h"
#include "ListNodesRequest.pb.h"
#include "RegisterNodeAnswer.pb.h"
#include "RegisterNodeRequest.pb.h"
#include "UnregisterNodeAnswer.pb.h"
#include "UnregisterNodeRequest.pb.h"

#endif

using namespace mvsim;

struct Client::ZMQImpl
{
#if defined(MVSIM_HAS_ZMQ)
	zmq::context_t context{1};
	zmq::socket_t mainReqSocket;
#endif
};

Client::Client()
	: mrpt::system::COutputLogger("mvsim::Client"),
	  zmq_(mrpt::make_impl<std::shared_ptr<Client::ZMQImpl>>(
		  std::make_shared<Client::ZMQImpl>()))
{
}
Client::Client(const std::string& nodeName) : Client() { setName(nodeName); }

Client::~Client() { shutdown(); }

void Client::setName(const std::string& nodeName) { nodeName_ = nodeName; }

void Client::connect()
{
	using namespace std::string_literals;
	ASSERTMSG_(
		!(*zmq_)->mainReqSocket.connected(), "Client is already running.");

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

	(*zmq_)->mainReqSocket = zmq::socket_t((*zmq_)->context, ZMQ_REQ);
	(*zmq_)->mainReqSocket.connect(
		"tcp://"s + serverHostAddress_ + ":"s +
		std::to_string(MVSIM_PORTNO_MAIN_REP));

	// Let the server know about this new node:
	doRegisterClient();

#else
	THROW_EXCEPTION(
		"MVSIM needs building with ZMQ and PROTOBUF to enable "
		"client/server");
#endif
}

void Client::shutdown() noexcept
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

	if (!(*zmq_)->mainReqSocket.connected()) return;

	try
	{
		MRPT_LOG_DEBUG_STREAM("Unregistering from server.");
		doUnregisterClient();
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"shutdown: Exception: " << mrpt::exception_to_str(e));
	}

#if ZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 4, 0)
	(*zmq_)->context.shutdown();
#else
	// Missing shutdown() in older versions:
	zmq_ctx_shutdown((*zmq_)->context.operator void*());
#endif

#endif
}

void Client::doRegisterClient()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	auto& s = (*zmq_)->mainReqSocket;

	mvsim_msgs::RegisterNodeRequest rnq;
	rnq.set_nodename(nodeName_);
	mvsim::sendMessage(rnq, s);

	//  Get the reply.
	const zmq::message_t reply = mvsim::receiveMessage(s);

	mvsim_msgs::RegisterNodeAnswer rna;
	mvsim::parseMessage(reply, rna);
	if (!rna.success())
	{
		THROW_EXCEPTION_FMT(
			"Server did not allow registering node: %s",
			rna.errormessage().c_str());
	}
	MRPT_LOG_DEBUG("Successfully registered in the server.");
#else
	THROW_EXCEPTION("MVSIM built without ZMQ");
#endif
}

void Client::doUnregisterClient()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	auto& s = (*zmq_)->mainReqSocket;

	mvsim_msgs::UnregisterNodeRequest rnq;
	rnq.set_nodename(nodeName_);
	mvsim::sendMessage(rnq, s);

	//  Get the reply.
	const zmq::message_t reply = mvsim::receiveMessage(s);

	mvsim_msgs::UnregisterNodeAnswer rna;
	mvsim::parseMessage(reply, rna);
	if (!rna.success())
	{
		THROW_EXCEPTION_FMT(
			"Server answered an error unregistering node: %s",
			rna.errormessage().c_str());
	}
	MRPT_LOG_DEBUG("Successfully unregistered in the server.");
#else
	THROW_EXCEPTION("MVSIM built without ZMQ");
#endif
}

std::vector<Client::InfoPerNode> Client::requestListOfNodes()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	auto& s = (*zmq_)->mainReqSocket;

	mvsim_msgs::ListNodesRequest req;
	mvsim::sendMessage(req, s);

	//  Get the reply.
	const zmq::message_t reply = mvsim::receiveMessage(s);

	mvsim_msgs::ListNodesAnswer lna;
	mvsim::parseMessage(reply, lna);

	std::vector<Client::InfoPerNode> nodes;
	nodes.resize(lna.nodes_size());

	for (int i = 0; i < lna.nodes_size(); i++)
	{
		nodes[i].name = lna.nodes(i);
	}
	return nodes;
#else
	THROW_EXCEPTION("MVSIM built without ZMQ");
#endif
}
