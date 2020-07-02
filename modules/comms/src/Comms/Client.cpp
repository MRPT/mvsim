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

#include <mutex>
#include <shared_mutex>

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

#include <google/protobuf/text_format.h>

#include <zmq.hpp>

#include "AdvertiseTopicRequest.pb.h"
#include "GenericAnswer.pb.h"
#include "ListNodesAnswer.pb.h"
#include "ListNodesRequest.pb.h"
#include "RegisterNodeAnswer.pb.h"
#include "RegisterNodeRequest.pb.h"
#include "UnregisterNodeRequest.pb.h"

#endif

using namespace mvsim;

#if defined(MVSIM_HAS_ZMQ)
struct InfoPerAdvertisedTopic
{
	InfoPerAdvertisedTopic(zmq::context_t& c) : context(c) {}

	zmq::context_t& context;

	std::string topicName;
	zmq::socket_t pubSocket = zmq::socket_t(context, ZMQ_PUB);
	std::string endpoint;
	const google::protobuf::Descriptor* descriptor = nullptr;
};
#endif

struct Client::ZMQImpl
{
#if defined(MVSIM_HAS_ZMQ)
	zmq::context_t context{1};
	std::optional<zmq::socket_t> mainReqSocket;

	std::map<std::string, InfoPerAdvertisedTopic> advertisedTopics;
	std::shared_mutex advertisedTopics_mtx;

#endif
};

Client::Client()
	: mrpt::system::COutputLogger("mvsim::Client"),
	  zmq_(std::make_unique<Client::ZMQImpl>())
{
}
Client::Client(const std::string& nodeName) : Client() { setName(nodeName); }

Client::~Client() { shutdown(); }

void Client::setName(const std::string& nodeName) { nodeName_ = nodeName; }

void Client::connect()
{
	using namespace std::string_literals;
	ASSERTMSG_(
		!zmq_->mainReqSocket || !zmq_->mainReqSocket->connected(),
		"Client is already running.");

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

	zmq_->mainReqSocket = zmq::socket_t(zmq_->context, ZMQ_REQ);
	zmq_->mainReqSocket->connect(
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

	if (!zmq_->mainReqSocket->connected()) return;

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
	zmq_->context.shutdown();
#else
	// Missing shutdown() in older versions:
	zmq_ctx_shutdown(zmq_->context.operator void*());
#endif

#endif
}

void Client::doRegisterClient()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	auto& s = *zmq_->mainReqSocket;

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
	auto& s = *zmq_->mainReqSocket;

	mvsim_msgs::UnregisterNodeRequest rnq;
	rnq.set_nodename(nodeName_);
	mvsim::sendMessage(rnq, s);

	//  Get the reply.
	const zmq::message_t reply = mvsim::receiveMessage(s);

	mvsim_msgs::GenericAnswer rna;
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
	auto& s = *zmq_->mainReqSocket;

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

void Client::doAdvertiseTopic(
	const std::string& topicName,
	const google::protobuf::Descriptor* descriptor)
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

	auto& advTopics = zmq_->advertisedTopics;

	std::unique_lock<std::shared_mutex> lck(zmq_->advertisedTopics_mtx);

	if (advTopics.find(topicName) != advTopics.end())
		THROW_EXCEPTION_FMT(
			"Topic `%s` already registered for publication in this same client "
			"(!)",
			topicName.c_str());

	// the ctor of InfoPerAdvertisedTopic automatically creates a ZMQ_PUB
	// socket in pubSocket
	InfoPerAdvertisedTopic& ipat =
		advTopics.emplace_hint(advTopics.begin(), topicName, zmq_->context)
			->second;

	lck.unlock();

	// Create PUBLISH socket:
	ipat.pubSocket.bind("tcp://0.0.0.0:*");
	if (!ipat.pubSocket.connected())
		THROW_EXCEPTION("Could not bind publisher socket");

	// Retrieve assigned TCP port:
	char assignedPort[100];
	size_t assignedPortLen = sizeof(assignedPort);
	ipat.pubSocket.getsockopt(
		ZMQ_LAST_ENDPOINT, assignedPort, &assignedPortLen);
	assignedPort[assignedPortLen] = '\0';

	ipat.endpoint = assignedPort;
	ipat.topicName = topicName;  // redundant in container, but handy.
	ipat.descriptor = descriptor;

	MRPT_LOG_DEBUG_FMT(
		"Advertising topic `%s` [%s] on endpoint `%s`", topicName.c_str(),
		descriptor->full_name().c_str(), ipat.endpoint.c_str());

	// MRPT_LOG_INFO_STREAM("Type: " << descriptor->DebugString());

	mvsim_msgs::AdvertiseTopicRequest req;
	req.set_topicname(ipat.topicName);
	req.set_endpoint(ipat.endpoint);
	req.set_topictypename(ipat.descriptor->full_name());
	req.set_nodename(nodeName_);

	mvsim::sendMessage(req, *zmq_->mainReqSocket);

	//  Get the reply.
	const zmq::message_t reply = mvsim::receiveMessage(*zmq_->mainReqSocket);
	mvsim_msgs::GenericAnswer ans;
	mvsim::parseMessage(reply, ans);

	if (!ans.success())
		THROW_EXCEPTION_FMT(
			"Error registering topic `%s` in server: `%s`",
			ans.errormessage().c_str());

#else
	THROW_EXCEPTION("MVSIM built without ZMQ & PROTOBUF");
#endif
}

void Client::doAdvertiseService(
	const std::string& topicName, const google::protobuf::Descriptor* descIn,
	const google::protobuf::Descriptor* descOut);
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

	MRPT_TODO("continue here");

	auto& advTopics = zmq_->advertisedTopics;

	std::unique_lock<std::shared_mutex> lck(zmq_->advertisedTopics_mtx);

	if (advTopics.find(topicName) != advTopics.end())
		THROW_EXCEPTION_FMT(
			"Topic `%s` already registered for publication in this same client "
			"(!)",
			topicName.c_str());

	// the ctor of InfoPerAdvertisedTopic automatically creates a ZMQ_PUB
	// socket in pubSocket
	InfoPerAdvertisedTopic& ipat =
		advTopics.emplace_hint(advTopics.begin(), topicName, zmq_->context)
			->second;

	lck.unlock();

	// Create PUBLISH socket:
	ipat.pubSocket.bind("tcp://0.0.0.0:*");
	if (!ipat.pubSocket.connected())
		THROW_EXCEPTION("Could not bind publisher socket");

	// Retrieve assigned TCP port:
	char assignedPort[100];
	size_t assignedPortLen = sizeof(assignedPort);
	ipat.pubSocket.getsockopt(
		ZMQ_LAST_ENDPOINT, assignedPort, &assignedPortLen);
	assignedPort[assignedPortLen] = '\0';

	ipat.endpoint = assignedPort;
	ipat.topicName = topicName;  // redundant in container, but handy.
	ipat.descriptor = descriptor;

	MRPT_LOG_DEBUG_FMT(
		"Advertising topic `%s` [%s] on endpoint `%s`", topicName.c_str(),
		descriptor->full_name().c_str(), ipat.endpoint.c_str());

	// MRPT_LOG_INFO_STREAM("Type: " << descriptor->DebugString());

	mvsim_msgs::AdvertiseTopicRequest req;
	req.set_topicname(ipat.topicName);
	req.set_endpoint(ipat.endpoint);
	req.set_topictypename(ipat.descriptor->full_name());
	req.set_nodename(nodeName_);

	mvsim::sendMessage(req, *zmq_->mainReqSocket);

	//  Get the reply.
	const zmq::message_t reply = mvsim::receiveMessage(*zmq_->mainReqSocket);
	mvsim_msgs::GenericAnswer ans;
	mvsim::parseMessage(reply, ans);

	if (!ans.success())
		THROW_EXCEPTION_FMT(
			"Error registering topic `%s` in server: `%s`",
			ans.errormessage().c_str());

#else
	THROW_EXCEPTION("MVSIM built without ZMQ & PROTOBUF");
#endif
}

void Client::publishTopic(
	const std::string& topicName, const google::protobuf::Message& msg)
{
	MRPT_START
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	ASSERTMSG_(
		zmq_ && zmq_->mainReqSocket && zmq_->mainReqSocket->connected(),
		"Client not connected to Server");

	std::shared_lock<std::shared_mutex> lck(zmq_->advertisedTopics_mtx);
	auto itIpat = zmq_->advertisedTopics.find(topicName);

	ASSERTMSG_(
		itIpat != zmq_->advertisedTopics.end(),
		mrpt::format(
			"Topic `%s` cannot been registered. Missing former call to "
			"advertiseTopic()?",
			topicName.c_str()));

	lck.unlock();

	auto& ipat = itIpat->second;

	ASSERTMSG_(
		msg.GetDescriptor() == ipat.descriptor,
		mrpt::format(
			"Topic `%s` has type `%s`, but expected `%s` from former call to "
			"advertiseTopic()?",
			topicName.c_str(), msg.GetDescriptor()->name().c_str(),
			ipat.descriptor->name().c_str()));

	ASSERT_(ipat.pubSocket.connected());

	mvsim::sendMessage(msg, ipat.pubSocket);

#if 0
	MRPT_LOG_DEBUG_FMT(
		"Published on topic `%s`: %s", topicName.c_str(),
		msg.DebugString().c_str());
#endif

#else
	THROW_EXCEPTION("MVSIM built without ZMQ & PROTOBUF");
#endif
	MRPT_END
}
