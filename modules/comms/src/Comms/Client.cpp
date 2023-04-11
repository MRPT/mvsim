/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/version.h>
#include <mvsim/Comms/Client.h>
#include <mvsim/Comms/common.h>
#include <mvsim/Comms/ports.h>
#include <mvsim/Comms/zmq_monitor.h>
#if MRPT_VERSION >= 0x204
#include <mrpt/system/thread_name.h>
#endif

#include <iostream>
#include <mutex>
#include <shared_mutex>

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

#include <google/protobuf/text_format.h>
#include <mvsim/mvsim-msgs/AdvertiseServiceRequest.pb.h>
#include <mvsim/mvsim-msgs/AdvertiseTopicRequest.pb.h>
#include <mvsim/mvsim-msgs/CallService.pb.h>
#include <mvsim/mvsim-msgs/GenericAnswer.pb.h>
#include <mvsim/mvsim-msgs/GetServiceInfoAnswer.pb.h>
#include <mvsim/mvsim-msgs/GetServiceInfoRequest.pb.h>
#include <mvsim/mvsim-msgs/ListNodesAnswer.pb.h>
#include <mvsim/mvsim-msgs/ListNodesRequest.pb.h>
#include <mvsim/mvsim-msgs/ListTopicsAnswer.pb.h>
#include <mvsim/mvsim-msgs/ListTopicsRequest.pb.h>
#include <mvsim/mvsim-msgs/RegisterNodeAnswer.pb.h>
#include <mvsim/mvsim-msgs/RegisterNodeRequest.pb.h>
#include <mvsim/mvsim-msgs/SubscribeAnswer.pb.h>
#include <mvsim/mvsim-msgs/SubscribeRequest.pb.h>
#include <mvsim/mvsim-msgs/UnregisterNodeRequest.pb.h>

#include <zmq.hpp>

#endif

using namespace mvsim;

#if defined(MVSIM_HAS_ZMQ)
namespace mvsim::internal
{
struct InfoPerAdvertisedTopic
{
	InfoPerAdvertisedTopic(zmq::context_t& c) : context(c) {}

	zmq::context_t& context;

	std::string topicName;
	zmq::socket_t pubSocket = zmq::socket_t(context, ZMQ_PUB);
	std::string endpoint;
	const google::protobuf::Descriptor* descriptor = nullptr;
};

struct InfoPerService
{
	InfoPerService() = default;

	std::string serviceName;
	const google::protobuf::Descriptor* descInput = nullptr;
	const google::protobuf::Descriptor* descOutput = nullptr;
	Client::service_callback_t callback;
};
struct InfoPerSubscribedTopic
{
	InfoPerSubscribedTopic(zmq::context_t& c) : context(c) {}
	~InfoPerSubscribedTopic()
	{
		if (topicThread.joinable()) topicThread.join();
	}

	zmq::context_t& context;

	std::string topicName;
	zmq::socket_t subSocket = zmq::socket_t(context, ZMQ_SUB);
	const google::protobuf::Descriptor* descriptor = nullptr;

	std::vector<Client::topic_callback_t> callbacks;

	std::thread topicThread;
};
}  // namespace mvsim::internal
#endif

struct Client::ZMQImpl
{
#if defined(MVSIM_HAS_ZMQ)
	zmq::context_t context{2 /* io sockets */, ZMQ_MAX_SOCKETS_DFLT};
	std::optional<zmq::socket_t> mainReqSocket;
	std::recursive_mutex mainReqSocketMtx;
	mvsim::SocketMonitor mainReqSocketMonitor;

	std::map<std::string, internal::InfoPerAdvertisedTopic> advertisedTopics;
	std::shared_mutex advertisedTopics_mtx;

	std::optional<zmq::socket_t> srvListenSocket;
	std::map<std::string, internal::InfoPerService> offeredServices;
	std::shared_mutex offeredServices_mtx;

	std::map<std::string, internal::InfoPerSubscribedTopic> subscribedTopics;
	std::shared_mutex subscribedTopics_mtx;

	std::optional<zmq::socket_t> topicNotificationsSocket;
	std::string topicNotificationsEndPoint;

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

bool Client::connected() const
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	return zmq_->mainReqSocketMonitor.connected();
#else
	return false;
#endif
}

void Client::connect()
{
	using namespace std::string_literals;
	ASSERTMSG_(
		!zmq_->mainReqSocket || !zmq_->mainReqSocket,
		"Client is already running.");

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

	mrpt::system::CTimeLoggerEntry tle(profiler_, "connect");

	auto lck = mrpt::lockHelper(zmq_->mainReqSocketMtx);

	zmq_->mainReqSocket.emplace(zmq_->context, ZMQ_REQ);

	// Monitor to listen on ZMQ socket events:
	zmq_->mainReqSocketMonitor.monitor(zmq_->mainReqSocket.value());

	zmq_->mainReqSocket->connect(
		"tcp://"s + serverHostAddress_ + ":"s +
		std::to_string(MVSIM_PORTNO_MAIN_REP));

	// Let the server know about this new node:
	doRegisterClient();

	// Create listening socket for services:
	zmq_->srvListenSocket.emplace(zmq_->context, ZMQ_REP);
	zmq_->srvListenSocket->bind("tcp://0.0.0.0:*"s);

	if (!zmq_->srvListenSocket)
		THROW_EXCEPTION("Error binding service listening socket.");

	ASSERTMSG_(
		!serviceInvokerThread_.joinable(),
		"Client service thread is already running!");

	serviceInvokerThread_ =
		std::thread(&Client::internalServiceServingThread, this);
#if MRPT_VERSION >= 0x204
	mrpt::system::thread_name("services_"s + nodeName_, serviceInvokerThread_);
#endif

	// Create listening socket for subscription updates:
	zmq_->topicNotificationsSocket.emplace(zmq_->context, ZMQ_PAIR);
	zmq_->topicNotificationsSocket->bind("tcp://0.0.0.0:*"s);

	if (!zmq_->topicNotificationsSocket)
		THROW_EXCEPTION("Error binding topic updates listening socket.");

	zmq_->topicNotificationsEndPoint =
		get_zmq_endpoint(*zmq_->topicNotificationsSocket);

	ASSERTMSG_(
		!topicUpdatesThread_.joinable(),
		"Client topic updates thread is already running!");

	topicUpdatesThread_ =
		std::thread(&Client::internalTopicUpdatesThread, this);
#if MRPT_VERSION >= 0x204
	mrpt::system::thread_name(
		"topicUpdates_"s + nodeName_, topicUpdatesThread_);
#endif

#else
	THROW_EXCEPTION(
		"MVSIM needs building with ZMQ and PROTOBUF to enable "
		"client/server");
#endif
}

void Client::shutdown() noexcept
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mrpt::system::CTimeLoggerEntry tle(profiler_, "shutdown");

	auto lck = mrpt::lockHelper(zmq_->mainReqSocketMtx);
	if (!zmq_->mainReqSocket) return;

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

#if CPPZMQ_VERSIONZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 4, 0)
	zmq_->context.shutdown();
#else
	// Missing shutdown() in older versions:
	zmq_ctx_shutdown(zmq_->context.operator void*());
#endif

	if (serviceInvokerThread_.joinable()) serviceInvokerThread_.join();
	if (topicUpdatesThread_.joinable()) topicUpdatesThread_.join();
	zmq_->subscribedTopics.clear();
	zmq_->offeredServices.clear();

#endif
}

void Client::doRegisterClient()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mrpt::system::CTimeLoggerEntry tle(profiler_, "doRegisterClient");

	auto lck = mrpt::lockHelper(zmq_->mainReqSocketMtx);
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
	mrpt::system::CTimeLoggerEntry tle(profiler_, "doUnregisterClient");

	auto lck = mrpt::lockHelper(zmq_->mainReqSocketMtx);
	if (!zmq_->mainReqSocket) return;
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
	mrpt::system::CTimeLoggerEntry tle(profiler_, "requestListOfNodes");

	auto lck = mrpt::lockHelper(zmq_->mainReqSocketMtx);
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

std::vector<Client::InfoPerTopic> Client::requestListOfTopics()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mrpt::system::CTimeLoggerEntry tle(profiler_, "requestListOfTopics");
	auto lck = mrpt::lockHelper(zmq_->mainReqSocketMtx);
	auto& s = *zmq_->mainReqSocket;

	mvsim_msgs::ListTopicsRequest req;
	mvsim::sendMessage(req, s);

	//  Get the reply.
	const zmq::message_t reply = mvsim::receiveMessage(s);

	mvsim_msgs::ListTopicsAnswer lta;
	mvsim::parseMessage(reply, lta);

	std::vector<Client::InfoPerTopic> topics;
	topics.resize(lta.topics_size());

	for (int i = 0; i < lta.topics_size(); i++)
	{
		const auto& t = lta.topics(i);
		auto& dst = topics[i];

		dst.name = t.topicname();
		dst.type = t.topictype();

		ASSERT_EQUAL_(t.publisherendpoint_size(), t.publishername_size());
		dst.endpoints.resize(t.publisherendpoint_size());
		dst.publishers.resize(t.publisherendpoint_size());

		for (int k = 0; k < t.publisherendpoint_size(); k++)
		{
			dst.publishers[k] = t.publishername(k);
			dst.endpoints[k] = t.publisherendpoint(k);
		}
	}
	return topics;
#else
	THROW_EXCEPTION("MVSIM built without ZMQ");
#endif
}

void Client::doAdvertiseTopic(
	const std::string& topicName,
	const google::protobuf::Descriptor* descriptor)
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mrpt::system::CTimeLoggerEntry tle(profiler_, "doAdvertiseTopic");
	auto& advTopics = zmq_->advertisedTopics;

	std::unique_lock<std::shared_mutex> lck(zmq_->advertisedTopics_mtx);

	if (advTopics.find(topicName) != advTopics.end())
		THROW_EXCEPTION_FMT(
			"Topic `%s` already registered for publication in this same "
			"client (!)",
			topicName.c_str());

	// the ctor of InfoPerAdvertisedTopic automatically creates a ZMQ_PUB
	// socket in pubSocket
	internal::InfoPerAdvertisedTopic& ipat =
		advTopics.emplace_hint(advTopics.begin(), topicName, zmq_->context)
			->second;

	lck.unlock();

	// Bind the PUBLISH socket:
	ipat.pubSocket.bind("tcp://0.0.0.0:*");

#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 7, 1)
	if (!ipat.pubSocket)
#else
	if (!ipat.pubSocket.connected())
#endif
	{
		THROW_EXCEPTION("Could not bind publisher socket");
	}

	// Retrieve assigned TCP port:
	ipat.endpoint = get_zmq_endpoint(ipat.pubSocket);
	ipat.topicName = topicName;	 // redundant in container, but handy.
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

	auto lckMain = mrpt::lockHelper(zmq_->mainReqSocketMtx);
	mvsim::sendMessage(req, *zmq_->mainReqSocket);

	//  Get the reply.
	const zmq::message_t reply = mvsim::receiveMessage(*zmq_->mainReqSocket);
	lckMain.unlock();

	mvsim_msgs::GenericAnswer ans;
	mvsim::parseMessage(reply, ans);

	if (!ans.success())
		THROW_EXCEPTION_FMT(
			"Error registering topic `%s` in server: `%s`", topicName.c_str(),
			ans.errormessage().c_str());

#else
	THROW_EXCEPTION("MVSIM built without ZMQ & PROTOBUF");
#endif
}

void Client::doAdvertiseService(
	const std::string& serviceName, const google::protobuf::Descriptor* descIn,
	const google::protobuf::Descriptor* descOut, service_callback_t callback)
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mrpt::system::CTimeLoggerEntry tle(profiler_, "doAdvertiseService");
	std::unique_lock<std::shared_mutex> lck(zmq_->offeredServices_mtx);

	auto& services = zmq_->offeredServices;

	if (services.find(serviceName) != services.end())
		THROW_EXCEPTION_FMT(
			"Service `%s` already registered in this same client!",
			serviceName.c_str());

	internal::InfoPerService& ips = services[serviceName];

	lck.unlock();

	// Retrieve assigned TCP port:
	const auto assignedPort = mvsim::get_zmq_endpoint(*zmq_->srvListenSocket);

	ips.serviceName = serviceName;	// redundant in container, but handy.
	ips.callback = callback;
	ips.descInput = descIn;
	ips.descOutput = descOut;

	MRPT_LOG_DEBUG_FMT(
		"Advertising service `%s` [%s->%s] on endpoint `%s`",
		serviceName.c_str(), descIn->full_name().c_str(),
		descOut->full_name().c_str(), assignedPort.c_str());

	mvsim_msgs::AdvertiseServiceRequest req;
	req.set_servicename(ips.serviceName);
	req.set_endpoint(assignedPort);
	req.set_inputtypename(ips.descInput->full_name());
	req.set_outputtypename(ips.descOutput->full_name());
	req.set_nodename(nodeName_);

	auto lckMain = mrpt::lockHelper(zmq_->mainReqSocketMtx);
	mvsim::sendMessage(req, *zmq_->mainReqSocket);

	//  Get the reply.
	const zmq::message_t reply = mvsim::receiveMessage(*zmq_->mainReqSocket);
	lckMain.unlock();
	mvsim_msgs::GenericAnswer ans;
	mvsim::parseMessage(reply, ans);

	if (!ans.success())
		THROW_EXCEPTION_FMT(
			"Error registering service `%s` in server: `%s`",
			serviceName.c_str(), ans.errormessage().c_str());

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
		zmq_ && zmq_->mainReqSocket && zmq_->mainReqSocket,
		"Client not connected to Server");
	mrpt::system::CTimeLoggerEntry tle(profiler_, "publishTopic");

	std::shared_lock<std::shared_mutex> lck(zmq_->advertisedTopics_mtx);
	auto itIpat = zmq_->advertisedTopics.find(topicName);

	ASSERTMSG_(
		itIpat != zmq_->advertisedTopics.end(),
		mrpt::format(
			"Topic `%s` has not been registered. Missing former call to "
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

#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 7, 1)
	ASSERT_(ipat.pubSocket);
#else
	ASSERT_(ipat.pubSocket.connected());
#endif

	mvsim::sendMessage(msg, ipat.pubSocket);

#if 0
	std::cout << "Published on topic " << topicName << std::endl;
#endif

#else
	THROW_EXCEPTION("MVSIM built without ZMQ & PROTOBUF");
#endif
	MRPT_END
}

void Client::internalServiceServingThread()
{
	using namespace std::string_literals;

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	try
	{
		MRPT_LOG_INFO_STREAM(
			"[" << nodeName_ << "] Client service thread started.");

		zmq::socket_t& s = *zmq_->srvListenSocket;

		for (;;)
		{
			//  Wait for next request from client:
			zmq::message_t m = mvsim::receiveMessage(s);

			// parse it:
			mvsim_msgs::CallService csMsg;
			mvsim::parseMessage(m, csMsg);

			std::shared_lock<std::shared_mutex> lck(zmq_->offeredServices_mtx);
			const auto& srvName = csMsg.servicename();

			auto itSrv = zmq_->offeredServices.find(srvName);
			if (itSrv == zmq_->offeredServices.end())
			{
				// Error: unknown service:
				mvsim_msgs::GenericAnswer ans;
				ans.set_success(false);
				ans.set_errormessage(mrpt::format(
					"Requested unknown service `%s`", srvName.c_str()));
				MRPT_LOG_ERROR_STREAM(ans.errormessage());

				mvsim::sendMessage(ans, s);
				continue;
			}

			internal::InfoPerService& ips = itSrv->second;

			// MRPT_TODO("Check input descriptor?");

			auto outMsgPtr = ips.callback(csMsg.serializedinput());

			// Send response:
			mvsim::sendMessage(*outMsgPtr, s);
		}
	}
	catch (const zmq::error_t& e)
	{
		if (e.num() == ETERM)
		{
			// This simply means someone called
			// requestMainThreadTermination(). Just exit silently.
			MRPT_LOG_DEBUG_STREAM(
				"internalServiceServingThread about to exit for ZMQ term "
				"signal.");
		}
		else
		{
			MRPT_LOG_ERROR_STREAM(
				"internalServiceServingThread: ZMQ error: " << e.what());
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"internalServiceServingThread: Exception: "
			<< mrpt::exception_to_str(e));
	}
	MRPT_LOG_DEBUG_STREAM("internalServiceServingThread quitted.");

#endif
}

void Client::internalTopicUpdatesThread()
{
	using namespace std::string_literals;

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	try
	{
		MRPT_LOG_DEBUG_STREAM(
			"[" << nodeName_ << "] Client topic updates thread started.");

		zmq::socket_t& s = *zmq_->topicNotificationsSocket;

		for (;;)
		{
			//  Wait for next update from server:
			zmq::message_t m = mvsim::receiveMessage(s);

			// parse it:
			mvsim_msgs::TopicInfo tiMsg;
			mvsim::parseMessage(m, tiMsg);

			// Let the server know that we received this:
			mvsim_msgs::GenericAnswer ans;
			ans.set_success(true);
			mvsim::sendMessage(ans, s);

			// We got a message. This means we have to new endpoints to
			// subscribe, either because we have just subscribed to a new topic,
			// or a new node has advertised a topic we already subscribed in the
			// past.

			// Look for the entry in the list of subscribed topics:
			std::unique_lock<std::shared_mutex> lck(zmq_->subscribedTopics_mtx);
			const auto& topicName = tiMsg.topicname();

			auto itTopic = zmq_->subscribedTopics.find(topicName);
			if (itTopic == zmq_->subscribedTopics.end())
			{
				// This shouldn't happen (?).
				MRPT_LOG_WARN_STREAM(
					"Received a topic `"
					<< topicName
					<< "` update message from server, but this node is not "
					   "subscribed to it (!).");
				continue;
			}

			MRPT_LOG_DEBUG_STREAM(
				"[internalTopicUpdatesThread] Received: "
				<< tiMsg.DebugString());

			internal::InfoPerSubscribedTopic& ipt = itTopic->second;

			for (int i = 0; i < tiMsg.publisherendpoint_size(); i++)
			{
				ipt.subSocket.connect(tiMsg.publisherendpoint(i));
			}

			// No need to send response back to server.
		}
	}
	catch (const zmq::error_t& e)
	{
		if (e.num() == ETERM)
		{
			// This simply means someone called
			// requestMainThreadTermination(). Just exit silently.
			MRPT_LOG_DEBUG_STREAM(
				"internalTopicUpdatesThread about to exit for ZMQ term "
				"signal.");
		}
		else
		{
			MRPT_LOG_ERROR_STREAM(
				"internalTopicUpdatesThread: ZMQ error: " << e.what());
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"internalTopicUpdatesThread: Exception: "
			<< mrpt::exception_to_str(e));
	}
	MRPT_LOG_DEBUG_STREAM(
		"[" << nodeName_ << "] Client topic updates thread quitted.");

#endif
}

// Overload for python wrapper
std::string Client::callService(
	const std::string& serviceName, const std::string& inputSerializedMsg)
{
	MRPT_START
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mrpt::system::CTimeLoggerEntry tle(profiler_, "callService");

	std::string outMsgData, outMsgType;
	doCallService(
		serviceName, inputSerializedMsg, std::nullopt, outMsgData, outMsgType);
	return outMsgData;
#endif
	MRPT_END
}
/// Overload for python wrapper
void Client::subscribeTopic(
	const std::string& topicName,
	const std::function<void(
		const std::string& /*msgType*/,
		const std::vector<uint8_t>& /*serializedMsg*/)>& callback)
{
	MRPT_START
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

	subscribe_topic_raw(topicName, [callback](const zmq::message_t& m) {
		const auto [sType, sData] = mvsim::internal::parseMessageToParts(m);
		std::vector<uint8_t> d(sData.size());
		::memcpy(d.data(), sData.data(), sData.size());
		callback(sType, d);
	});
#endif
	MRPT_END
}

void Client::doCallService(
	const std::string& serviceName, const std::string& inputSerializedMsg,
	mrpt::optional_ref<google::protobuf::Message> outputMsg,
	mrpt::optional_ref<std::string> outputSerializedMsg,
	mrpt::optional_ref<std::string> outputMsgTypeName)
{
	MRPT_START
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mrpt::system::CTimeLoggerEntry tle(profiler_, "doCallService");

	// 1) Request to the server who is serving this service:
	std::string srvEndpoint;

	auto lckCache = mrpt::lockHelper(serviceToEndPointCacheMtx_);
	if (auto it = serviceToEndPointCache_.find(serviceName);
		it != serviceToEndPointCache_.end())
	{
		srvEndpoint = it->second;
	}
	else
	{
		mrpt::system::CTimeLoggerEntry tle2(profiler_, "doCallService.getinfo");

		auto lckMain = mrpt::lockHelper(zmq_->mainReqSocketMtx);
		zmq::socket_t& s = *zmq_->mainReqSocket;

		mvsim_msgs::GetServiceInfoRequest gsi;
		gsi.set_servicename(serviceName);
		mvsim::sendMessage(gsi, s);

		auto m = mvsim::receiveMessage(s);
		mvsim_msgs::GetServiceInfoAnswer gsia;
		mvsim::parseMessage(m, gsia);

		if (!gsia.success())
			THROW_EXCEPTION_FMT(
				"Error requesting information about service `%s`: %s",
				serviceName.c_str(), gsia.errormessage().c_str());

		srvEndpoint = gsia.serviceendpoint();

		serviceToEndPointCache_[serviceName] = srvEndpoint;
	}
	lckCache.unlock();

	// 2) Connect to the service offerrer and request the execution:
	zmq::socket_t srvReqSock(zmq_->context, ZMQ_REQ);
	srvReqSock.connect(srvEndpoint);

	mvsim_msgs::CallService csMsg;
	csMsg.set_servicename(serviceName);
	csMsg.set_serializedinput(inputSerializedMsg);

	mvsim::sendMessage(csMsg, srvReqSock);

	const auto m = mvsim::receiveMessage(srvReqSock);
	if (outputMsg)
	{
		mvsim::parseMessage(m, outputMsg.value().get());
	}
	if (outputSerializedMsg)
	{
		const auto [typeName, serializedData] =
			internal::parseMessageToParts(m);

		outputSerializedMsg.value().get() = serializedData;
		if (outputMsgTypeName) outputMsgTypeName.value().get() = typeName;
	}
#endif
	MRPT_END
}

void Client::subscribe_topic_raw(
	const std::string& topicName, const topic_callback_t& callback)
{
	doSubscribeTopic(topicName, nullptr, callback);
}

void Client::doSubscribeTopic(
	const std::string& topicName,
	[[maybe_unused]] const google::protobuf::Descriptor* descriptor,
	const topic_callback_t& callback)
{
	MRPT_START
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mrpt::system::CTimeLoggerEntry tle(profiler_, "doSubscribeTopic");
	// Register in my internal DB:
	std::unique_lock<std::shared_mutex> lck(zmq_->subscribedTopics_mtx);

	auto& topics = zmq_->subscribedTopics;

	// It's ok to subscribe more than once:
	internal::InfoPerSubscribedTopic& ipt =
		topics.emplace_hint(topics.begin(), topicName, zmq_->context)->second;

	// subscribe to .recv() any message:
#if CPPZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 7, 1)
	ipt.subSocket.set(zmq::sockopt::subscribe, "");
#else
	ipt.subSocket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
#endif

	ipt.callbacks.push_back(callback);

	ipt.topicName = topicName;

	lck.unlock();

	ipt.topicThread =
		std::thread([&]() { this->internalTopicSubscribeThread(ipt); });

	// Let the server know about our interest in the topic:
	mvsim_msgs::SubscribeRequest subReq;
	subReq.set_topic(topicName);
	subReq.set_updatesendpoint(zmq_->topicNotificationsEndPoint);

	auto lckMain = mrpt::lockHelper(zmq_->mainReqSocketMtx);
	mvsim::sendMessage(subReq, *zmq_->mainReqSocket);

	const auto m = mvsim::receiveMessage(*zmq_->mainReqSocket);
	lckMain.unlock();
	mvsim_msgs::SubscribeAnswer subAns;
	mvsim::parseMessage(m, subAns);

	ASSERT_EQUAL_(subAns.topic(), topicName);
	ASSERT_(subAns.success());

	// That is... the rest will be done upon reception of messages from the
	// server on potential clients we should subscribe to.

#endif
	MRPT_END
}

void Client::internalTopicSubscribeThread(internal::InfoPerSubscribedTopic& ipt)
{
	using namespace std::string_literals;

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	try
	{
		MRPT_LOG_DEBUG_STREAM(
			"[" << nodeName_ << "] Client topic subscribe thread for `"
				<< ipt.topicName << "` started.");

		zmq::socket_t& s = ipt.subSocket;

		for (;;)
		{
			//  Wait for next update from server:
			const zmq::message_t m = mvsim::receiveMessage(s);

			// Send to subscriber callbacks:
			try
			{
				for (const auto& callback : ipt.callbacks) callback(m);
			}
			catch (const std::exception& e)
			{
				MRPT_LOG_ERROR_STREAM(
					"Exception in topic `"
					<< ipt.topicName
					<< "` subscription callback:" << mrpt::exception_to_str(e));
			}
		}
	}
	catch (const zmq::error_t& e)
	{
		if (e.num() == ETERM)
		{
			// This simply means someone called
			// requestMainThreadTermination(). Just exit silently.
			MRPT_LOG_DEBUG_STREAM(
				"[" << nodeName_
					<< "] Client topic subscribe thread about to exit for ZMQ "
					   "term signal.");
		}
		else
		{
			MRPT_LOG_ERROR_STREAM(
				"internalTopicSubscribeThread: ZMQ error: " << e.what());
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"internalTopicSubscribeThread: Exception: "
			<< mrpt::exception_to_str(e));
	}
	MRPT_LOG_DEBUG_STREAM(
		"[" << nodeName_ << "] Client topic subscribe thread quitted.");

#endif
}
