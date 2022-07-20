/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <mrpt/core/exceptions.h>
#include <mrpt/version.h>
#include <mvsim/Comms/Server.h>
#include <mvsim/Comms/common.h>
#if MRPT_VERSION >= 0x204
#include <mrpt/system/thread_name.h>
#endif

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#include <mvsim/mvsim-msgs/GenericAnswer.pb.h>
#include <mvsim/mvsim-msgs/GetServiceInfoAnswer.pb.h>
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

Server::Server() : mrpt::system::COutputLogger("mvsim::Server") {}

Server::~Server() { shutdown(); }

void Server::start()
{
	ASSERTMSG_(!mainThread_.joinable(), "Server is already running.");

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	requestMainThreadTermination();
	mainThread_ = std::thread(&Server::internalServerThread, this);

#if MRPT_VERSION >= 0x204
	mrpt::system::thread_name("serverMain", mainThread_);
#endif

#else
	THROW_EXCEPTION(
		"MVSIM needs building with ZMQ and PROTOBUF to enable client/server");
#endif
}

void Server::shutdown() noexcept
{
	try
	{
		MRPT_LOG_DEBUG_STREAM("Waiting for the thread to quit.");
		requestMainThreadTermination();

		if (mainThread_.joinable()) mainThread_.join();

		MRPT_LOG_DEBUG_STREAM("Joined thread.");
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"shutdown: Exception: " << mrpt::exception_to_str(e));
	}
}

void Server::internalServerThread()
{
	using namespace std::string_literals;

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	try
	{
		MRPT_LOG_INFO_STREAM("Server thread started.");

		zmq::context_t context(1);
		mainThreadZMQcontext_ = &context;

		zmq::socket_t mainRepSocket(context, ZMQ_REP);
		mainRepSocket.bind("tcp://*:"s + std::to_string(serverPortNo_));

		for (;;)
		{
			zmq::message_t request;

			//  Wait for next request from client
#if ZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 3, 1)
			std::optional<size_t> reqSize = mainRepSocket.recv(request);
			ASSERT_(reqSize.has_value());
#else
			mainRepSocket.recv(&request);
#endif

			// Variant with all valid client requests:
			using client_requests_t = std::variant<
				mvsim_msgs::RegisterNodeRequest,
				mvsim_msgs::UnregisterNodeRequest, mvsim_msgs::SubscribeRequest,
				mvsim_msgs::ListNodesRequest, mvsim_msgs::ListTopicsRequest,
				mvsim_msgs::AdvertiseTopicRequest,
				mvsim_msgs::AdvertiseServiceRequest,
				mvsim_msgs::GetServiceInfoRequest>;

			// Parse and dispatch:
			try
			{
				client_requests_t req =
					mvsim::parseMessageVariant<client_requests_t>(request);

				std::visit(
					overloaded{
						[&](const auto& m) { this->handle(m, mainRepSocket); },
					},
					req);
			}
			catch (const UnexpectedMessageException& e)
			{
				MRPT_LOG_ERROR_STREAM(e.what());
			}
		}
	}
	catch (const zmq::error_t& e)
	{
		if (e.num() == ETERM)
		{
			// This simply means someone called requestMainThreadTermination().
			// Just exit silently.
			MRPT_LOG_DEBUG_STREAM(
				"Server thread about to exit for ZMQ term signal.");
		}
		else
		{
			MRPT_LOG_ERROR_STREAM(
				"internalServerThread: ZMQ error: " << e.what());
		}
	}
	catch (const std::exception& e)
	{
		MRPT_LOG_ERROR_STREAM(
			"internalServerThread: Exception: " << mrpt::exception_to_str(e));
	}
	MRPT_LOG_DEBUG_STREAM("Server thread quitted.");

	mainThreadZMQcontext_ = nullptr;
#endif
}

void Server::requestMainThreadTermination()
{
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	zmq::context_t* ctx = mainThreadZMQcontext_;
	if (ctx)
	{
#if ZMQ_VERSION >= ZMQ_MAKE_VERSION(4, 4, 0)
		ctx->shutdown();
#else
		// Missing shutdown() in older versions:
		zmq_ctx_shutdown(ctx->operator void*());
#endif
	}
#endif
}

void Server::db_remove_node(const std::string& nodeName)
{
	std::unique_lock lck(dbMutex);

	auto itNode = connectedNodes_.find(nodeName);
	if (itNode == connectedNodes_.end()) return;  // Nothing to do

	for (const std::string& topic : itNode->second.advertisedTopics)
	{
		auto itTopic = knownTopics_.find(topic);
		if (itTopic == knownTopics_.end()) continue;
		knownTopics_.erase(itTopic);
	}

	// for (const std::string& topic : itNode->second.subscribedTopics) { }

	connectedNodes_.erase(itNode);
}

void Server::db_register_node(const std::string& nodeName)
{
	std::unique_lock lck(dbMutex);

	connectedNodes_.emplace_hint(connectedNodes_.end(), nodeName, nodeName);
}

void Server::db_advertise_topic(
	const std::string& topicName, const std::string& topicTypeName,
	const std::string& publisherEndpoint, const std::string& nodeName)
{
	std::unique_lock lck(dbMutex);

	// 1) Add as a source of this topic:
	auto& dbTopic = knownTopics_[topicName];

	if (!dbTopic.topicTypeName.empty() &&
		dbTopic.topicTypeName != topicTypeName)
	{
		throw std::runtime_error(mrpt::format(
			"Trying to register topic `%s` [%s] but already known with type "
			"[%s]",
			topicName.c_str(), topicTypeName.c_str(),
			dbTopic.topicTypeName.c_str()));
	}
	dbTopic.topicName = topicName;
	dbTopic.topicTypeName = topicTypeName;

	dbTopic.publishers.try_emplace(
		nodeName, topicName, nodeName, publisherEndpoint);

	// 2) If clients are already waiting for this topic, inform them so they
	// can subscribe to this new source of data:
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
#endif
	MRPT_TODO("TO-DO");
}

void Server::db_add_topic_subscriber(
	const std::string& topicName, const std::string& updatesEndPoint)
{
	std::unique_lock lck(dbMutex);

	auto& dbTopic = knownTopics_[topicName];

	dbTopic.subscribers.try_emplace(
		updatesEndPoint, topicName, updatesEndPoint);

	// Send all currently-existing publishers:
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	mvsim_msgs::TopicInfo tiMsg;
	tiMsg.set_topicname(topicName);
	tiMsg.set_topictype(dbTopic.topicTypeName);

	for (const auto& pub : dbTopic.publishers)
	{
		tiMsg.add_publishername(pub.second.publisherNodeName);
		tiMsg.add_publisherendpoint(pub.second.publisherEndpoint);
	}

	ASSERT_(mainThreadZMQcontext_);
	zmq::socket_t s(*mainThreadZMQcontext_, ZMQ_PAIR);
	s.connect(updatesEndPoint);
	ASSERT_(s.connected());
	sendMessage(tiMsg, s);

	mvsim_msgs::GenericAnswer ans;
	const auto m = receiveMessage(s);
	mvsim::parseMessage(m, ans);
	ASSERT_(ans.success());

#endif
}

void Server::db_advertise_service(
	const std::string& serviceName, const std::string& inputTypeName,
	const std::string& outputTypeName, const std::string& publisherEndpoint,
	const std::string& nodeName)
{
	std::unique_lock lck(dbMutex);

	// 1) Add as a source of this topic:
	auto& dbSrv = knownServices_[serviceName];

	if (!dbSrv.inputTypeName.empty() &&
		(dbSrv.inputTypeName != inputTypeName ||
		 dbSrv.outputTypeName != outputTypeName))
	{
		throw std::runtime_error(mrpt::format(
			"Trying to register service `%s` [%s->%s] but already known with "
			"types "
			"[%s->%s]",
			serviceName.c_str(), inputTypeName.c_str(), outputTypeName.c_str(),
			dbSrv.inputTypeName.c_str(), dbSrv.outputTypeName.c_str()));
	}
	dbSrv.serviceName = serviceName;
	dbSrv.inputTypeName = inputTypeName;
	dbSrv.outputTypeName = outputTypeName;
	dbSrv.endpoint = publisherEndpoint;
	dbSrv.nodeName = nodeName;
}

bool Server::db_get_service_info(
	const std::string& serviceName, std::string& publisherEndpoint,
	std::string& nodeName) const
{
	std::shared_lock lck(dbMutex);

	// 1) Add as a source of this topic:
	auto itSrv = knownServices_.find(serviceName);
	if (itSrv == knownServices_.end()) return false;

	auto& dbSrv = itSrv->second;

	publisherEndpoint = dbSrv.endpoint;
	nodeName = dbSrv.nodeName;

	return true;
}

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

// mvsim_msgs::RegisterNodeRequest
void Server::handle(const mvsim_msgs::RegisterNodeRequest& m, zmq::socket_t& s)
{
	//  Send reply back to client
	MRPT_LOG_DEBUG_STREAM(
		"Registering new node named '" << m.nodename() << "'");

	// Make sure we don't have already a node named like this:
	// Don't raise an error if the name was already registered, since it might
	// be that the same node disconnected and connected again:
	db_remove_node(m.nodename());

	db_register_node(m.nodename());

	mvsim_msgs::RegisterNodeAnswer rna;
	rna.set_success(true);
	mvsim::sendMessage(rna, s);
}

// mvsim_msgs::UnregisterNodeRequest
void Server::handle(
	const mvsim_msgs::UnregisterNodeRequest& m, zmq::socket_t& s)
{
	//  Send reply back to client
	MRPT_LOG_DEBUG_STREAM("Unregistering node named '" << m.nodename() << "'");

	db_remove_node(m.nodename());

	mvsim_msgs::GenericAnswer rna;
	rna.set_success(true);
	mvsim::sendMessage(rna, s);
}

// mvsim_msgs::SubscribeRequest
void Server::handle(const mvsim_msgs::SubscribeRequest& m, zmq::socket_t& s)
{
	//  Send reply back to client
	MRPT_LOG_DEBUG_STREAM(
		"Subscription request for topic " << m.topic() << "'");

	// Include in our DB of subscriptions:
	// This also sends the subcriber the list of existing endpoints it must
	// subscribe to:
	db_add_topic_subscriber(m.topic(), m.updatesendpoint());

	mvsim_msgs::SubscribeAnswer ans;
	ans.set_topic(m.topic());
	ans.set_success(true);
	mvsim::sendMessage(ans, s);
}

// mvsim_msgs::GetServiceInfoRequest
void Server::handle(
	const mvsim_msgs::GetServiceInfoRequest& m, zmq::socket_t& s)
{
	//  Send reply back to client
	MRPT_LOG_DEBUG_STREAM(
		"GetServiceInfo request for service '" << m.servicename() << "'");

	mvsim_msgs::GetServiceInfoAnswer ans;
	std::string node, endpoint;

	if (db_get_service_info(m.servicename(), endpoint, node))
	{
		ans.set_success(true);
		ans.set_serviceendpoint(endpoint);
		ans.set_servicenodename(node);
	}
	else
	{
		ans.set_success(false);
		ans.set_errormessage(mrpt::format(
			"Could not find service `%s`", m.servicename().c_str()));
	}

	mvsim::sendMessage(ans, s);
}

// mvsim_msgs::ListTopicsRequest
void Server::handle(const mvsim_msgs::ListTopicsRequest& m, zmq::socket_t& s)
{
	//  Send reply back to client
	MRPT_LOG_DEBUG("Listing topics request");

	mvsim_msgs::ListTopicsAnswer ans;

	// Optional name filter:
	const auto& queryPrefix = m.topicstartswith();

	std::shared_lock lck(dbMutex);

	for (const auto& kv : knownTopics_)
	{
		const auto& t = kv.second;
		const auto& name = t.topicName;

		if (!queryPrefix.empty() ||
			name.substr(0, queryPrefix.size()) == queryPrefix)
		{
			auto tInfo = ans.add_topics();
			tInfo->set_topicname(name);
			tInfo->set_topictype(t.topicTypeName);

			for (const auto& pubs : t.publishers)
			{
				tInfo->add_publishername(pubs.second.publisherNodeName);
				tInfo->add_publisherendpoint(pubs.second.publisherEndpoint);
			}
		}
	}
	mvsim::sendMessage(ans, s);
}

// mvsim_msgs::ListNodesRequest
void Server::handle(const mvsim_msgs::ListNodesRequest& m, zmq::socket_t& s)
{
	//  Send reply back to client
	MRPT_LOG_DEBUG("Listing nodes request");

	// Optional name filter:
	const auto& queryPrefix = m.nodestartswith();

	mvsim_msgs::ListNodesAnswer ans;
	for (const auto& n : connectedNodes_)
	{
		const auto& name = n.second.nodeName;

		if (!queryPrefix.empty() ||
			name.substr(0, queryPrefix.size()) == queryPrefix)
		{
			ans.add_nodes(name);
		}
	}
	mvsim::sendMessage(ans, s);
}

// mvsim_msgs::AdvertiseTopicRequest
void Server::handle(
	const mvsim_msgs::AdvertiseTopicRequest& m, zmq::socket_t& s)
{
	//  Send reply back to client
	MRPT_LOG_DEBUG_FMT(
		"Received new topic advertiser: `%s` [%s] @ %s (%s)",
		m.topicname().c_str(), m.topictypename().c_str(), m.endpoint().c_str(),
		m.nodename().c_str());

	mvsim_msgs::GenericAnswer ans;
	try
	{
		db_advertise_topic(
			m.topicname(), m.topictypename(), m.endpoint(), m.nodename());
		ans.set_success(true);
	}
	catch (const std::exception& e)
	{
		ans.set_success(false);
		ans.set_errormessage(mrpt::exception_to_str(e));
	}
	mvsim::sendMessage(ans, s);
}

// mvsim_msgs::AdvertiseServiceRequest
void Server::handle(
	const mvsim_msgs::AdvertiseServiceRequest& m, zmq::socket_t& s)
{
	//  Send reply back to client
	MRPT_LOG_DEBUG_FMT(
		"Received new service offering: `%s` [%s->%s] @ %s (%s)",
		m.servicename().c_str(), m.inputtypename().c_str(),
		m.outputtypename().c_str(), m.endpoint().c_str(), m.nodename().c_str());

	mvsim_msgs::GenericAnswer ans;
	try
	{
		db_advertise_service(
			m.servicename(), m.inputtypename(), m.outputtypename(),
			m.endpoint(), m.nodename());
		ans.set_success(true);
	}
	catch (const std::exception& e)
	{
		ans.set_success(false);
		ans.set_errormessage(mrpt::exception_to_str(e));
	}
	mvsim::sendMessage(ans, s);
}

#endif
