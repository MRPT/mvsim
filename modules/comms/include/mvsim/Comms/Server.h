/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2023  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/core/Clock.h>
#include <mrpt/system/COutputLogger.h>
#include <mvsim/Comms/ports.h>
#include <mvsim/Comms/zmq_fwrds.h>

#include <atomic>
#include <set>
#include <shared_mutex>	 // read/write mutex
#include <thread>

#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)

#include <mvsim/mvsim-msgs/AdvertiseServiceRequest.pb.h>
#include <mvsim/mvsim-msgs/AdvertiseTopicRequest.pb.h>
#include <mvsim/mvsim-msgs/GetServiceInfoRequest.pb.h>
#include <mvsim/mvsim-msgs/ListNodesRequest.pb.h>
#include <mvsim/mvsim-msgs/ListTopicsRequest.pb.h>
#include <mvsim/mvsim-msgs/RegisterNodeRequest.pb.h>
#include <mvsim/mvsim-msgs/SubscribeRequest.pb.h>
#include <mvsim/mvsim-msgs/UnregisterNodeRequest.pb.h>

#endif

namespace mvsim
{
/** \addtogroup mvsim_comms_module
 * @{ */

class World;

/** This class creates a parallel thread and listens for incoming connections
 * from clients. This is the main hub for advertising and subscribing to topics.
 * Users should instance a class mvsim::Client (C++) or mvsim.Client (Python) to
 * interface with this server.
 *
 * Usage:
 *  - Instantiate a Server object.
 *  - Set all parameters as desired.
 *  - Call start(). It will return immediately.
 *  - The server will be working on the background as long as the object is not
 * destroyed.
 *  - Upon destruction, the server thread will be automatically shut-down.
 *
 * Messages and topics are described as Protobuf messages, and communications
 * are done via ZMQ sockets.
 *
 * See: https://mvsimulator.readthedocs.io/
 */
class Server : public mrpt::system::COutputLogger
{
   public:
	Server();
	~Server();

	/** @name Main mvsim Server API
	 * @{ */
	/** Launches the server in a parallel thread and returns immediately. */
	void start();

	/** Shutdowns the server. Blocks until the thread is stopped. There is no
	 * need to manually call this method, it is called upon destruction. */
	void shutdown() noexcept;
	/** @} */

	unsigned int listenningPort() const { return serverPortNo_; }
	void listenningPort(unsigned int port) { serverPortNo_ = port; }

   private:
	std::thread mainThread_;
	std::atomic<zmq::context_t*> mainThreadZMQcontext_ = nullptr;
	void requestMainThreadTermination();

	void internalServerThread();

	// ========= Message handlers ========
#if defined(MVSIM_HAS_ZMQ) && defined(MVSIM_HAS_PROTOBUF)
	void handle(const mvsim_msgs::RegisterNodeRequest& m, zmq::socket_t& s);
	void handle(const mvsim_msgs::UnregisterNodeRequest& m, zmq::socket_t& s);
	void handle(const mvsim_msgs::SubscribeRequest& m, zmq::socket_t& s);
	void handle(const mvsim_msgs::ListTopicsRequest& m, zmq::socket_t& s);
	void handle(const mvsim_msgs::ListNodesRequest& m, zmq::socket_t& s);
	void handle(const mvsim_msgs::AdvertiseTopicRequest& m, zmq::socket_t& s);
	void handle(const mvsim_msgs::AdvertiseServiceRequest& m, zmq::socket_t& s);
	void handle(const mvsim_msgs::GetServiceInfoRequest& m, zmq::socket_t& s);
#endif

	/** @name Database (db_*) about connected nodes, topics, etc.
	 * @{ */

	/** Adquired by all db_* functions; must be locked whenever any of the
	 * variables in this block is read/write. */
	mutable std::shared_mutex dbMutex;

	/** Remove all references to a given node name */
	void db_remove_node(const std::string& nodeName);

	/** Adds the given node  */
	void db_register_node(const std::string& nodeName);

	/** Adds a new publisher for a given topic */
	void db_advertise_topic(
		const std::string& topicName, const std::string& topicTypeName,
		const std::string& publisherEndpoint, const std::string& nodeName);

	/** Adds a new offer for a service */
	void db_advertise_service(
		const std::string& serviceName, const std::string& inputTypeName,
		const std::string& outputTypeName, const std::string& publisherEndpoint,
		const std::string& nodeName);

	/** \return true on success */
	bool db_get_service_info(
		const std::string& serviceName, std::string& publisherEndpoint,
		std::string& nodeName) const;

	void db_add_topic_subscriber(
		const std::string& topicName, const std::string& updatesEndPoint);

	/** Send to updatesEndPoint only, if given; otherwise, send to all
	 * subscribers */
	void send_topic_publishers_to_subscribed_clients(
		const std::string& topicName,
		const std::optional<std::string>& updatesEndPoint = std::nullopt);

	struct InfoPerNode
	{
		InfoPerNode(const std::string& name) : nodeName(name) {}

		const std::string nodeName;

		/** Time when the node connected to this server */
		const mrpt::Clock::time_point timeConnected = mrpt::Clock::now();

		std::set<std::string> advertisedTopics;
		std::set<std::string> subscribedTopics;
	};
	using node_name_t = std::string;
	std::map<node_name_t, InfoPerNode> connectedNodes_;

	using endpoint_t = std::string;

	struct InfoPerPublisher
	{
		InfoPerPublisher(
			const std::string& topic_name,
			const std::string& publisher_node_name,
			const std::string& publisher_endpoint)
			: topicName(topic_name),
			  publisherNodeName(publisher_node_name),
			  publisherEndpoint(publisher_endpoint)
		{
		}
		const std::string topicName;
		const std::string publisherNodeName;
		const endpoint_t publisherEndpoint;
	};

	struct InfoPerSubscriber
	{
		InfoPerSubscriber(
			const std::string& topic_name,
			const std::string& sub_updates_endpoint)
			: topicName(topic_name),
			  subscriberUpdatesEndpoint(sub_updates_endpoint)
		{
		}
		const std::string topicName;
		const endpoint_t subscriberUpdatesEndpoint;
	};

	struct InfoPerTopic
	{
		InfoPerTopic() = default;
		InfoPerTopic(
			const std::string& name, const std::string& topic_type_name)
			: topicName(name), topicTypeName(topic_type_name)
		{
		}
		std::string topicName, topicTypeName;

		std::map<node_name_t, InfoPerPublisher> publishers;
		std::map<endpoint_t, InfoPerSubscriber> subscribers;
	};
	using topic_name_t = std::string;
	std::map<topic_name_t, InfoPerTopic> knownTopics_;

	struct InfoPerService
	{
		InfoPerService() = default;
		InfoPerService(
			const std::string& name, const std::string& in_type_name,
			const std::string& out_type_name, const std::string& end_point,
			const std::string& node_name)
			: serviceName(name),
			  inputTypeName(in_type_name),
			  outputTypeName(out_type_name),
			  endpoint(end_point),
			  nodeName(node_name)
		{
		}
		std::string serviceName, inputTypeName, outputTypeName;
		std::string endpoint, nodeName;
	};
	using service_name_t = std::string;
	std::map<service_name_t, InfoPerService> knownServices_;

	/** @} */

	unsigned int serverPortNo_ = MVSIM_PORTNO_MAIN_REP;
};
/** @} */

}  // namespace mvsim
