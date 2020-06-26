/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
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

#include "ListNodesRequest.pb.h"
#include "ListTopicsRequest.pb.h"
#include "RegisterNodeRequest.pb.h"
#include "SubscribeRequest.pb.h"
#include "UnregisterNodeRequest.pb.h"

#endif

namespace mvsim
{
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
#endif

	/** @name Database (db_*) about connected nodes, topics, etc.
	 * @{ */

	/** Adquired by all db_* functions; must be locked whenever any of the
	 * variables in this block is read/write. */
	std::shared_mutex dbMutex;

	/** Remove all references to a given node name */
	void db_remove_node(const std::string& nodeName);

	/** Adds the given node  */
	void db_register_node(const std::string& nodeName);

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

	struct InfoPerPublisher
	{
		InfoPerPublisher(
			const std::string& topic_name,
			const std::string& publisher_node_name)
			: topicName(topic_name), publisherNodeName(publisher_node_name)
		{
		}
		const std::string topicName;
		const std::string publisherNodeName;

		std::string publisherIPAddressAndPort;
	};

	struct InfoPerTopic
	{
		InfoPerTopic(const std::string& name) : topicName(name) {}
		const std::string topicName;

		std::map<node_name_t, InfoPerPublisher> publishers;
	};
	using topic_name_t = std::string;
	std::map<topic_name_t, InfoPerTopic> knownTopics_;

	/** @} */

	unsigned int serverPortNo_ = MVSIM_PORTNO_MAIN_REP;
};

}  // namespace mvsim
