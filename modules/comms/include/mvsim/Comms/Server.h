/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2020  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/COutputLogger.h>
#include <mvsim/Comms/ports.h>
#include <mvsim/Comms/zmq_fwrds.h>

#include <atomic>
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

	unsigned int serverPortNo_ = MVSIM_PORTNO_MAIN_REP;
};

}  // namespace mvsim
