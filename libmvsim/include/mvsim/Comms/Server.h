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

#include <atomic>
#include <thread>

namespace zmq
{
class context_t;
}

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
	/** Launches the server in a parallel thread. */
	void start();

	/** Shutdowns the server. Blocks until the thread is stopped. There is no
	 * need to manually call this method, it is called upon destruction. */
	void shutdown() noexcept;
	/** @} */

   private:
	std::thread mainThread_;
	std::atomic<zmq::context_t*> mainThreadZMQcontext_ = nullptr;
	void requestMainThreadTermination();

	void internalServerThread();
};

}  // namespace mvsim
