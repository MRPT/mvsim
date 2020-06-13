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
/** This is the connection of any user program with the MVSIM server, so
 * it can advertise and subscribe to topics.
 * Users should instance a class mvsim::Client (C++) or mvsim.Client (Python) to
 * communicate with the simulation runnin in mvsim::World or any other module.
 *
 * Usage:
 *  - Instantiate a Client object.
 *  - Call connect(). It will return immediately.
 *  - The client will be working on the background as long as the object is not
 * destroyed.
 *
 * Messages and topics are described as Protobuf messages, and communications
 * are done via ZMQ sockets.
 *
 * See: https://mvsimulator.readthedocs.io/
 */
class Client : public mrpt::system::COutputLogger
{
   public:
	Client();
	~Client();

	/** @name Main mvsim client communication API
	 * @{ */

	/** Connects to the server in a parallel thread. */
	void connect();

	/** Shutdowns the communication thread. Blocks until the thread is stopped.
	 * There is no need to manually call this method, it is called upon
	 * destruction. */
	void shutdown() noexcept;

	/** @} */

   private:
	std::string serverHostAddress_ = "localhost";

	std::string nodeName_ = "anonymous";

	std::thread mainThread_;
	std::atomic<zmq::context_t*> mainThreadZMQcontext_ = nullptr;
	void requestMainThreadTermination();

	void internalClientThread();
};

}  // namespace mvsim