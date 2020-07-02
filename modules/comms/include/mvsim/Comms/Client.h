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
#include <mvsim/Comms/zmq_fwrds.h>

#include <atomic>
#include <memory>
#include <thread>

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
	Client(const std::string& nodeName);
	~Client();

	/** @name Main mvsim client communication API
	 * @{ */
	void setName(const std::string& nodeName);

	/** Connects to the server in a parallel thread. */
	void connect();

	/** Shutdowns the communication thread. Blocks until the thread is stopped.
	 * There is no need to manually call this method, it is called upon
	 * destruction. */
	void shutdown() noexcept;

	template <typename T>
	void advertiseTopic(const std::string& topicName);

	template <typename INPUT_MSG_T, typename OUTPUT_MSG_T>
	void advertiseService(const std::string& serviceName);

	void publishTopic(
		const std::string& topicName, const google::protobuf::Message& msg);

	struct InfoPerNode
	{
		std::string name;
	};
	std::vector<InfoPerNode> requestListOfNodes();

	/** @} */

   private:
	struct ZMQImpl;
	std::unique_ptr<ZMQImpl> zmq_;

	std::string serverHostAddress_ = "localhost";

	std::string nodeName_ = "anonymous";

	void doRegisterClient();
	void doUnregisterClient();

	void doAdvertiseTopic(
		const std::string& topicName,
		const google::protobuf::Descriptor* descriptor);
	void doAdvertiseService(
		const std::string& topicName,
		const google::protobuf::Descriptor* descIn,
		const google::protobuf::Descriptor* descOut);
};

template <typename T>
void Client::advertiseTopic(const std::string& topicName)
{
	doAdvertiseTopic(topicName, T::descriptor());
}

template <typename INPUT_MSG_T, typename OUTPUT_MSG_T>
void Client::advertiseService(const std::string& serviceName)
{
	doAdvertiseService(
		serviceName, INPUT_MSG_T::descriptor(), OUTPUT_MSG_T::descriptor());
}

}  // namespace mvsim
