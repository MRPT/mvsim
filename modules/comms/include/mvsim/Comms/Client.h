/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <mvsim/Comms/common.h>
#include <mvsim/Comms/zmq_fwrds.h>

#include <atomic>
#include <memory>
#include <thread>

namespace mvsim::internal
{
struct InfoPerService;
struct InfoPerSubscribedTopic;
}  // namespace mvsim::internal

namespace mvsim
{
/** This is the connection of any user program with the MVSIM server, so
 * it can advertise and subscribe to topics and use remote services.
 *
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
 *
 * \ingroup mvsim_comms_module
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

	const std::string& serverHostAddress() const { return serverHostAddress_; }
	void serverHostAddress(const std::string& serverIpOrAddressName)
	{
		ASSERT_(!connected());
		serverHostAddress_ = serverIpOrAddressName;
	}

	/** Connects to the server in a parallel thread.
	 *  Default server address is `localhost`, can be changed with
	 * serverHostAddress().
	 */
	void connect();

	/** Whether the client is correctly connected to the server. */
	bool connected() const;

	/** Shutdowns the communication thread. Blocks until the thread is stopped.
	 * There is no need to manually call this method, it is called upon
	 * destruction. */
	void shutdown() noexcept;

	template <typename T>
	void advertiseTopic(const std::string& topicName);

	void publishTopic(
		const std::string& topicName, const google::protobuf::Message& msg);

	template <typename MSG_T>
	void subscribeTopic(
		const std::string& topicName,
		const std::function<void(const MSG_T&)>& callback);

	template <typename INPUT_MSG_T, typename OUTPUT_MSG_T>
	void advertiseService(
		const std::string& serviceName,
		const std::function<OUTPUT_MSG_T(const INPUT_MSG_T&)>& callback);

	template <typename INPUT_MSG_T, typename OUTPUT_MSG_T>
	void callService(
		const std::string& serviceName, const INPUT_MSG_T& input,
		OUTPUT_MSG_T& output);

	/// Overload for python wrapper
	std::string callService(
		const std::string& serviceName, const std::string& inputSerializedMsg);
	/// Overload for python wrapper
	void subscribeTopic(
		const std::string& topicName,
		const std::function<void(const std::string& /*serializedMsg*/)>&
			callback);

	struct InfoPerNode
	{
		std::string name;
	};
	std::vector<InfoPerNode> requestListOfNodes();

	struct InfoPerTopic
	{
		std::string name;
		std::string type;
		std::vector<std::string> endpoints, publishers;
	};
	std::vector<InfoPerTopic> requestListOfTopics();

	using topic_callback_t = std::function<void(const zmq::message_t& /*m*/)>;

	void subscribe_topic_raw(
		const std::string& topicName, const topic_callback_t& callback);

	/** @} */

	const mrpt::system::CTimeLogger& profiler() const { return profiler_; }
	void enable_profiler(bool enable) { profiler_.enable(enable); }

   private:
	struct ZMQImpl;
	std::unique_ptr<ZMQImpl> zmq_;

	std::string serverHostAddress_ = "localhost";
	std::string nodeName_ = "anonymous";

	std::thread serviceInvokerThread_;
	std::thread topicUpdatesThread_;

	mrpt::system::CTimeLogger profiler_{false, "mvsim::Client"};

	void doRegisterClient();
	void doUnregisterClient();

	void internalServiceServingThread();
	void internalTopicUpdatesThread();
	void internalTopicSubscribeThread(internal::InfoPerSubscribedTopic& ipt);

	using service_callback_t =
		std::function<std::shared_ptr<google::protobuf::Message>(
			const std::string& /*inAsString*/)>;

	void doAdvertiseTopic(
		const std::string& topicName,
		const google::protobuf::Descriptor* descriptor);
	void doAdvertiseService(
		const std::string& serviceName,
		const google::protobuf::Descriptor* descIn,
		const google::protobuf::Descriptor* descOut,
		service_callback_t callback);

	void doSubscribeTopic(
		const std::string& topicName,
		const google::protobuf::Descriptor* descriptor,
		const topic_callback_t& callback);
	void doCallService(
		const std::string& serviceName, const std::string& inputSerializedMsg,
		mrpt::optional_ref<google::protobuf::Message> outputMsg,
		mrpt::optional_ref<std::string> outputSerializedMsg = std::nullopt,
		mrpt::optional_ref<std::string> outputMsgTypeName = std::nullopt);

	friend struct internal::InfoPerService;
	friend struct internal::InfoPerSubscribedTopic;
};

template <typename T>
void Client::advertiseTopic(const std::string& topicName)
{
	doAdvertiseTopic(topicName, T::descriptor());
}

template <typename INPUT_MSG_T, typename OUTPUT_MSG_T>
void Client::advertiseService(
	const std::string& serviceName,
	const std::function<OUTPUT_MSG_T(const INPUT_MSG_T&)>& callback)
{
	doAdvertiseService(
		serviceName, INPUT_MSG_T::descriptor(), OUTPUT_MSG_T::descriptor(),
		service_callback_t([callback](const std::string& inData) {
			INPUT_MSG_T in;
			in.ParseFromString(inData);
			return std::make_shared<OUTPUT_MSG_T>(callback(in));
		}));
}

template <typename MSG_T>
void Client::subscribeTopic(
	const std::string& topicName,
	const std::function<void(const MSG_T&)>& callback)
{
	doSubscribeTopic(
		topicName, MSG_T::descriptor(),
		topic_callback_t([callback](const zmq::message_t& m) {
			MSG_T in;
			mvsim::parseMessage(m, in);
			callback(in);
		}));
}

template <typename INPUT_MSG_T, typename OUTPUT_MSG_T>
void Client::callService(
	const std::string& serviceName, const INPUT_MSG_T& input,
	OUTPUT_MSG_T& output)
{
	doCallService(serviceName, input.SerializeAsString(), output);
}

}  // namespace mvsim
