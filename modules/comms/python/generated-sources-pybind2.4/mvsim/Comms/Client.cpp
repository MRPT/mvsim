#include <functional>
#include <iterator>
#include <memory>
#include <mrpt/system/CTimeLogger.h>
#include <mvsim/Comms/Client.h>
#include <sstream> // __str__
#include <string>
#include <string_view>

#include <functional>
#include <pybind11/pybind11.h>
#include <string>
#include <pybind11/stl.h>


#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mvsim_Comms_Client(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mvsim::Client file:mvsim/Comms/Client.h line:48
		pybind11::class_<mvsim::Client, std::shared_ptr<mvsim::Client>> cl(M("mvsim"), "Client", "This is the connection of any user program with the MVSIM server, so\n it can advertise and subscribe to topics and use remote services.\n\n Users should instance a class mvsim::Client (C++) or mvsim.Client (Python) to\n communicate with the simulation runnin in mvsim::World or any other module.\n\n Usage:\n  - Instantiate a Client object.\n  - Call connect(). It will return immediately.\n  - The client will be working on the background as long as the object is not\n destroyed.\n\n Messages and topics are described as Protobuf messages, and communications\n are done via ZMQ sockets.\n\n See: https://mvsimulator.readthedocs.io/\n\n \n\n ");
		cl.def( pybind11::init( [](){ return new mvsim::Client(); } ) );
		cl.def( pybind11::init<const std::string &>(), pybind11::arg("nodeName") );

		cl.def("setName", (void (mvsim::Client::*)(const std::string &)) &mvsim::Client::setName, "@{ \n\nC++: mvsim::Client::setName(const std::string &) --> void", pybind11::arg("nodeName"));
		cl.def("serverHostAddress", (const std::string & (mvsim::Client::*)() const) &mvsim::Client::serverHostAddress, "C++: mvsim::Client::serverHostAddress() const --> const std::string &", pybind11::return_value_policy::automatic);
		cl.def("serverHostAddress", (void (mvsim::Client::*)(const std::string &)) &mvsim::Client::serverHostAddress, "C++: mvsim::Client::serverHostAddress(const std::string &) --> void", pybind11::arg("serverIpOrAddressName"));
		cl.def("connect", (void (mvsim::Client::*)()) &mvsim::Client::connect, "Connects to the server in a parallel thread.\n  Default server address is `localhost`, can be changed with\n serverHostAddress().\n\nC++: mvsim::Client::connect() --> void");
		cl.def("connected", (bool (mvsim::Client::*)() const) &mvsim::Client::connected, "Whether the client is correctly connected to the server. \n\nC++: mvsim::Client::connected() const --> bool");
		cl.def("shutdown", (void (mvsim::Client::*)()) &mvsim::Client::shutdown, "Shutdowns the communication thread. Blocks until the thread is stopped.\n There is no need to manually call this method, it is called upon\n destruction. \n\nC++: mvsim::Client::shutdown() --> void");
		cl.def("callService", (std::string (mvsim::Client::*)(const std::string &, const std::string &)) &mvsim::Client::callService, "Overload for python wrapper\n\nC++: mvsim::Client::callService(const std::string &, const std::string &) --> std::string", pybind11::arg("serviceName"), pybind11::arg("inputSerializedMsg"));
		cl.def("subscribeTopic", (void (mvsim::Client::*)(const std::string &, const class std::function<void (const std::string &)> &)) &mvsim::Client::subscribeTopic, "Overload for python wrapper\n\nC++: mvsim::Client::subscribeTopic(const std::string &, const class std::function<void (const std::string &)> &) --> void", pybind11::arg("topicName"), pybind11::arg("callback"));
		cl.def("subscribe_topic_raw", (void (mvsim::Client::*)(const std::string &, const class std::function<void (const class zmq::message_t &)> &)) &mvsim::Client::subscribe_topic_raw, "C++: mvsim::Client::subscribe_topic_raw(const std::string &, const class std::function<void (const class zmq::message_t &)> &) --> void", pybind11::arg("topicName"), pybind11::arg("callback"));
		cl.def("enable_profiler", (void (mvsim::Client::*)(bool)) &mvsim::Client::enable_profiler, "C++: mvsim::Client::enable_profiler(bool) --> void", pybind11::arg("enable"));

		{ // mvsim::Client::InfoPerNode file:mvsim/Comms/Client.h line:111
			auto & enclosing_class = cl;
			pybind11::class_<mvsim::Client::InfoPerNode, std::shared_ptr<mvsim::Client::InfoPerNode>> cl(enclosing_class, "InfoPerNode", "");
			cl.def( pybind11::init( [](){ return new mvsim::Client::InfoPerNode(); } ) );
			cl.def( pybind11::init( [](mvsim::Client::InfoPerNode const &o){ return new mvsim::Client::InfoPerNode(o); } ) );
			cl.def_readwrite("name", &mvsim::Client::InfoPerNode::name);
		}

		{ // mvsim::Client::InfoPerTopic file:mvsim/Comms/Client.h line:117
			auto & enclosing_class = cl;
			pybind11::class_<mvsim::Client::InfoPerTopic, std::shared_ptr<mvsim::Client::InfoPerTopic>> cl(enclosing_class, "InfoPerTopic", "");
			cl.def( pybind11::init( [](){ return new mvsim::Client::InfoPerTopic(); } ) );
			cl.def( pybind11::init( [](mvsim::Client::InfoPerTopic const &o){ return new mvsim::Client::InfoPerTopic(o); } ) );
			cl.def_readwrite("name", &mvsim::Client::InfoPerTopic::name);
			cl.def_readwrite("type", &mvsim::Client::InfoPerTopic::type);
			cl.def_readwrite("endpoints", &mvsim::Client::InfoPerTopic::endpoints);
			cl.def_readwrite("publishers", &mvsim::Client::InfoPerTopic::publishers);
		}

	}
}
