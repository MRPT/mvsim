#include <mrpt/system/CTimeLogger.h>
#include <mvsim/Comms/Client.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <functional>
#include <map>
#include <memory>
#include <sstream>	// __str__
#include <string>
#include <string_view>
#include <vector>

#ifndef BINDER_PYBIND11_TYPE_CASTER
#define BINDER_PYBIND11_TYPE_CASTER
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_mvsim_Comms_Client(std::function<pybind11::module&(std::string const& namespace_)>& M)
{
	{  // mvsim::Client file:mvsim/Comms/Client.h line:48
		pybind11::class_<mvsim::Client, std::shared_ptr<mvsim::Client>> cl(
			M("mvsim"), "Client",
			"This is the connection of any user program with the MVSIM server, "
			"so\\n it can advertise and subscribe to topics and use remote "
			"services.\\n\\n Users should instance a class mvsim::Client (C++) "
			"or mvsim.Client (Python) to\\n communicate with the simulation "
			"running in mvsim::World or any other module.\\n\\n Usage:\\n  - "
			"Instantiate a Client object.\\n  - Call connect(). It will return "
			"immediately.\\n  - The client will be working on the background as "
			"long as the object is not\\n destroyed.\\n\\n Messages and topics "
			"are described as Protobuf messages, and communications\\n are done "
			"via ZMQ sockets.\\n\\n See: https://mvsimulator.readthedocs.io/\\n\\n "
			"\\n\\n ");
		cl.def(pybind11::init([]() { return new mvsim::Client(); }));
		cl.def(pybind11::init<const std::string&>(), pybind11::arg("nodeName"));

		cl.def(
			"setName", (void(mvsim::Client::*)(const std::string&)) & mvsim::Client::setName,
			"C++: mvsim::Client::setName(const std::string &) --> void", pybind11::arg("nodeName"));
		cl.def(
			"serverHostAddress",
			(const std::string& (mvsim::Client::*)() const) & mvsim::Client::serverHostAddress,
			"C++: mvsim::Client::serverHostAddress() const --> const std::string &",
			pybind11::return_value_policy::automatic);
		cl.def(
			"serverHostAddress",
			(void(mvsim::Client::*)(const std::string&)) & mvsim::Client::serverHostAddress,
			"C++: mvsim::Client::serverHostAddress(const std::string &) --> void",
			pybind11::arg("serverIpOrAddressName"));
		cl.def(
			"connect", (void(mvsim::Client::*)()) & mvsim::Client::connect,
			"Connects to the server in a parallel thread.\\n  Default server "
			"address is `localhost`, can be changed with\\n "
			"serverHostAddress().\\n\\nC++: mvsim::Client::connect() --> void");
		cl.def(
			"connected", (bool(mvsim::Client::*)() const) & mvsim::Client::connected,
			"Whether the client is correctly connected to the server. \\n\\nC++: "
			"mvsim::Client::connected() const --> bool");
		cl.def(
			"shutdown", (void(mvsim::Client::*)()) & mvsim::Client::shutdown,
			"Shutdowns the communication thread. Blocks until the thread is "
			"stopped.\\n There is no need to manually call this method, it is "
			"called upon\\n destruction. \\n\\nC++: mvsim::Client::shutdown() --> "
			"void");

		// callService: Use a py::bytes lambda wrapper to avoid pybind11
		// attempting a UTF-8 decode of serialized protobuf binary data.
		// See: https://pybind11.readthedocs.io/en/stable/advanced/cast/strings.html
		// This fixes: https://github.com/MRPT/mvsim/issues/62
		cl.def(
			"callService",
			[](mvsim::Client& self, const std::string& serviceName,
			   const pybind11::bytes& inputSerializedMsg) -> pybind11::bytes
			{
				const std::string inputStr(inputSerializedMsg);
				const std::string result = self.callService(serviceName, inputStr);
				return pybind11::bytes(result);
			},
			"Overload for python wrapper. Calls a service by name, passing a "
			"serialized protobuf request and receiving a serialized protobuf "
			"response as bytes.\\n\\nC++: mvsim::Client::callService("
			"const std::string &, const std::string &) --> bytes",
			pybind11::arg("serviceName"), pybind11::arg("inputSerializedMsg"));

		// subscribeTopic: Wrap the callback so that the serialized message
		// data arrives as py::bytes instead of std::vector<uint8_t> (which
		// pybind11 would convert to a Python list[int]).
		cl.def(
			"subscribeTopic",
			[](mvsim::Client& self, const std::string& topicName, pybind11::object callback)
			{
				self.subscribeTopic(
					topicName,
					[callback](const std::string& msgType, const std::vector<uint8_t>& data)
					{
						pybind11::gil_scoped_acquire acquire;
						callback(
							pybind11::str(msgType),
							pybind11::bytes(
								reinterpret_cast<const char*>(data.data()), data.size()));
					});
			},
			"Overload for python wrapper (callback receives msgType: str, "
			"data: bytes).\\n\\nC++: mvsim::Client::subscribeTopic("
			"const std::string &, callback) --> void",
			pybind11::arg("topicName"), pybind11::arg("callback"));

		cl.def(
			"enable_profiler", (void(mvsim::Client::*)(bool)) & mvsim::Client::enable_profiler,
			"C++: mvsim::Client::enable_profiler(bool) --> void", pybind11::arg("enable"));

		{  // mvsim::Client::InfoPerNode file:mvsim/Comms/Client.h line:109
			auto& enclosing_class = cl;
			pybind11::class_<
				mvsim::Client::InfoPerNode, std::shared_ptr<mvsim::Client::InfoPerNode>>
				cl(enclosing_class, "InfoPerNode", "");
			cl.def(pybind11::init([]() { return new mvsim::Client::InfoPerNode(); }));
			cl.def(pybind11::init([](mvsim::Client::InfoPerNode const& o)
								  { return new mvsim::Client::InfoPerNode(o); }));
			cl.def_readwrite("name", &mvsim::Client::InfoPerNode::name);
			cl.def(
				"assign",
				(struct mvsim::Client::InfoPerNode &
				 (mvsim::Client::InfoPerNode::*)(const struct mvsim::Client::InfoPerNode&)) &
					mvsim::Client::InfoPerNode::operator=,
				"C++: mvsim::Client::InfoPerNode::operator=(const struct "
				"mvsim::Client::InfoPerNode &) --> struct "
				"mvsim::Client::InfoPerNode &",
				pybind11::return_value_policy::automatic, pybind11::arg(""));
		}

		{  // mvsim::Client::InfoPerTopic file:mvsim/Comms/Client.h line:115
			auto& enclosing_class = cl;
			pybind11::class_<
				mvsim::Client::InfoPerTopic, std::shared_ptr<mvsim::Client::InfoPerTopic>>
				cl(enclosing_class, "InfoPerTopic", "");
			cl.def(pybind11::init([]() { return new mvsim::Client::InfoPerTopic(); }));
			cl.def(pybind11::init([](mvsim::Client::InfoPerTopic const& o)
								  { return new mvsim::Client::InfoPerTopic(o); }));
			cl.def_readwrite("name", &mvsim::Client::InfoPerTopic::name);
			cl.def_readwrite("type", &mvsim::Client::InfoPerTopic::type);
			cl.def_readwrite("endpoints", &mvsim::Client::InfoPerTopic::endpoints);
			cl.def_readwrite("publishers", &mvsim::Client::InfoPerTopic::publishers);
			cl.def(
				"assign",
				(struct mvsim::Client::InfoPerTopic &
				 (mvsim::Client::InfoPerTopic::*)(const struct mvsim::Client::InfoPerTopic&)) &
					mvsim::Client::InfoPerTopic::operator=,
				"C++: mvsim::Client::InfoPerTopic::operator=(const struct "
				"mvsim::Client::InfoPerTopic &) --> struct "
				"mvsim::Client::InfoPerTopic &",
				pybind11::return_value_policy::automatic, pybind11::arg(""));
		}
	}
}