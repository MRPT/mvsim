#include <iterator>
#include <memory>
#include <mvsim/Comms/Client.h>
#include <sstream> // __str__
#include <string>

#include <pybind11/pybind11.h>
#include <functional>
#include <string>

#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*);
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>);
#endif

void bind_mvsim_Comms_Client(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mvsim::Client file:mvsim/Comms/Client.h line:36
		pybind11::class_<mvsim::Client, std::shared_ptr<mvsim::Client>> cl(M("mvsim"), "Client", "This is the connection of any user program with the MVSIM server, so\n it can advertise and subscribe to topics.\n Users should instance a class mvsim::Client (C++) or mvsim.Client (Python) to\n communicate with the simulation runnin in mvsim::World or any other module.\n\n Usage:\n  - Instantiate a Client object.\n  - Call connect(). It will return immediately.\n  - The client will be working on the background as long as the object is not\n destroyed.\n\n Messages and topics are described as Protobuf messages, and communications\n are done via ZMQ sockets.\n\n See: https://mvsimulator.readthedocs.io/");
		cl.def( pybind11::init( [](){ return new mvsim::Client(); } ) );
		cl.def("connect", (void (mvsim::Client::*)()) &mvsim::Client::connect, "Connects to the server in a parallel thread. \n\nC++: mvsim::Client::connect() --> void");
		cl.def("shutdown", (void (mvsim::Client::*)()) &mvsim::Client::shutdown, "Shutdowns the communication thread. Blocks until the thread is stopped.\n There is no need to manually call this method, it is called upon\n destruction. \n\nC++: mvsim::Client::shutdown() --> void");
	}
}
