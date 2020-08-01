#include <mvsim/Comms/common.h>
#include <sstream> // __str__

#include <pybind11/pybind11.h>
#include <functional>
#include <string>

#ifndef BINDER_PYBIND11_TYPE_CASTER
	#define BINDER_PYBIND11_TYPE_CASTER
	PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
	PYBIND11_DECLARE_HOLDER_TYPE(T, T*);
	PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>);
#endif

// mvsim::UnexpectedMessageException file:mvsim/Comms/common.h line:46
struct PyCallBack_mvsim_UnexpectedMessageException : public mvsim::UnexpectedMessageException {
	using mvsim::UnexpectedMessageException::UnexpectedMessageException;

};

void bind_mvsim_Comms_common(std::function< pybind11::module &(std::string const &namespace_) > &M)
{
	{ // mvsim::UnexpectedMessageException file:mvsim/Comms/common.h line:46
		pybind11::class_<mvsim::UnexpectedMessageException, std::shared_ptr<mvsim::UnexpectedMessageException>, PyCallBack_mvsim_UnexpectedMessageException> cl(M("mvsim"), "UnexpectedMessageException", "");
		cl.def( pybind11::init<const char *>(), pybind11::arg("reason") );

		cl.def( pybind11::init( [](PyCallBack_mvsim_UnexpectedMessageException const &o){ return new PyCallBack_mvsim_UnexpectedMessageException(o); } ) );
		cl.def( pybind11::init( [](mvsim::UnexpectedMessageException const &o){ return new mvsim::UnexpectedMessageException(o); } ) );
		cl.def("assign", (class mvsim::UnexpectedMessageException & (mvsim::UnexpectedMessageException::*)(const class mvsim::UnexpectedMessageException &)) &mvsim::UnexpectedMessageException::operator=, "C++: mvsim::UnexpectedMessageException::operator=(const class mvsim::UnexpectedMessageException &) --> class mvsim::UnexpectedMessageException &", pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
