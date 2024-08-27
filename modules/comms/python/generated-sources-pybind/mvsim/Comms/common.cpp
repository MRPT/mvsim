#include <mvsim/Comms/common.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <functional>
#include <sstream>	// __str__
#include <stdexcept>
#include <string>

#ifndef BINDER_PYBIND11_TYPE_CASTER
#define BINDER_PYBIND11_TYPE_CASTER
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

// mvsim::UnexpectedMessageException file:mvsim/Comms/common.h line:49
struct PyCallBack_mvsim_UnexpectedMessageException : public mvsim::UnexpectedMessageException
{
	using mvsim::UnexpectedMessageException::UnexpectedMessageException;

	const char* what() const throw() override
	{
		pybind11::gil_scoped_acquire gil;
		pybind11::function overload = pybind11::get_overload(
			static_cast<const mvsim::UnexpectedMessageException*>(this), "what");
		if (overload)
		{
			auto o = overload.operator()<pybind11::return_value_policy::reference>();
			if (pybind11::detail::cast_is_temporary_value_reference<const char*>::value)
			{
// pybind11 <=2.4: overload_caster_t, otherwise: override_caster_t
#if (PYBIND11_MAJOR_VERSION == 2 && PYBIND11_MINOR_VERSION <= 4)
				static pybind11::detail::overload_caster_t<const char*> caster;
#else
				static pybind11::detail::override_caster_t<const char*> caster;
#endif
				return pybind11::detail::cast_ref<const char*>(std::move(o), caster);
			}
			else
				return pybind11::detail::cast_safe<const char*>(std::move(o));
		}
		return runtime_error::what();
	}
};

void bind_mvsim_Comms_common(std::function<pybind11::module&(std::string const& namespace_)>& M)
{
	{  // mvsim::UnexpectedMessageException file:mvsim/Comms/common.h line:49
		pybind11::class_<
			mvsim::UnexpectedMessageException, std::shared_ptr<mvsim::UnexpectedMessageException>,
			PyCallBack_mvsim_UnexpectedMessageException, std::runtime_error>
			cl(M("mvsim"), "UnexpectedMessageException", "");
		cl.def(pybind11::init<const char*>(), pybind11::arg("reason"));

		cl.def(pybind11::init([](PyCallBack_mvsim_UnexpectedMessageException const& o)
							  { return new PyCallBack_mvsim_UnexpectedMessageException(o); }));
		cl.def(pybind11::init([](mvsim::UnexpectedMessageException const& o)
							  { return new mvsim::UnexpectedMessageException(o); }));
		cl.def(
			"assign",
			(class mvsim::UnexpectedMessageException &
			 (mvsim::UnexpectedMessageException::*)(const class mvsim::
														UnexpectedMessageException&)) &
				mvsim::UnexpectedMessageException::operator=,
			"C++: mvsim::UnexpectedMessageException::operator=(const class "
			"mvsim::UnexpectedMessageException &) --> class "
			"mvsim::UnexpectedMessageException &",
			pybind11::return_value_policy::automatic, pybind11::arg(""));
	}
}
