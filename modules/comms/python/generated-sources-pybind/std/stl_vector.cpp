#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <functional>
#include <iterator>
#include <memory>
#include <sstream>	// __str__
#include <string>
#include <vector>

#ifndef BINDER_PYBIND11_TYPE_CASTER
#define BINDER_PYBIND11_TYPE_CASTER
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>)
PYBIND11_DECLARE_HOLDER_TYPE(T, T*)
PYBIND11_MAKE_OPAQUE(std::shared_ptr<void>)
#endif

void bind_std_stl_vector(std::function<pybind11::module&(std::string const& namespace_)>& M)
{
	{  // std::vector file:bits/stl_vector.h line:425
		pybind11::class_<std::vector<unsigned char>> cl(M("std"), "vector_unsigned_char_t", "");
		cl.def(pybind11::init([]() { return new std::vector<unsigned char>(); }));
		cl.def(pybind11::init<const class std::allocator<unsigned char>&>(), pybind11::arg("__a"));

		cl.def(
			pybind11::init([](unsigned long const& a0)
						   { return new std::vector<unsigned char>(a0); }),
			"doc", pybind11::arg("__n"));
		cl.def(
			pybind11::init<unsigned long, const class std::allocator<unsigned char>&>(),
			pybind11::arg("__n"), pybind11::arg("__a"));

		cl.def(
			pybind11::init([](unsigned long const& a0, const unsigned char& a1)
						   { return new std::vector<unsigned char>(a0, a1); }),
			"doc", pybind11::arg("__n"), pybind11::arg("__value"));
		cl.def(
			pybind11::init<
				unsigned long, const unsigned char&, const class std::allocator<unsigned char>&>(),
			pybind11::arg("__n"), pybind11::arg("__value"), pybind11::arg("__a"));

		cl.def(pybind11::init([](std::vector<unsigned char> const& o)
							  { return new std::vector<unsigned char>(o); }));
		cl.def(
			pybind11::init<
				const class std::vector<unsigned char>&,
				const class std::allocator<unsigned char>&>(),
			pybind11::arg("__x"), pybind11::arg("__a"));

		cl.def(
			"assign",
			(class std::vector<unsigned char> &
			 (std::vector<unsigned char>::*)(const class std::vector<unsigned char>&)) &
				std::vector<unsigned char>::operator=,
			"C++: std::vector<unsigned char>::operator=(const class std::vector<unsigned char> &) "
			"--> class std::vector<unsigned char> &",
			pybind11::return_value_policy::automatic, pybind11::arg("__x"));
		cl.def(
			"assign",
			(void(std::vector<unsigned char>::*)(unsigned long, const unsigned char&)) &
				std::vector<unsigned char>::assign,
			"C++: std::vector<unsigned char>::assign(unsigned long, const unsigned char &) --> "
			"void",
			pybind11::arg("__n"), pybind11::arg("__val"));
		cl.def(
			"begin",
			(class __gnu_cxx::__normal_iterator<unsigned char*, class std::vector<unsigned char>>(
				std::vector<unsigned char>::*)()) &
				std::vector<unsigned char>::begin,
			"C++: std::vector<unsigned char>::begin() --> class "
			"__gnu_cxx::__normal_iterator<unsigned char *, class std::vector<unsigned char> >");
		cl.def(
			"end",
			(class __gnu_cxx::__normal_iterator<unsigned char*, class std::vector<unsigned char>>(
				std::vector<unsigned char>::*)()) &
				std::vector<unsigned char>::end,
			"C++: std::vector<unsigned char>::end() --> class "
			"__gnu_cxx::__normal_iterator<unsigned char *, class std::vector<unsigned char> >");
		cl.def(
			"cbegin",
			(class __gnu_cxx::__normal_iterator<
				const unsigned char*, class std::vector<unsigned char>>(
				std::vector<unsigned char>::*)() const) &
				std::vector<unsigned char>::cbegin,
			"C++: std::vector<unsigned char>::cbegin() const --> class "
			"__gnu_cxx::__normal_iterator<const unsigned char *, class std::vector<unsigned char> "
			">");
		cl.def(
			"cend",
			(class __gnu_cxx::__normal_iterator<
				const unsigned char*, class std::vector<unsigned char>>(
				std::vector<unsigned char>::*)() const) &
				std::vector<unsigned char>::cend,
			"C++: std::vector<unsigned char>::cend() const --> class "
			"__gnu_cxx::__normal_iterator<const unsigned char *, class std::vector<unsigned char> "
			">");
		cl.def(
			"size",
			(unsigned long (std::vector<unsigned char>::*)() const) &
				std::vector<unsigned char>::size,
			"C++: std::vector<unsigned char>::size() const --> unsigned long");
		cl.def(
			"max_size",
			(unsigned long (std::vector<unsigned char>::*)() const) &
				std::vector<unsigned char>::max_size,
			"C++: std::vector<unsigned char>::max_size() const --> unsigned long");
		cl.def(
			"resize",
			(void(std::vector<unsigned char>::*)(unsigned long)) &
				std::vector<unsigned char>::resize,
			"C++: std::vector<unsigned char>::resize(unsigned long) --> void",
			pybind11::arg("__new_size"));
		cl.def(
			"resize",
			(void(std::vector<unsigned char>::*)(unsigned long, const unsigned char&)) &
				std::vector<unsigned char>::resize,
			"C++: std::vector<unsigned char>::resize(unsigned long, const unsigned char &) --> "
			"void",
			pybind11::arg("__new_size"), pybind11::arg("__x"));
		cl.def(
			"shrink_to_fit",
			(void(std::vector<unsigned char>::*)()) & std::vector<unsigned char>::shrink_to_fit,
			"C++: std::vector<unsigned char>::shrink_to_fit() --> void");
		cl.def(
			"capacity",
			(unsigned long (std::vector<unsigned char>::*)() const) &
				std::vector<unsigned char>::capacity,
			"C++: std::vector<unsigned char>::capacity() const --> unsigned long");
		cl.def(
			"empty",
			(bool(std::vector<unsigned char>::*)() const) & std::vector<unsigned char>::empty,
			"C++: std::vector<unsigned char>::empty() const --> bool");
		cl.def(
			"reserve",
			(void(std::vector<unsigned char>::*)(unsigned long)) &
				std::vector<unsigned char>::reserve,
			"C++: std::vector<unsigned char>::reserve(unsigned long) --> void",
			pybind11::arg("__n"));
		cl.def(
			"__getitem__",
			(unsigned char& (std::vector<unsigned char>::*)(unsigned long)) &
				std::vector<unsigned char>::operator[],
			"C++: std::vector<unsigned char>::operator[](unsigned long) --> unsigned char &",
			pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def(
			"at",
			(unsigned char& (std::vector<unsigned char>::*)(unsigned long)) &
				std::vector<unsigned char>::at,
			"C++: std::vector<unsigned char>::at(unsigned long) --> unsigned char &",
			pybind11::return_value_policy::automatic, pybind11::arg("__n"));
		cl.def(
			"front",
			(unsigned char& (std::vector<unsigned char>::*)()) & std::vector<unsigned char>::front,
			"C++: std::vector<unsigned char>::front() --> unsigned char &",
			pybind11::return_value_policy::automatic);
		cl.def(
			"back",
			(unsigned char& (std::vector<unsigned char>::*)()) & std::vector<unsigned char>::back,
			"C++: std::vector<unsigned char>::back() --> unsigned char &",
			pybind11::return_value_policy::automatic);
		cl.def(
			"data",
			(unsigned char* (std::vector<unsigned char>::*)()) & std::vector<unsigned char>::data,
			"C++: std::vector<unsigned char>::data() --> unsigned char *",
			pybind11::return_value_policy::automatic);
		cl.def(
			"push_back",
			(void(std::vector<unsigned char>::*)(const unsigned char&)) &
				std::vector<unsigned char>::push_back,
			"C++: std::vector<unsigned char>::push_back(const unsigned char &) --> void",
			pybind11::arg("__x"));
		cl.def(
			"pop_back",
			(void(std::vector<unsigned char>::*)()) & std::vector<unsigned char>::pop_back,
			"C++: std::vector<unsigned char>::pop_back() --> void");
		cl.def(
			"insert",
			(class __gnu_cxx::__normal_iterator<unsigned char*, class std::vector<unsigned char>>(
				std::vector<unsigned char>::*)(
				class __gnu_cxx::__normal_iterator<
					const unsigned char*, class std::vector<unsigned char>>,
				const unsigned char&)) &
				std::vector<unsigned char>::insert,
			"C++: std::vector<unsigned char>::insert(class __gnu_cxx::__normal_iterator<const "
			"unsigned char *, class std::vector<unsigned char> >, const unsigned char &) --> class "
			"__gnu_cxx::__normal_iterator<unsigned char *, class std::vector<unsigned char> >",
			pybind11::arg("__position"), pybind11::arg("__x"));
		cl.def(
			"insert",
			(class __gnu_cxx::__normal_iterator<unsigned char*, class std::vector<unsigned char>>(
				std::vector<unsigned char>::*)(
				class __gnu_cxx::__normal_iterator<
					const unsigned char*, class std::vector<unsigned char>>,
				unsigned long, const unsigned char&)) &
				std::vector<unsigned char>::insert,
			"C++: std::vector<unsigned char>::insert(class __gnu_cxx::__normal_iterator<const "
			"unsigned char *, class std::vector<unsigned char> >, unsigned long, const unsigned "
			"char &) --> class __gnu_cxx::__normal_iterator<unsigned char *, class "
			"std::vector<unsigned char> >",
			pybind11::arg("__position"), pybind11::arg("__n"), pybind11::arg("__x"));
		cl.def(
			"erase",
			(class __gnu_cxx::__normal_iterator<unsigned char*, class std::vector<unsigned char>>(
				std::vector<unsigned char>::*)(
				class __gnu_cxx::__normal_iterator<
					const unsigned char*, class std::vector<unsigned char>>)) &
				std::vector<unsigned char>::erase,
			"C++: std::vector<unsigned char>::erase(class __gnu_cxx::__normal_iterator<const "
			"unsigned char *, class std::vector<unsigned char> >) --> class "
			"__gnu_cxx::__normal_iterator<unsigned char *, class std::vector<unsigned char> >",
			pybind11::arg("__position"));
		cl.def(
			"erase",
			(class __gnu_cxx::__normal_iterator<unsigned char*, class std::vector<unsigned char>>(
				std::vector<unsigned char>::*)(
				class __gnu_cxx::__normal_iterator<
					const unsigned char*, class std::vector<unsigned char>>,
				class __gnu_cxx::__normal_iterator<
					const unsigned char*, class std::vector<unsigned char>>)) &
				std::vector<unsigned char>::erase,
			"C++: std::vector<unsigned char>::erase(class __gnu_cxx::__normal_iterator<const "
			"unsigned char *, class std::vector<unsigned char> >, class "
			"__gnu_cxx::__normal_iterator<const unsigned char *, class std::vector<unsigned char> "
			">) --> class __gnu_cxx::__normal_iterator<unsigned char *, class std::vector<unsigned "
			"char> >",
			pybind11::arg("__first"), pybind11::arg("__last"));
		cl.def(
			"swap",
			(void(std::vector<unsigned char>::*)(class std::vector<unsigned char>&)) &
				std::vector<unsigned char>::swap,
			"C++: std::vector<unsigned char>::swap(class std::vector<unsigned char> &) --> void",
			pybind11::arg("__x"));
		cl.def(
			"clear", (void(std::vector<unsigned char>::*)()) & std::vector<unsigned char>::clear,
			"C++: std::vector<unsigned char>::clear() --> void");
	}
}
