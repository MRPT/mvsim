#include <pybind11/pybind11.h>

#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

typedef std::function<pybind11::module&(std::string const&)> ModuleGetter;

void bind_std_exception(
	std::function<pybind11::module&(std::string const& namespace_)>& M);
void bind_std_stdexcept(
	std::function<pybind11::module&(std::string const& namespace_)>& M);
void bind_mvsim_Comms_common(
	std::function<pybind11::module&(std::string const& namespace_)>& M);
void bind_mvsim_Comms_Client(
	std::function<pybind11::module&(std::string const& namespace_)>& M);

PYBIND11_MODULE(pymvsim_comms, root_module)
{
	root_module.doc() = "pymvsim_comms module";

	std::map<std::string, pybind11::module> modules;
	ModuleGetter M = [&](std::string const& namespace_) -> pybind11::module& {
		auto it = modules.find(namespace_);
		if (it == modules.end())
			throw std::runtime_error(
				"Attempt to access pybind11::module for namespace " +
				namespace_ + " before it was created!!!");
		return it->second;
	};

	modules[""] = root_module;

	static std::vector<std::string> const reserved_python_words{
		"nonlocal",
		"global",
	};

	auto mangle_namespace_name([](std::string const& ns) -> std::string {
		if (std::find(
				reserved_python_words.begin(), reserved_python_words.end(),
				ns) == reserved_python_words.end())
			return ns;
		else
			return ns + '_';
	});

	std::vector<std::pair<std::string, std::string>> sub_modules{
		{"", "mvsim"},
		{"", "std"},
	};
	for (auto& p : sub_modules)
		modules[p.first.size() ? p.first + "::" + p.second : p.second] =
			modules[p.first].def_submodule(
				mangle_namespace_name(p.second).c_str(),
				("Bindings for " + p.first + "::" + p.second + " namespace")
					.c_str());

	// pybind11::class_<std::shared_ptr<void>>(M(""), "_encapsulated_data_");

	bind_std_exception(M);
	bind_std_stdexcept(M);
	bind_mvsim_Comms_common(M);
	bind_mvsim_Comms_Client(M);
}
