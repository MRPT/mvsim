/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2024  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */
#pragma once

#include <mvsim/basic_types.h>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>

namespace mvsim
{
/** Templatized-class factory for mvsim objects. Does not explicitly handle
 * multi-threading. */
template <class CLASS, typename ARG1 = void, typename ARG2 = int>
class ClassFactory
{
   public:
	using Ptr = std::shared_ptr<CLASS>;

	struct TClassData
	{
		CLASS* (*ptr_factory1)(ARG1);
		CLASS* (*ptr_factory2)(ARG1, ARG2);
		TClassData() : ptr_factory1(nullptr), ptr_factory2(nullptr) {}
	};

	void do_register(const std::string& class_name, const TClassData& data)
	{
		classes_[class_name] = data;
	}

	Ptr create(const std::string& class_name, ARG1 a1) const
	{
		auto it = classes_.find(class_name);
		if (it == classes_.end())
			throw std::runtime_error(
				(std::string("ClassFactory: Unknown class ") + class_name).c_str());
		if (!it->second.ptr_factory1)
			throw std::runtime_error(
				(std::string("ClassFactory: factory(1) pointer is nullptr for ") + class_name)
					.c_str());
		return Ptr((*it->second.ptr_factory1)(a1));
	}
	Ptr create(const std::string& class_name, ARG1 a1, ARG2 a2) const
	{
		auto it = classes_.find(class_name);
		if (it == classes_.end())
			throw std::runtime_error(
				(std::string("ClassFactory: Unknown class ") + class_name).c_str());
		if (!it->second.ptr_factory2)
			throw std::runtime_error(
				(std::string("ClassFactory: factory(2) pointer is nullptr for ") + class_name)
					.c_str());
		return Ptr((*it->second.ptr_factory2)(a1, a2));
	}

   private:
	std::map<std::string, TClassData> classes_;
};	// namespace mvsim

#define DECLARES_REGISTER_CLASS1(CLASS_NAME, BASE_CLASS, ARG1) \
   public:                                                     \
	static BASE_CLASS* Create(ARG1 a1) { return new CLASS_NAME(a1); }
#define DECLARES_REGISTER_CLASS2(CLASS_NAME, BASE_CLASS, ARG1, ARG2) \
   public:                                                           \
	static BASE_CLASS* Create(ARG1 a1, ARG2 a2) { return new CLASS_NAME(a1, a2); }

#define REGISTER_CLASS1(FACTORY_TYPE, FACTORY_OBJ, TEXTUAL_NAME, CLASS_NAME) \
	{                                                                        \
		FACTORY_TYPE::TClassData data;                                       \
		data.ptr_factory1 = &CLASS_NAME::Create;                             \
		FACTORY_OBJ.do_register(TEXTUAL_NAME, data);                         \
	}

#define REGISTER_CLASS2(FACTORY_TYPE, FACTORY_OBJ, TEXTUAL_NAME, CLASS_NAME) \
	{                                                                        \
		FACTORY_TYPE::TClassData data;                                       \
		data.ptr_factory2 = &CLASS_NAME::Create;                             \
		FACTORY_OBJ.do_register(TEXTUAL_NAME, data);                         \
	}
}  // namespace mvsim
