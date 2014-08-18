/*+-------------------------------------------------------------------------+
  |                       MultiVehicle 2D simulator (libmv2dsim)            |
  |                                                                         |
  | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
  | Distributed under GNU General Public License version 3                  |
  |   See <http://www.gnu.org/licenses/>                                    |
  +-------------------------------------------------------------------------+  */
#pragma once

#include <mv2dsim/basic_types.h>
#include <map>
#include <stdexcept>
#include <string>

namespace mv2dsim
{
	/** Templatized-class factory for mv2dsim objects. Does not explicitly handle multi-threading. */
	template <class CLASS,typename ARG1, typename ARG2>
	class ClassFactory
	{
	private:
		struct TClassData
		{
			CLASS* (*ptr_factory1)(ARG1);
			CLASS* (*ptr_factory2)(ARG1,ARG2);
			TClassData() : ptr_factory1(NULL), ptr_factory2(NULL) {}
		};
		std::map<std::string,TClassData> m_classes;

	public:
		void do_register(const std::string &class_name, const TClassData &data ) {
			m_classes[class_name] = data;
		}

		CLASS* create(const std::string &class_name, ARG1 a1) const  {
			std::map<std::string,TClassData>::const_iterator it=m_classes.find(class_name);
			if (it==m_classes.end()) throw std::runtime_error( (std::string("ClassFactory: Unknown class ")+class_name).c_str() );
			if (!it->second.ptr_factory1) throw std::runtime_error( (std::string("ClassFactory: factory(1) pointer is NULL for ")+class_name).c_str() );
			return (*it->second.ptr_factory1)(a1);
		}
		CLASS* create(const std::string &class_name, ARG1 a1, ARG2 a2) const  {
			std::map<std::string,TClassData>::const_iterator it=m_classes.find(class_name);
			if (it==m_classes.end()) throw std::runtime_error( (std::string("ClassFactory: Unknown class ")+class_name).c_str() );
			if (!it->second.ptr_factory2) throw std::runtime_error( (std::string("ClassFactory: factory(2) pointer is NULL for ")+class_name).c_str() );
			return (*it->second.ptr_factory2)(a1,a2);
		}

	}; // end class
}
