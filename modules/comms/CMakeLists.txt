set(MODULE_NAME "comms")
project(mvsim-${MODULE_NAME} LANGUAGES CXX VERSION ${MVSIM_VERSION})

file(GLOB Comms_SRCS
	"${CMAKE_CURRENT_SOURCE_DIR}/src/Comms/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/mvsim/Comms/*.h"
)

# Declare the target (a library)
add_library(${PROJECT_NAME}
	${Comms_SRCS}
)

source_group(Comms FILES ${Comms_SRCS})

set(PACKAGE_DEPENDENCIES
  mrpt-system
  mrpt-io
  mrpt-serialization
  mvsim-msgs
  )
mvsim_common_target_settings(${MODULE_NAME})

target_link_libraries(${PROJECT_NAME}
 PRIVATE
	${CMAKE_THREAD_LIBS_INIT}
	${ZeroMQ_LIBRARY}
 PUBLIC
	${Protobuf_LIBRARIES}
	mrpt::system
	mrpt::io
	mrpt::serialization
	mvsim::msgs
)

# =========== Python wrapper ==================
if (MVSIM_WITH_PYTHON)
  # Different versions of pybind11 required different generated-sources:
  string(SUBSTRING ${pybind11_VERSION} 0 3 PYBIND11_MM_VERSION)
  #message(STATUS "PYBIND11_MM_VERSION: ${PYBIND11_MM_VERSION}")

  set(PY_SRCS_DIR ${CMAKE_CURRENT_LIST_DIR}/python/generated-sources-pybind${PYBIND11_MM_VERSION})

  if (NOT EXISTS ${PY_SRCS_DIR})
	message(WARNING "Skipping python wrapper due to missing wrapper C++ files for especific pybind11 version ${PYBIND11_MM_VERSION}: expected directory '${PY_SRCS_DIR}'")
  else()
	file(GLOB_RECURSE PY_SRCS ${PY_SRCS_DIR}/*.cpp)

	pybind11_add_module(pymvsim_${MODULE_NAME}
		${PY_SRCS}
		)
	target_link_libraries(pymvsim_${MODULE_NAME} PUBLIC ${PROJECT_NAME})

	# Python files:
	file(MAKE_DIRECTORY ${MVSIM_PYTHON_BUILD_DIRECTORY}/mvsim_${MODULE_NAME}/)
	add_custom_command(TARGET pymvsim_${MODULE_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:pymvsim_${MODULE_NAME}>
		${MVSIM_PYTHON_BUILD_DIRECTORY}/mvsim_${MODULE_NAME}/
	)
	file(WRITE ${MVSIM_PYTHON_BUILD_DIRECTORY}/mvsim_${MODULE_NAME}/__init__.py
		"from . import *")
  endif()

endif()
