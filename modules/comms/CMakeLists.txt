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

mvsim_common_target_settings(${MODULE_NAME})

target_link_libraries(${PROJECT_NAME} PUBLIC mvsim::msgs)

target_link_libraries(${PROJECT_NAME}
 PRIVATE
	${CMAKE_THREAD_LIBS_INIT}
	${ZeroMQ_LIBRARY}
 PUBLIC
	${Protobuf_LIBRARIES}
	mrpt::system
	mrpt::io
	mrpt::serialization
)

# =========== Python wrapper ==================
if (MVSIM_WITH_PYTHON)
  file(GLOB_RECURSE PY_SRCS python/generated-sources/*.cpp)

  pybind11_add_module(pymvsim_${MODULE_NAME}
	${PY_SRCS}
	)
  target_link_libraries(pymvsim_${MODULE_NAME} PUBLIC ${PROJECT_NAME})
endif()
