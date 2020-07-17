set(MODULE_NAME "msgs")
project(mvsim-${MODULE_NAME} LANGUAGES CXX VERSION ${MVSIM_VERSION})

if (NOT MVSIM_WITH_PROTOBUF)
  message(FATAL_ERROR "Cannot build this module without protobuf")
endif()

# Protobuf messages:
file(GLOB PROTOBUF_DEFINITION_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOBUF_DEFINITION_FILES})
protobuf_generate_python(PROTO_PY_FILES ${PROTOBUF_DEFINITION_FILES})

# Make a copy of headers so they appear under the same local 
# path for both, INSTALL and BUILD interfaces:
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/mvsim/mvsim-msgs/)
file(COPY ${PROTO_HDRS} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/mvsim/mvsim-msgs/)

# Declare the target (a library)
add_library(${PROJECT_NAME}
  ${PROTO_SRCS} ${PROTO_HDRS}
  ${PROTO_PY_FILES}
  ${PROTOBUF_DEFINITION_FILES}
)

source_group(ProtobufMessages FILES
  ${PROTO_SRCS} ${PROTO_HDRS}
  ${PROTO_PY_FILES}
  ${PROTOBUF_DEFINITION_FILES}
)

mvsim_common_target_settings(${MODULE_NAME})

if (MVSIM_WITH_PROTOBUF)
	target_include_directories(${PROJECT_NAME}
		SYSTEM PUBLIC
			$<BUILD_INTERFACE:${Protobuf_INCLUDE_DIRS}>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
			$<INSTALL_INTERFACE:include/mvsim-msgs>
	)

	#message(STATUS "Python_SITELIB: ${Python_SITELIB}")
	# TODO: Install python files in a better place?

	install(
	FILES
	  ${PROTO_HDRS}
	  ${PROTO_PY_FILES}
	DESTINATION
	  ${CMAKE_INSTALL_INCLUDEDIR}/mvsim/mvsim-msgs/
	)
	install(
	FILES
	  ${PROTOBUF_DEFINITION_FILES}
	DESTINATION
	  ${CMAKE_INSTALL_INCLUDEDIR}/mvsim/mvsim-msgs/proto/
	)
endif()


if (NOT MSVC)
  # silent warnings for older protobuf versions:
  target_compile_options(${PROJECT_NAME} PRIVATE -Wno-suggest-override)
endif()
