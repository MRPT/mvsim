set(MODULE_NAME "msgs")
project(mvsim-${MODULE_NAME} LANGUAGES CXX VERSION ${MVSIM_VERSION})

if (NOT MVSIM_WITH_PROTOBUF)
  message(FATAL_ERROR "Cannot build this module without protobuf")
endif()

# Protobuf messages:
file(GLOB PROTOBUF_DEFINITION_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOBUF_DEFINITION_FILES})
protobuf_generate_python(PROTO_PY_FILES ${PROTOBUF_DEFINITION_FILES})

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
set(PACKAGE_DEPENDENCIES "")
mvsim_common_target_settings(${MODULE_NAME})

if (MVSIM_WITH_PROTOBUF)

	# Make a copy of headers so they appear under the same local
	# path for both, INSTALL and BUILD interfaces:
	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/mvsim/mvsim-msgs/)
	add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_HDRS} ${CMAKE_CURRENT_BINARY_DIR}/mvsim/mvsim-msgs/
	)

	# Python files:
	if (MVSIM_WITH_PYTHON)
		file(MAKE_DIRECTORY ${MVSIM_PYTHON_BUILD_DIRECTORY}/mvsim_msgs/)
		add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY_FILES} ${MVSIM_PYTHON_BUILD_DIRECTORY}/mvsim_msgs/
		)
		file(COPY python/__init__.py DESTINATION ${MVSIM_PYTHON_BUILD_DIRECTORY}/mvsim_msgs/)
	endif()
	
	target_include_directories(${PROJECT_NAME}
		SYSTEM PUBLIC
			#$<BUILD_INTERFACE:${Protobuf_INCLUDE_DIRS}>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
			$<INSTALL_INTERFACE:include>
	)

	install(
	FILES
	  ${PROTO_HDRS}
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
