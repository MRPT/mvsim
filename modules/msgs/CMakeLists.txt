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
		file(COPY python/__init__.py DESTINATION ${MVSIM_PYTHON_BUILD_DIRECTORY}/mvsim_msgs/)

		add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_PY_FILES} ${MVSIM_PYTHON_BUILD_DIRECTORY}/mvsim_msgs/
			COMMENT "Copying protobuf python files to: ${MVSIM_PYTHON_BUILD_DIRECTORY}/mvsim_msgs/"
		)

		# We must fix protobuf broken code: 
		# Read https://github.com/cpcloud/protoletariat
		# and https://github.com/protocolbuffers/protobuf/issues/1491
		find_program(PROTOL_EXECUTABLE NAMES protol)

		if (NOT PROTOL_EXECUTABLE)
			message(STATUS "======================================================")
			message(STATUS "protol not found, installing it now with pip3...      ")
			message(STATUS " If pip3 is not installed or another error is found,  ")
			message(STATUS " you can still build MVSIM without Python support     ")
			message(STATUS " setting the CMake variable MVSIM_WITH_PYTHON=Off     ")
			message(STATUS "======================================================")
			
			find_program(PIP3_EXECUTABLE NAMES pip3 REQUIRED)

			set(PROTOL_INSTALL_DIR ${MVSIM_PYTHON_BUILD_DIRECTORY}/protol-install/)

			execute_process(
				COMMAND ${PIP3_EXECUTABLE} install --root ${PROTOL_INSTALL_DIR} --prefix /usr/ .
				WORKING_DIRECTORY ${mvsim_SOURCE_DIR}/externals/protoletariat/
				RESULT_VARIABLE PIP3_INSTALL_RESULT
				OUTPUT_VARIABLE PIP3_OUTPUT
				ERROR_VARIABLE  PIP3_OUTPUT
			)
			message(STATUS "PIP3_INSTALL_RESULT: ${PIP3_INSTALL_RESULT}. Output:\n${PIP3_OUTPUT}")

			# Hint to find protol executable:
			set(PROTOL_DESTDIR ${PROTOL_INSTALL_DIR}/usr/bin)
			
			# And add pkg to PYTHONPATH:
			set(PROTOL_PKG_DIR ${PROTOL_INSTALL_DIR}/usr/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages/)
			message(STATUS "PROTOL_PKG_DIR: ${PROTOL_PKG_DIR}")
			unset(PROTOL_EXECUTABLE CACHE)
		endif()

		find_program(PROTOL_EXECUTABLE NAMES protol REQUIRED HINTS ${PROTOL_DESTDIR})
		message(STATUS "PROTOL_EXECUTABLE: ${PROTOL_EXECUTABLE}")

		configure_file(
			${mvsim_SOURCE_DIR}/cmake/apply-protol.cmake.in
			${MVSIM_PYTHON_BUILD_DIRECTORY}/apply-protol.cmake
			@ONLY
			)

		add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
			COMMAND ${CMAKE_COMMAND} -P ${MVSIM_PYTHON_BUILD_DIRECTORY}/apply-protol.cmake
			COMMENT "Applying protoletariat protobuf broken code fix..."
		)

		install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} --build \"${mvsim_BINARY_DIR}\" --target python-install)")
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
