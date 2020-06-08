project(libmvsim LANGUAGES CXX VERSION 0.3.0)

# It must be at the same directory than protobuf_generate_cpp()
option(MVSIM_WITH_PROTOBUF "Use protobuf for communications" ON)

if (MVSIM_WITH_PROTOBUF)
	find_package(Protobuf)
	if (Protobuf_FOUND)
		#
	else()
		set(MVSIM_WITH_PROTOBUF OFF CACHE BOOL "Use protobuf for communications" FORCE)
	endif()
endif()

option(MVSIM_WITH_ZMQ "Use ZMQ for communications" ON)

if (MVSIM_WITH_ZMQ)
	find_package(PkgConfig)
	pkg_check_modules(PC_ZeroMQ QUIET zmq)

	# use the hint from above to find where 'zmq.hpp' is located
	find_path(ZeroMQ_INCLUDE_DIR
			NAMES zmq.hpp
			PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
			)

	# use the hint from about to find the location of libzmq
	find_library(ZeroMQ_LIBRARY
			NAMES zmq
			PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
			)

	if ($ENV{VERBOSE})
		message(STATUS "ZeroMQ_LIBRARY: ${ZeroMQ_LIBRARY}")
		message(STATUS "ZeroMQ_INCLUDE_DIR: ${ZeroMQ_INCLUDE_DIR}")
	endif()

	if (ZeroMQ_LIBRARY AND ZeroMQ_INCLUDE_DIR)
		#
	else()
		set(MVSIM_WITH_ZMQ OFF CACHE BOOL "Use ZMQ for communications" FORCE)
	endif()
endif()



file(GLOB ROOT_SRCS
	"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/mvsim/*.h"
	)
file(GLOB Sensors_SRCS
	"${CMAKE_CURRENT_SOURCE_DIR}/src/Sensors/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/mvsim/Sensors/*.h"
	)
file(GLOB Friction_SRCS
	"${CMAKE_CURRENT_SOURCE_DIR}/src/FrictionModels/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/mvsim/FrictionModels/*.h"
	)
file(GLOB VehicleDynamics_SRCS
	"${CMAKE_CURRENT_SOURCE_DIR}/src/VehicleDynamics/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/mvsim/VehicleDynamics/*.h"
	)
file(GLOB WorldElements_SRCS
	"${CMAKE_CURRENT_SOURCE_DIR}/src/WorldElements/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/mvsim/WorldElements/*.h"
	)

# Protobuf messages:
file(GLOB PROTOBUF_DEFINITION_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
if (MVSIM_WITH_PROTOBUF)
	foreach(file ${PROTOBUF_DEFINITION_FILES})
		protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${file})
		#protobuf_generate_python(...)
	endforeach()
endif()

# Declare the target (a library)
add_library(${PROJECT_NAME}
	${ROOT_SRCS}
	${Sensors_SRCS}
	${Friction_SRCS}
	${VehicleDynamics_SRCS}
	${WorldElements_SRCS}
	${PROTO_SRCS} ${PROTO_HDRS} ${PROTOBUF_DEFINITION_FILES}
)

# To make code portable between using directly the mvsim source tree, or
# importing exported cmake targets:
add_library(mvsim::mvsim   ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_THREAD_LIBS_INIT}
    ${Protobuf_LIBRARIES}
    ${ZeroMQ_LIBRARY}
  PUBLIC
    ${MRPT_LIBRARIES}
    ${BOX2D_LIBRARIES}
)

source_group(Main FILES ${ROOT_SRCS})
source_group(Sensors FILES ${Sensors_SRCS})
source_group(Friction FILES ${Friction_SRCS})
source_group(VehicleDynamics FILES ${VehicleDynamics_SRCS})
source_group(WorldElements FILES ${WorldElements_SRCS})
source_group(ProtobufMessages FILES ${PROTO_SRCS} ${PROTO_HDRS} ${PROTOBUF_DEFINITION_FILES})


set_target_properties(
  ${PROJECT_NAME}
PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  VERSION ${PROJECT_VERSION}
  OUTPUT_NAME mvsim
  ARCHIVE_OUTPUT_DIRECTORY "lib"
  LIBRARY_OUTPUT_DIRECTORY "lib"
  WINDOWS_EXPORT_ALL_SYMBOLS ON
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${ZeroMQ_INCLUDE_DIR}
)
target_include_directories(${PROJECT_NAME}
    SYSTEM PUBLIC
        $<BUILD_INTERFACE:${mvsim_SOURCE_DIR}/externals/rapidxml>
)

if (MVSIM_WITH_PROTOBUF)
	target_include_directories(${PROJECT_NAME}
	    SYSTEM PRIVATE
	        $<BUILD_INTERFACE:${Protobuf_INCLUDE_DIRS}>
			$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
	)
endif()

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
	target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall -pedantic
        -Wsuggest-override
        -Werror=return-type
        # -Werror=suggest-override # Removed for protobuf headers in u18.04
    )

	if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
		target_compile_options(${PROJECT_NAME} PRIVATE
            "-O3"
        )
	endif()
endif()

if(MSVC)
	target_compile_definitions(${PROJECT_NAME} PRIVATE
		_CRT_SECURE_NO_WARNINGS
	)
	target_compile_options(${PROJECT_NAME} PRIVATE
		/wd4275
		/wd4251
	)
endif()

if (MVSIM_WITH_PROTOBUF)
	target_compile_definitions(${PROJECT_NAME} PRIVATE MVSIM_HAS_PROTOBUF)
endif()

if (MVSIM_WITH_ZMQ)
	target_compile_definitions(${PROJECT_NAME} PRIVATE MVSIM_HAS_ZMQ)
endif()

# ==== Install & export target ========
install(TARGETS	${PROJECT_NAME}
		EXPORT 	${PROJECT_NAME}-targets
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}-targets.cmake
    NAMESPACE mvsim::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
export(
	TARGETS ${PROJECT_NAME}
	FILE "${CMAKE_BINARY_DIR}/cmake/${PROJECT_NAME}-targets.cmake"
	NAMESPACE mvsim::
)


include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${mvsim_SOURCE_DIR}/cmake/mvsim-config.cmake.in
    ${CMAKE_BINARY_DIR}/cmake/mvsim-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mvsim
)

write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/cmake/mvsim-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES
        ${CMAKE_BINARY_DIR}/cmake/mvsim-config.cmake
        ${CMAKE_BINARY_DIR}/cmake/mvsim-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/mvsim
)


# ==== Show results ========
message(STATUS "----- Configured libmvsim (version ${PROJECT_VERSION}) ------")
message(STATUS "MRPT version   : ${MRPT_VERSION}")
message(STATUS "libbox2d       : ${BOX2D_VERSION_STRING}")
message(STATUS "Protobuf       : ${MVSIM_WITH_PROTOBUF} ${Protobuf_VERSION}")
message(STATUS "ZeroMQ         : ${MVSIM_WITH_ZMQ} ${ZeroMQ_VERSION}")
message(STATUS "")
