project(libmvsim LANGUAGES CXX VERSION 0.3.0)

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

if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif(MSVC)

# Declare the target (a library)
add_library(${PROJECT_NAME}
	${ROOT_SRCS}
	${Sensors_SRCS}
	${Friction_SRCS}
	${VehicleDynamics_SRCS}
	${WorldElements_SRCS}
)

# To make code portable between using directly the mvsim source tree, or 
# importing exported cmake targets:
add_library(mvsim::mvsim   ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${CMAKE_THREAD_LIBS_INIT}
  PUBLIC
    ${MRPT_LIBRARIES}
    ${BOX2D_LIBRARIES}
)

source_group(General FILES ${ROOT_SRCS})
source_group(Sensors FILES ${Sensors_SRCS})
source_group(Friction FILES ${Friction_SRCS})
source_group(VehicleDynamics FILES ${VehicleDynamics_SRCS})
source_group(WorldElements FILES ${WorldElements_SRCS})

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
)

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
	target_compile_options(${PROJECT_NAME} PRIVATE
        -Wall -pedantic
    )

	if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
		target_compile_options(${PROJECT_NAME} PRIVATE
            "-O3"
        )
	endif()
endif()

if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /wd4275
        /wd4251
   )
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


message(STATUS "Configured libmvsim (version ${PROJECT_VERSION})")
