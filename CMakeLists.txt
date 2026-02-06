#*+-------------------------------------------------------------------------+
# |                       MultiVehicle simulator (libmvsim)                 |
# |                                                                         |
# |  https://github.com/ual-arm-ros-pkg/multivehicle-simulator              |
# |                                                                         |
# | Copyright (C) 2014-2026  Jose Luis Blanco Claraco                       |
# | Distributed under 3-clause BSD License                                  |
# |   See COPYING                                                           |
# +-------------------------------------------------------------------------+

cmake_minimum_required(VERSION 3.9) # for CMAKE_MATCH_1

if("$ENV{ROS_VERSION}" STREQUAL "2")
  set(DETECTED_ROS2 TRUE)
  set(PACKAGE_ROS_VERSION 2)
elseif("$ENV{ROS_VERSION}" STREQUAL "1")
  set(DETECTED_ROS1 TRUE)
  set(PACKAGE_ROS_VERSION 1)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(default_build_type "Release")
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
endif()
# Set the possible values of build type for cmake-gui
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo" "SanitizeAddress" "SanitizeThread")

# CMake file for the library target
project(mvsim)

#----
# Extract version from package.xml
# Example line:" <version>0.3.2</version>"
file(READ package.xml contentPackageXML)
string(REGEX MATCH "<version>([0-9\.]*)</version>" _ ${contentPackageXML})
set(MVSIM_VERSION ${CMAKE_MATCH_1})
message(STATUS "MVSIM version: ${MVSIM_VERSION} (detected in package.xml)")
string(REGEX MATCH "^([0-9]+)\\.([0-9]+)\\.([0-9]+)" _ ${MVSIM_VERSION})
set(MVSIM_MAJOR_VERSION ${CMAKE_MATCH_1})
set(MVSIM_MINOR_VERSION ${CMAKE_MATCH_2})
set(MVSIM_PATCH_VERSION ${CMAKE_MATCH_3})
#TODO: Avoid overwriting source dir.
configure_file(
  cmake/mvsim_version.h.in
  ${CMAKE_CURRENT_SOURCE_DIR}/modules/simulator/include/mvsim/mvsim_version.h
  @ONLY
  )
#----

# This is required for pybind11 to generate C++17 code in all OS versions:
set(CMAKE_CXX_STANDARD 17 CACHE STRING "C++ version selection")
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # optional, ensure standard is supported
set(CMAKE_CXX_EXTENSIONS OFF)  # optional, keep compiler extensions off


find_package(Threads)
include(GNUInstallDirs)
include(cmake/mvsim_cmake_functions.cmake REQUIRED)

if (POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW) # IN_LIST (needed by pybind11 cmake scripts)
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  # BUILD_TYPE: SanitizeAddress
  set(CMAKE_CXX_FLAGS_SANITIZEADDRESS "-fsanitize=address  -fsanitize=leak -g")
  set(CMAKE_EXE_LINKER_FLAGS_SANITIZEADDRESS "-fsanitize=address  -fsanitize=leak")
  set(CMAKE_SHARED_LINKER_FLAGS_SANITIZEADDRESS "-fsanitize=address  -fsanitize=leak")

  # BUILD_TYPE: SanitizeThread
  set(CMAKE_CXX_FLAGS_SANITIZETHREAD "-fsanitize=thread -g")
  set(CMAKE_EXE_LINKER_FLAGS_SANITIZETHREAD "-fsanitize=thread")
  set(CMAKE_SHARED_LINKER_FLAGS_SANITIZETHREAD "-fsanitize=thread")
endif()


# ------------------------------------------------------------------
#                              IMPORTANT NOTE
#
#  This package can be built as:
#  1) Standalone lib+app
#  2) ROS1 or ROS2 node(s)
# Depending on what packages are found, this script will instruct
# CMake to configure the project accordingly.
# ------------------------------------------------------------------

# ROS1:
set(BUILD_FOR_ROS FALSE)
if (DETECTED_ROS1)
  set(BUILD_FOR_ROS TRUE)
endif()

# ROS2:
if (DETECTED_ROS2)
  set(BUILD_FOR_ROS TRUE)

  find_package(rclcpp REQUIRED)
endif()


if (BUILD_FOR_ROS)
  message(STATUS " ==== multivehicle-simulator: ROS$ENV{ROS_VERSION} detected. ROS nodes will be built. ===== ")
  add_definitions(-DMVSIM_HAS_ROS)
endif()


# -----------------------------------------------------------------------------
#  ROS1
# -----------------------------------------------------------------------------
if (BUILD_FOR_ROS AND DETECTED_ROS1)
  # Find catkin macros and libraries
  # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  # is used, also find other catkin packages
  find_package(catkin
    REQUIRED
    COMPONENTS
    roscpp tf2 dynamic_reconfigure std_msgs nav_msgs sensor_msgs visualization_msgs
    tf2_geometry_msgs
  )
  find_package(mrpt-ros1bridge REQUIRED)

  #add dynamic reconfigure api
  if (dynamic_reconfigure_FOUND)
    generate_dynamic_reconfigure_options(
      cfg/mvsimNode.cfg
      )
  endif()

  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if you package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also need

  catkin_package(
    CATKIN_DEPENDS
      dynamic_reconfigure nav_msgs roscpp sensor_msgs visualization_msgs
      tf2 tf2_geometry_msgs
  #  INCLUDE_DIRS include
  #  LIBRARIES mrpt_localization
  #  DEPENDS mrpt
  )
endif()

# -----------------------------------------------------------------------------
#  ROS2
# -----------------------------------------------------------------------------
if (BUILD_FOR_ROS AND DETECTED_ROS2)
  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  find_package(mrpt-ros2bridge  REQUIRED)
endif()


# --------------------------
# Build options
# --------------------------
if (UNIX)
  set(DEFAULT_SHARED_LIBS ON)
else()
  set(DEFAULT_SHARED_LIBS OFF)
endif()
set(BUILD_SHARED_LIBS ${DEFAULT_SHARED_LIBS} CACHE BOOL "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")

# Save all libs and executables in the same place
if (NOT BUILD_FOR_ROS)
  set( LIBRARY_OUTPUT_PATH ${${PROJECT_NAME}_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
  set( EXECUTABLE_OUTPUT_PATH ${${PROJECT_NAME}_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )
endif()

set(CMAKE_DEBUG_POSTFIX "-dbg")

# --------------------------
# Global compiler flags
# --------------------------
if(MSVC)
  # Force usage of UNICODE projects, which is not the default in MSVC:
  add_definitions(-DUNICODE -D_UNICODE)
endif()

# ----------------------------------------------------------
# Dependency: MRPT
# Use the first cmd to set the minimum required version
# ----------------------------------------------------------
find_package(mrpt-maps  REQUIRED)
find_package(mrpt-tclap  REQUIRED)
find_package(mrpt-gui  REQUIRED)
find_package(mrpt-tfest  REQUIRED)
find_package(mrpt-topography REQUIRED)

if (mrpt-maps_VERSION VERSION_LESS "2.14.1")
    message(FATAL_ERROR "MRPT version 2.14.1 or newer is required, but found ${mrpt-maps_VERSION}")
endif()

# --------------------------
# Dependency: Box2D
# --------------------------
set(EMBEDDED_box2d_BUILD_DIR "${${PROJECT_NAME}_BINARY_DIR}/externals/box2d/")
set(EMBEDDED_box2d_INSTALL_DIR "${${PROJECT_NAME}_BINARY_DIR}/externals/box2d/install/")
set(EMBEDDED_box2d_DIR "${EMBEDDED_box2d_INSTALL_DIR}/lib/cmake/box2d/")

# 1st) Try to locate it via CMake (installed in the system or precompiled somewhere)
# Since box2d 2.4.1, the name changed "Box2D" -> "box2d". We now only support the version >=2.4.x
# for compatibility with modern Ubuntu distros:
#
# Update: Since MVSIM 0.7.0, we now always ship a custom build of box2d to
# increase its default maximum number of polygon vertices:
##find_package(box2d QUIET) # Defines: box2d::box2d

if (NOT box2d_FOUND)
  message(STATUS "--- Running CMake on external submodule 'box2d'...")
  file(MAKE_DIRECTORY "${EMBEDDED_box2d_BUILD_DIR}")
  if(NOT ${CMAKE_VERSION} VERSION_LESS "3.15")
    set(echo_flag COMMAND_ECHO STDOUT)
  endif()
  execute_process(COMMAND
    ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" "${${PROJECT_NAME}_SOURCE_DIR}/externals/box2d"
    -DBOX2D_BUILD_DOCS=OFF
    -DBOX2D_BUILD_TESTBED=OFF
    -DBOX2D_BUILD_UNIT_TESTS=OFF
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON
    -DCMAKE_INSTALL_PREFIX=${EMBEDDED_box2d_INSTALL_DIR}
  RESULT_VARIABLE result
  WORKING_DIRECTORY "${EMBEDDED_box2d_BUILD_DIR}"
  ${echo_flag}
  )
  if(result)
    message(FATAL_ERROR "CMake step for box2d failed: ${result}")
  endif()

  execute_process(COMMAND
    ${CMAKE_COMMAND}
    --build ${EMBEDDED_box2d_BUILD_DIR}
    --parallel
    --target install
  RESULT_VARIABLE result
  ${echo_flag}
  )
  if(result)
    message(FATAL_ERROR "CMake make install step for box2d failed: ${result}")
  endif()
  message(STATUS "--- End running CMake")

  # Search again:
  set(box2d_DIR "${EMBEDDED_box2d_DIR}" CACHE PATH "Path to box2d CMake config file" FORCE)
  mark_as_advanced(box2d_DIR)
  find_package(box2d CONFIG QUIET)

    # install the embedded copy too (we need box2d-config.cmake, etc.)
  execute_process(COMMAND
    ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" "${${PROJECT_NAME}_SOURCE_DIR}/externals/box2d"
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  RESULT_VARIABLE result
  WORKING_DIRECTORY "${EMBEDDED_box2d_BUILD_DIR}"
  ${echo_flag}
  )
    install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} --build \"${EMBEDDED_box2d_BUILD_DIR}\" --target install)")
endif()

if (NOT box2d_FOUND)
    message(FATAL_ERROR "box2d not found, neither as system library nor git submodule. Check error messages above for possible reasons.")
endif()

if (box2d_FOUND)
  set(BOX2D_LIBRARIES box2d::box2d)
endif()

# --------------------------
#  Main library modules
# --------------------------
add_subdirectory(modules)

# --------------------------
#       Apps
# --------------------------
add_subdirectory(mvsim-cli)

if (BUILD_FOR_ROS)
  ###########
  ## Build ##
  ###########

  ## Declare a cpp executable
  add_executable(mvsim_node
    mvsim_node_src/mvsim_node.cpp
    mvsim_node_src/mvsim_node_main.cpp
    mvsim_node_src/include/mvsim/mvsim_node_core.h
    mvsim_node_src/include/wrapper/publisher_wrapper.h
  )

  mvsim_set_target_build_options(mvsim_node)

  target_include_directories(mvsim_node PRIVATE "mvsim_node_src/include")

  if (catkin_INCLUDE_DIRS)
    # Specify additional locations of header files
    # Your package locations should be listed before other locations
    target_include_directories(mvsim_node PUBLIC ${catkin_INCLUDE_DIRS})
  endif()

  if (DETECTED_ROS1)
    add_dependencies(mvsim_node
      mvsim_gencfg
    )

    target_link_libraries(mvsim_node
      #PRIVATE  # ros catkin already used the plain signature, we cannot use this here...
      ${catkin_LIBRARIES}
      mrpt::ros1bridge
    )

    # make sure configure headers are built before any node using them
    if (${PROJECT_NAME}_EXPORTED_TARGETS})
      add_dependencies(mvsim_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
    endif()
  elseif(DETECTED_ROS2)
    target_link_libraries(mvsim_node #PRIVATE
      rclcpp::rclcpp
      tf2::tf2
      mrpt::ros2bridge
      ${geometry_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      ${sensor_msgs_TARGETS}
      ${visualization_msgs_TARGETS}
      ${tf2_geometry_msgs_TARGETS}
    )

    # We need this to handle this backwards-incompatible change: https://github.com/ros2/geometry2/pull/416
    if("${tf2_geometry_msgs_VERSION}" VERSION_GREATER_EQUAL "0.18.0")
      target_compile_definitions(mvsim_node PRIVATE MVSIM_HAS_TF2_GEOMETRY_MSGS_HPP)
    endif()

  endif()

  target_compile_definitions(mvsim_node PRIVATE
  PACKAGE_ROS_VERSION=${PACKAGE_ROS_VERSION}
  )

  if (TARGET mvsim::comms)
    set(DEP_MVSIM_COMMS_TRG mvsim::comms)
  endif()
  if (TARGET mvsim::msgs)
    set(DEP_MVSIM_MSGS_TRG mvsim::msgs)
  endif()


  # Specify libraries to link a library or executable target against
  target_link_libraries(
    mvsim_node
    #PRIVATE  # ros ament already used the plain signature, we cannot use this here...
    mvsim::simulator
    ${DEP_MVSIM_MSGS_TRG}
    ${DEP_MVSIM_COMMS_TRG}
    ${BOX2D_LIBRARIES}  # Box2D libs
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
  )

  if (DETECTED_ROS1)
    target_link_libraries(mvsim_node mrpt::ros1bridge)
  else()
    target_link_libraries(mvsim_node mrpt::ros2bridge)
  endif()

  #############
  ## Install ##
  #############

  # all install targets should use catkin DESTINATION variables
  # See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

  # Mark executables and/or libraries for installation
  if (DETECTED_ROS2)
    set(CATKIN_PACKAGE_LIB_DESTINATION lib)
    set(CATKIN_PACKAGE_BIN_DESTINATION lib/${PROJECT_NAME})
    set(CATKIN_PACKAGE_INCLUDE_DESTINATION include)
    set(CATKIN_PACKAGE_SHARE_DESTINATION share/${PROJECT_NAME})
  endif()

  # Mark executables and/or libraries for installation
  install(TARGETS mvsim_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  # Mark other files for installation (e.g. launch and bag files, etc.)
  install(DIRECTORY
    definitions
    mvsim_tutorial
    models
    launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

  # Mark executables and/or libraries for installation
  if (DETECTED_ROS2)
    #ament_export_dependencies(mrpt-gui)

    # Export old-style CMake variables
    #ament_export_include_directories("include/${PROJECT_NAME}")
    #ament_export_libraries(${PROJECT_NAME})

    # Export modern CMake targets
    #ament_export_targets(export_${PROJECT_NAME})
    ament_package()
  endif()
endif() # BUILD_FOR_ROS

option(MVSIM_UNIT_TESTS "Build and run MVSIM unit tests (requires Python and ZMQ)" ON)

# Disable tests in armhf, since they fail due to (probably?) very slow rendering capabilities.
# See: https://github.com/ros-infrastructure/ros_buildfarm_config/pull/220
if (MVSIM_UNIT_TESTS AND
  (
  ("armhf" STREQUAL "${CMAKE_SYSTEM_PROCESSOR}") OR
  ("arm64" STREQUAL "${CMAKE_SYSTEM_PROCESSOR}") OR
  ("aarch64" STREQUAL "${CMAKE_SYSTEM_PROCESSOR}")
  )
  )
  set(MVSIM_UNIT_TESTS OFF CACHE BOOL "" FORCE)
  message(STATUS "*Warning*: Disabling unit tests in this architecture (${CMAKE_SYSTEM_PROCESSOR})")
endif()

if (MVSIM_UNIT_TESTS AND MVSIM_WITH_PYTHON AND MVSIM_WITH_ZMQ)
  enable_testing()  # This must be in the root cmake file
  add_subdirectory(tests)
endif()

