#*+-------------------------------------------------------------------------+
# |                       MultiVehicle simulator (libmvsim)                 |
# |                                                                         |
# |  https://github.com/ual-arm-ros-pkg/multivehicle-simulator              |
# |                                                                         |
# | Copyright (C) 2014-2022  Jose Luis Blanco Claraco                       |
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
	set (CMAKE_CXX_STANDARD 14)
endif()
project(mrpt_msgs)

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
#TODO: Avoid overwritting source dir.
configure_file(
	cmake/mvsim_version.h.in
	${CMAKE_CURRENT_SOURCE_DIR}/modules/simulator/include/mvsim/mvsim_version.h
	@ONLY
	)
#----

find_package(Threads)
include(GNUInstallDirs)

if (POLICY CMP0057)
	cmake_policy(SET CMP0057 NEW) # IN_LIST (needed by pybind11 cmake scripts)
endif()

# ------------------------------------------------------------------
#                              IMPORTANT NOTE
#
#  This package can be built as:
#  1) Standalone lib+app
#  2) ROS node(s)
#  3) MOOS module for OpenMORA
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

	find_package(mrpt-ros2bridge REQUIRED)

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
	# Warnings level
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
	# Force usage of UNICODE projects, which is not the default in MSVC:
	add_definitions(-DUNICODE -D_UNICODE)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
  # Set optimized building:
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
endif()

# --------------------------
# Dependency: MRPT
# --------------------------
find_package(mrpt-maps REQUIRED)
find_package(mrpt-tclap REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-tfest REQUIRED)

# --------------------------
# Dependency: Box2D
# --------------------------
set(EMBEDDED_Box2D_DIR "${${PROJECT_NAME}_BINARY_DIR}/externals/Box2D/Box2D")

# 1st) Try to locate it via CMake (installed in the system or precompiled somewhere)
set(Box2D_DIR "${EMBEDDED_Box2D_DIR}" CACHE PATH "Path to Box2D CMake config file" FORCE)
find_package(Box2D QUIET) # Quiet in a 1st try...
# Defines:

if (NOT Box2D_FOUND OR "${Box2D_DIR}" STREQUAL "${EMBEDDED_Box2D_DIR}")
	# Use embedded version:
	# Override some defaults in the embedded project:
	set(BOX2D_INSTALL ON CACHE BOOL "Install Box2D libs, includes, and CMake scripts" FORCE) # IMPORTANT: This must be ON to enable parsing Box2D's .cmake.in file!!
	option(BOX2D_INSTALL_DOC "Install Box2D documentation" OFF)
	option(BOX2D_BUILD_SHARED "Build Box2D shared libraries" OFF)
	option(BOX2D_BUILD_STATIC "Build Box2D static libraries" ON)
	option(BOX2D_BUILD_EXAMPLES "Build Box2D examples" OFF)
	mark_as_advanced(BOX2D_INSTALL)
	mark_as_advanced(BOX2D_INSTALL_DOC)
	mark_as_advanced(BOX2D_BUILD_SHARED)
	mark_as_advanced(BOX2D_BUILD_STATIC)
	mark_as_advanced(BOX2D_BUILD_EXAMPLES)

	add_subdirectory(externals/Box2D)

	set_property(TARGET Box2D PROPERTY POSITION_INDEPENDENT_CODE ON)

	# 2nd attempt: Fatal error if not found:
	message(STATUS "INFO: Box2D library not found in the system. Using embedded copy.")
	set(Box2D_DIR "${EMBEDDED_Box2D_DIR}" CACHE PATH "Path to Box2D CMake config file" FORCE)
	find_package(Box2D REQUIRED)

	# It seems BOX2D doesn't return the correct #include dir (???):
	set(BOX2D_INCLUDE_DIR "${${PROJECT_NAME}_SOURCE_DIR}/externals/Box2D/")
endif()

if (Box2D_FOUND)
	#message(STATUS "BOX2D_INCLUDE_DIR: ${BOX2D_INCLUDE_DIR}")
	if (NOT "" STREQUAL "${BOX2D_INCLUDE_DIR}")
		include_directories("${BOX2D_INCLUDE_DIR}")
	endif()

	# It seems this var is not always set (!?)
	if ("" STREQUAL "${BOX2D_LIBRARIES}")
		set(BOX2D_LIBRARIES Box2D)
	endif()
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
	)

	if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		# High level of warnings.
		target_compile_options(mvsim_node PRIVATE -Wall -Wextra -Wpedantic)

		# The -Wno-long-long is required in 64bit systems when including sytem headers.
		# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
		target_compile_options(mvsim_node PUBLIC -Wno-long-long -Wno-variadic-macros)
		# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
		if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
			target_compile_options(mvsim_node PUBLIC -Wno-ignored-attributes -Wno-int-in-bool-context)
		endif()
	endif()

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
			PRIVATE
			${catkin_LIBRARIES}
			mrpt::ros1bridge
		)

		# make sure configure headers are built before any node using them
		if (${PROJECT_NAME}_EXPORTED_TARGETS})
			add_dependencies(mvsim_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
		endif()
	elseif(DETECTED_ROS2)
		ament_target_dependencies(mvsim_node
			rclcpp
			geometry_msgs
			nav_msgs
			sensor_msgs
			visualization_msgs
			tf2
			tf2_geometry_msgs
		)

		target_link_libraries(mvsim_node
			PRIVATE
			mrpt::ros2bridge
		)
	endif()

	target_compile_definitions(mvsim_node PRIVATE
	PACKAGE_ROS_VERSION=${PACKAGE_ROS_VERSION}
	)

	# Specify libraries to link a library or executable target against
	target_link_libraries(
		mvsim_node
	PRIVATE
		mvsim::simulator
		mvsim::msgs
		mvsim::comms
		${BOX2D_LIBRARIES}  # Box2D libs
		${catkin_LIBRARIES}
		${CMAKE_THREAD_LIBS_INIT}
	)

	if (DETECTED_ROS1)
		target_link_libraries(mvsim_node PRIVATE mrpt::ros1bridge)
	else()
		target_link_libraries(mvsim_node PRIVATE mrpt::ros2bridge)
	endif()

	#############
	## Install ##
	#############

	# all install targets should use catkin DESTINATION variables
	# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

	# Mark executables and/or libraries for installation
	if (DETECTED_ROS2)
		set(CATKIN_PACKAGE_LIB_DESTINATION lib)
		set(CATKIN_PACKAGE_BIN_DESTINATION bin)
		set(CATKIN_PACKAGE_INCLUDE_DESTINATION include)
		set(CATKIN_PACKAGE_SHARE_DESTINATION share)
	endif()

	# Mark executables and/or libraries for installation
	install(TARGETS mvsim_node
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

	# Mark other files for installation (e.g. launch and bag files, etc.)
	install(DIRECTORY
		mvsim_tutorial
		models
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)
endif() # BUILD_FOR_ROS

