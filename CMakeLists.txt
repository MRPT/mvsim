#*+-------------------------------------------------------------------------+
# |                       MultiVehicle simulator (libmvsim)                 |
# |                                                                         |
# |  https://github.com/ual-arm-ros-pkg/multivehicle-simulator              |
# |                                                                         |
# | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
# | Distributed under GNU General Public License version 3                  |
# |   See <http://www.gnu.org/licenses/>                                    |
# +-------------------------------------------------------------------------+

cmake_minimum_required(VERSION 3.1)
set(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)
project(mvsim)  # Must match the ROS package name.
#set (CMAKE_CXX_STANDARD 14)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
      if(POLICY CMP0048)
      	cmake_policy(SET CMP0048 OLD)  # project version
      endif()
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

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin
#        REQUIRED  # mvsim can be built with or without ROS, as an standalong lib
	QUIET
	COMPONENTS
	roscpp tf mrpt_bridge dynamic_reconfigure std_msgs nav_msgs sensor_msgs visualization_msgs dynamic_reconfigure message_generation
	)

set(BUILD_FOR_ROS FALSE)
if (${catkin_FOUND})
	set(BUILD_FOR_ROS TRUE)
	message(STATUS " ==== multivehicle-simulator: ROS' catkin detected. ROS nodes will be built. ===== ")
	add_definitions(-DMVSIM_HAS_ROS)
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

set(EXTERNALS_DIR "${PROJECT_SOURCE_DIR}/externals/")

# --------------------------
# Dependency: MRPT
# --------------------------
find_package( MRPT REQUIRED COMPONENTS opengl obs maps gui tfest)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
if(MRPT_VERSION VERSION_LESS "1.9.9")
	set (CMAKE_CXX_STANDARD 14)
endif()
# Defines: MRPT_LIBS -> libs to link against

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
	endif (NOT "" STREQUAL "${BOX2D_INCLUDE_DIR}")

	# It seems this var is not always set (!?)
	if ("" STREQUAL "${BOX2D_LIBRARIES}")
		set(BOX2D_LIBRARIES Box2D)
	endif ("" STREQUAL "${BOX2D_LIBRARIES}")
endif()


# --------------------------
# Dependency: rapidxml
# --------------------------
# headers-only:
include_directories("${EXTERNALS_DIR}/rapidxml")


# --------------------------
#       The Library
# --------------------------
include_directories("${${PROJECT_NAME}_SOURCE_DIR}/libmvsim/include/")

add_subdirectory(libmvsim)

# --------------------------
#       Apps
# --------------------------
add_subdirectory(mvsim_app) # Standalone app

# -----------------------------------------------------------------------------
#                                   ROS SEGMENT
# -----------------------------------------------------------------------------
if (BUILD_FOR_ROS)
	################################################
	## Declare ROS messages, services and actions ##
	################################################

	## Generate messages in the 'msg' folder
	add_message_files(
	  FILES
	  NodeExampleData.msg
	)

	## Generate services in the 'srv' folder
	# add_service_files(
	#   FILES
	#   Service1.srv
	#   Service2.srv
	# )

	## Generate actions in the 'action' folder
	# add_action_files(
	#   FILES
	#   Action1.action
	#   Action2.action
	# )

	generate_messages(
	  DEPENDENCIES
	  std_msgs
	)

	#add dynamic reconfigure api
	generate_dynamic_reconfigure_options(
	  cfg/mvsimNode.cfg
	)

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
			dynamic_reconfigure message_runtime nav_msgs roscpp std_msgs sensor_msgs visualization_msgs
	#  INCLUDE_DIRS include
	#  LIBRARIES mrpt_localization
	#  DEPENDS mrpt
	)

	###########
	## Build ##
	###########

	## Specify additional locations of header files
	## Your package locations should be listed before other locations
	include_directories("mvsim_node_src/include" ${catkin_INCLUDE_DIRS})
	include_directories(${BOX2D_INCLUDE_DIR}) # Required by mvsim lib

	###########
	## Build ##
	###########

	## Declare a cpp executable
	add_executable(mvsim_node
	    mvsim_node_src/mvsim_node.cpp
	    mvsim_node_src/mvsim_node_main.cpp
	)

	## Add cmake target dependencies of the executable/library
	## as an example, message headers may need to be generated before nodes
	# add_dependencies(mrpt_localization_node mrpt_localization_generate_messages_cpp)
	add_dependencies(mvsim_node
		mvsim_gencfg
		mvsim # the simulator lib
		)

	# make sure configure headers are built before any node using them
	if (${PROJECT_NAME}_EXPORTED_TARGETS})
		add_dependencies(mvsim_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
	endif()

	## Specify libraries to link a library or executable target against
        target_link_libraries(
                mvsim_node
                mvsim  # the simulator lib
                ${BOX2D_LIBRARIES}  # Box2D libs
		${MRPT_LIBRARIES}
                ${catkin_LIBRARIES}
		)

	#############
	## Install ##
	#############

	# all install targets should use catkin DESTINATION variables
	# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

	## Mark executable scripts (Python etc.) for installation
	## in contrast to setup.py, you can choose the destination
	# install(PROGRAMS
	#   scripts/my_python_script
	#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	# )

	# Mark executables and/or libraries for installation
	install(TARGETS mvsim_node
	  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

	# Mark other files for installation (e.g. launch and bag files, etc.)
	install(DIRECTORY
	  mvsim_tutorial
	  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)

	## Mark other files for installation (e.g. launch and bag files, etc.)
	# install(FILES
	#   # myfile1
	#   # myfile2
	#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	# )

endif() # BUILD_FOR_ROS


# -----------------------------------------------------------------------------
#                                MOOS/OpenMORA SEGMENT
# -----------------------------------------------------------------------------
#if(xxx)
#endif() #
