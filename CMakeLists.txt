#*+-------------------------------------------------------------------------+
# |                       MultiVehicle 2D simulator (libmv2dsim)            |
# |                                                                         |
# |  https://github.com/ual-arm-ros-pkg/multivehicle_2d_simulator           |
# |                                                                         |
# | Copyright (C) 2014  Jose Luis Blanco Claraco (University of Almeria)    |
# | Distributed under GNU General Public License version 3                  |
# |   See <http://www.gnu.org/licenses/>                                    |
# +-------------------------------------------------------------------------+ 

PROJECT(multivehicle_2d_simulator)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# --------------------------
# Build options
# --------------------------
IF (UNIX)
	SET(DEFAULT_SHARED_LIBS ON)
ELSE(UNIX)
	SET(DEFAULT_SHARED_LIBS OFF)
ENDIF(UNIX)
SET(BUILD_SHARED_LIBS ${DEFAULT_SHARED_LIBS} CACHE BOOL "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")

# Save all libs and executables in the same place
SET( LIBRARY_OUTPUT_PATH ${${PROJECT_NAME}_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
SET( EXECUTABLE_OUTPUT_PATH ${${PROJECT_NAME}_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )

set(EXTERNALS_DIR "${PROJECT_SOURCE_DIR}/externals/")

# --------------------------
# Dependency: MRPT
# --------------------------
FIND_PACKAGE( MRPT REQUIRED COMPONENTS base opengl obs maps)
# Defines: MRPT_LIBS -> libs to link against

# --------------------------
# Dependency: Box2D
# --------------------------
SET(EMBEDDED_Box2D_DIR "${${PROJECT_NAME}_BINARY_DIR}/externals/Box2D/Box2D")


# 1st) Try to locate it via CMake (installed in the system or precompiled somewhere)
FIND_PACKAGE(Box2D QUIET) # Quiet in a 1st try...
# Defines: 

IF (NOT Box2D_FOUND OR "${Box2D_DIR}" STREQUAL "${EMBEDDED_Box2D_DIR}")
	# Use embedded version:
	# Override some defaults in the embedded project:
	set(BOX2D_INSTALL ON CACHE BOOL "Install Box2D libs, includes, and CMake scripts" FORCE) # IMPORTANT: This must be ON to enable parsing Box2D's .cmake.in file!!	
	option(BOX2D_INSTALL_DOC "Install Box2D documentation" OFF)
	option(BOX2D_BUILD_SHARED "Build Box2D shared libraries" OFF)
	option(BOX2D_BUILD_STATIC "Build Box2D static libraries" ON)
	option(BOX2D_BUILD_EXAMPLES "Build Box2D examples" OFF)
	MARK_AS_ADVANCED(BOX2D_INSTALL)
	MARK_AS_ADVANCED(BOX2D_INSTALL_DOC)
	MARK_AS_ADVANCED(BOX2D_BUILD_SHARED)
	MARK_AS_ADVANCED(BOX2D_BUILD_STATIC)
	MARK_AS_ADVANCED(BOX2D_BUILD_EXAMPLES)

	add_subdirectory(externals/Box2D)

	SET(Box2D_DIR "${EMBEDDED_Box2D_DIR}" CACHE PATH "Path to Box2D CMake config file" FORCE)
	MESSAGE(STATUS "[Info] Box2D not found in the system. Using embedded version in: ${Box2D_DIR}")

	# 2nd attempt: Fatal error if not found:
	FIND_PACKAGE(Box2D REQUIRED)
	
	# It seems BOX2D doesn't return the correct #include dir (???):
	SET(BOX2D_INCLUDE_DIR "${${PROJECT_NAME}_SOURCE_DIR}/externals/Box2D/")
	
ENDIF (NOT Box2D_FOUND OR "${Box2D_DIR}" STREQUAL "${EMBEDDED_Box2D_DIR}")

IF (Box2D_FOUND)
	#MESSAGE(STATUS "BOX2D_INCLUDE_DIR: ${BOX2D_INCLUDE_DIR}")
	INCLUDE_DIRECTORIES("${BOX2D_INCLUDE_DIR}")
	LINK_DIRECTORIES("${BOX2D_LIBRARY_DIRS}")
ENDIF (Box2D_FOUND)


# --------------------------
# Dependency: rapidxml
# --------------------------
# headers-only:
INCLUDE_DIRECTORIES("${EXTERNALS_DIR}/rapidxml")

# --------------------------
# Global compiler flags
# --------------------------
IF(MSVC)
	# Warnings level
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3") 
	# Force usage of UNICODE projects, which is not the default in MSVC:
	add_definitions(-DUNICODE -D_UNICODE)
ENDIF(MSVC)

IF(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	# Warnings level
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")
	# Set optimized building:
	IF(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
	ENDIF(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
ENDIF(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")

# --------------------------
#       The Library 
# --------------------------
INCLUDE_DIRECTORIES("${${PROJECT_NAME}_SOURCE_DIR}/libmv2dsim/include/")

add_subdirectory(libmv2dsim)

# --------------------------
#       Apps
# --------------------------
add_subdirectory(mv2dsim) # Standalone app
#add_subdirectory(mv2dsim_ros)   # ROS node

