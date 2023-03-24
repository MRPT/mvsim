set(MODULE_NAME "simulator")
project(mvsim-${MODULE_NAME} LANGUAGES CXX VERSION ${MVSIM_VERSION})

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

# Declare the target (a library)
add_library(${PROJECT_NAME}
	${ROOT_SRCS}
	${Sensors_SRCS}
	${Friction_SRCS}
	${VehicleDynamics_SRCS}
	${WorldElements_SRCS}
)
add_library(mvsim::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

mvsim_set_target_build_options(${PROJECT_NAME})

source_group(Main FILES ${ROOT_SRCS})
source_group(Sensors FILES ${Sensors_SRCS})
source_group(Friction FILES ${Friction_SRCS})
source_group(VehicleDynamics FILES ${VehicleDynamics_SRCS})
source_group(WorldElements FILES ${WorldElements_SRCS})

set(PACKAGE_DEPENDENCIES
  mrpt-maps
  mrpt-gui
  mvsim-comms
  box2d
  )
mvsim_common_target_settings(${MODULE_NAME})


target_link_libraries(${PROJECT_NAME}
 PRIVATE
	mvsim::msgs
#	${CMAKE_THREAD_LIBS_INIT}
#	${Protobuf_LIBRARIES}
#	${ZeroMQ_LIBRARY}
PUBLIC
	mrpt::obs
	mrpt::opengl
	mrpt::maps
	mrpt::gui
	mvsim::comms
	${BOX2D_LIBRARIES}
)


target_include_directories(${PROJECT_NAME}
	SYSTEM PUBLIC
		$<BUILD_INTERFACE:${mvsim_SOURCE_DIR}/externals/rapidxml>
)

