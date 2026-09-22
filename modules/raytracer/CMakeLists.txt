set(MODULE_NAME "raytracer")
project(mvsim-${MODULE_NAME} LANGUAGES CXX VERSION ${MVSIM_VERSION})

file(GLOB RayTracer_SRCS
	"${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/mvsim/raytracer/*.h"
)

# Declare the target (a library). No Box2D, no GUI, no ZMQ, no Protobuf:
# this module only depends on MRPT math/poses/maps so it can be unit-tested
# in isolation and reused from contexts that don't want the full simulator.
add_library(${PROJECT_NAME}
	${RayTracer_SRCS}
)
mvsim_set_target_build_options(${PROJECT_NAME})

source_group(RayTracer FILES ${RayTracer_SRCS})

set(PACKAGE_DEPENDENCIES
  mrpt-math
  mrpt-poses
  mrpt-maps
  )
mvsim_common_target_settings(${MODULE_NAME})

target_link_libraries(${PROJECT_NAME}
 PUBLIC
	mrpt::mrpt_math
	mrpt::mrpt_poses
	mrpt::mrpt_maps
)
