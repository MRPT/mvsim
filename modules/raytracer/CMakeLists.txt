set(MODULE_NAME "raytracer")
project(mvsim-${MODULE_NAME} LANGUAGES CXX VERSION ${MVSIM_VERSION})

set(RayTracer_SRCS
	src/BVH.cpp
	src/Lidar3DModel.cpp
	src/Primitive.cpp
	src/RayScene.cpp
	src/VecOps.h
	include/mvsim/raytracer/AABB.h
	include/mvsim/raytracer/BVH.h
	include/mvsim/raytracer/Lidar3DModel.h
	include/mvsim/raytracer/Primitive.h
	include/mvsim/raytracer/Ray.h
	include/mvsim/raytracer/RayScene.h
)

# Declare the target (a library). No Box2D, no GUI, no ZMQ, no Protobuf:
# this module only depends on MRPT math/poses so it can be unit-tested
# in isolation and reused from contexts that don't want the full simulator.
add_library(${PROJECT_NAME}
	${RayTracer_SRCS}
)
mvsim_set_target_build_options(${PROJECT_NAME})

source_group(RayTracer FILES ${RayTracer_SRCS})

set(PACKAGE_DEPENDENCIES
  mrpt_math
  mrpt_poses
  )
mvsim_common_target_settings(${MODULE_NAME})

target_link_libraries(${PROJECT_NAME}
 PUBLIC
	mrpt::mrpt_math
	mrpt::mrpt_poses
)
