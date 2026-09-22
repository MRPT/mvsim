project(mvsim-dataset-gen)

find_package(CLI11 REQUIRED)

add_executable(${PROJECT_NAME}
	main.cpp
	SceneBuilder.h
	SceneBuilder.cpp
	TrajectorySource.h
	TrajectorySource.cpp
	LidarSimulator.h
	LidarSimulator.cpp
)
target_link_libraries(
	${PROJECT_NAME}
	mvsim::simulator
	mvsim::raytracer
	CLI11::CLI11
	${CMAKE_THREAD_LIBS_INIT}
)

mvsim_set_target_build_options(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
