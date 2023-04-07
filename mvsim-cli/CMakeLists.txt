project(mvsim)

# === Define executable ===
add_executable(${PROJECT_NAME} 
	mvsim-cli-main.cpp
	mvsim-cli-node.cpp
	mvsim-cli-topic.cpp
	mvsim-cli-launch.cpp
	mvsim-cli-server.cpp
	mvsim-cli.h
)
target_link_libraries(
	${PROJECT_NAME}
	mvsim::simulator
	mvsim::msgs
	mvsim::comms
	mrpt::tclap
	${BOX2D_LIBRARIES}  # Box2D libs
	${CMAKE_THREAD_LIBS_INIT}
	)

mvsim_set_target_build_options(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
