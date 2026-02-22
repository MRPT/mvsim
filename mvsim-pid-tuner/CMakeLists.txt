project(mvsim-pid-tuner)

# === Define executable ===
add_executable(${PROJECT_NAME}
	mvsim-pid-tuner-main.cpp
)
target_link_libraries(
	${PROJECT_NAME}
	mvsim::simulator
	mrpt::tclap
	${BOX2D_LIBRARIES}
	${CMAKE_THREAD_LIBS_INIT}
	)

mvsim_set_target_build_options(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
