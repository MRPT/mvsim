project(mvsim-server)

# === Define executable ===
add_executable(${PROJECT_NAME} mvsim_main.cpp)
target_link_libraries(
	${PROJECT_NAME}
	mvsim::mvsim # the simulator lib
	${MRPT_LIBRARIES}  # MRPT libs
	${BOX2D_LIBRARIES}  # Box2D libs
	${CMAKE_THREAD_LIBS_INIT}
	)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
