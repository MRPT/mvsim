cmake_minimum_required(VERSION 2.8)
project(mvsim_app)

include_directories(${BOX2D_INCLUDE_DIR})

# === Define executable ===
add_executable(${PROJECT_NAME} mvsim_main.cpp)
target_link_libraries(
	${PROJECT_NAME} 
	mvsim # the simulator lib
	${MRPT_LIBS}  # MRPT libs
	${BOX2D_LIBRARIES}  # Box2D libs
	)
