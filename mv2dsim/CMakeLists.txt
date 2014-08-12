cmake_minimum_required(VERSION 2.8)
project(mv2dsim_app)

include_directories(${BOX2D_INCLUDE_DIR})

# === Define executable ===
add_executable(${PROJECT_NAME} mv2dsim_main.cpp)
target_link_libraries(
	${PROJECT_NAME} 
	mv2dsim # the simulator lib
	${MRPT_LIBS}  # MRPT libs
	${BOX2D_LIBRARIES}  # Box2D libs
	)
	