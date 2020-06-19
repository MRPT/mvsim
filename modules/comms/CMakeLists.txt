set(MODULE_NAME "comms")
project(mvsim-${MODULE_NAME} LANGUAGES CXX VERSION ${MVSIM_VERSION})

file(GLOB Comms_SRCS
	"${CMAKE_CURRENT_SOURCE_DIR}/src/Comms/*.cpp"
	"${CMAKE_CURRENT_SOURCE_DIR}/include/mvsim/Comms/*.h"
)

# Declare the target (a library)
add_library(${PROJECT_NAME}
	${Comms_SRCS}
)

source_group(Comms FILES ${Comms_SRCS})

mvsim_common_target_settings(${MODULE_NAME})

target_link_libraries(${PROJECT_NAME} PRIVATE mvsim::msgs)

