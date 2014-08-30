# CMake file for the library target
PROJECT(LIBMVSIM)

file(GLOB ROOT_SRCS 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/*.cpp" 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/*.h"
	"${${PROJECT_NAME}_SOURCE_DIR}/include/mvsim/*.h"
	)
file(GLOB Sensors_SRCS 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/Sensors/*.cpp" 
	"${${PROJECT_NAME}_SOURCE_DIR}/include/mvsim/Sensors/*.h"
	)
file(GLOB Friction_SRCS 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/FrictionModels/*.cpp" 
	"${${PROJECT_NAME}_SOURCE_DIR}/include/mvsim/FrictionModels/*.h"
	)
file(GLOB VehicleDynamics_SRCS 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/VehicleDynamics/*.cpp" 
	"${${PROJECT_NAME}_SOURCE_DIR}/include/mvsim/VehicleDynamics/*.h"
	)
file(GLOB WorldElements_SRCS 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/WorldElements/*.cpp" 
	"${${PROJECT_NAME}_SOURCE_DIR}/include/mvsim/WorldElements/*.h"
	)

IF(MSVC)
	ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
ENDIF(MSVC)

INCLUDE_DIRECTORIES("${${PROJECT_NAME}_SOURCE_DIR}/src")
INCLUDE_DIRECTORIES(".")

# Declare the target (a library)
ADD_LIBRARY(mvsim STATIC 
	${ROOT_SRCS} 
	${Sensors_SRCS}
	${Friction_SRCS}
	${VehicleDynamics_SRCS}
	${WorldElements_SRCS}
	)
TARGET_LINK_LIBRARIES(mvsim ${MRPT_LIBS} ${BOX2D_LIBRARIES})

source_group(General FILES ${ROOT_SRCS})
source_group(Sensors FILES ${Sensors_SRCS})
source_group(Friction FILES ${Friction_SRCS})
source_group(VehicleDynamics FILES ${VehicleDynamics_SRCS})
source_group(WorldElements FILES ${WorldElements_SRCS})
