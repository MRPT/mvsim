# CMake file for the library target
PROJECT(LIBMV2DSIM)

file(GLOB ROOT_SRCS 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/*.cpp" 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/*.h"
	"${${PROJECT_NAME}_SOURCE_DIR}/include/mv2dsim/*.h"
	)
file(GLOB Sensors_SRCS 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/Sensors/*.cpp" 
	"${${PROJECT_NAME}_SOURCE_DIR}/include/mv2dsim/Sensors/*.h"
	)
file(GLOB Friction_SRCS 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/FrictionModels/*.cpp" 
	"${${PROJECT_NAME}_SOURCE_DIR}/include/mv2dsim/FrictionModels/*.h"
	)
file(GLOB VehicleDynamics_SRCS 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/VehicleDynamics/*.cpp" 
	"${${PROJECT_NAME}_SOURCE_DIR}/include/mv2dsim/VehicleDynamics/*.h"
	)
file(GLOB WorldElements_SRCS 
	"${${PROJECT_NAME}_SOURCE_DIR}/src/WorldElements/*.cpp" 
	"${${PROJECT_NAME}_SOURCE_DIR}/include/mv2dsim/WorldElements/*.h"
	)

IF(MSVC)
	ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
ENDIF(MSVC)

INCLUDE_DIRECTORIES("${${PROJECT_NAME}_SOURCE_DIR}/src")
INCLUDE_DIRECTORIES(".")

# Declare the target (a library)
ADD_LIBRARY(mv2dsim STATIC 
	${ROOT_SRCS} 
	${Sensors_SRCS}
	${Friction_SRCS}
	${VehicleDynamics_SRCS}
	${WorldElements_SRCS}
	)
TARGET_LINK_LIBRARIES(mv2dsim ${MRPT_LIBS} ${BOX2D_LIBRARIES})

source_group(General FILES ${ROOT_SRCS})
source_group(Sensors FILES ${Sensors_SRCS})
source_group(Friction FILES ${Friction_SRCS})
source_group(VehicleDynamics FILES ${VehicleDynamics_SRCS})
source_group(WorldElements FILES ${WorldElements_SRCS})
