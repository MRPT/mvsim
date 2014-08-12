# CMake file for the library target
PROJECT(LIBMV2DSIM)

file(GLOB SRCS_LIST "${${PROJECT_NAME}_SOURCE_DIR}/src/*.cpp")
file(GLOB HRDS_LIST "${${PROJECT_NAME}_SOURCE_DIR}/include/mbde/*.h")
file(GLOB INTERNAL_HDRS_LIST "${${PROJECT_NAME}_SOURCE_DIR}/src/*.h")

# Declare the target (a library)
ADD_LIBRARY(mv2dsim ${SRCS_LIST} ${INTERNAL_HDRS_LIST} ${HRDS_LIST})
