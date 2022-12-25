project(tests)


set(TESTS_DIR ${CMAKE_CURRENT_LIST_DIR})
set(MVSIM_CLI_EXE_PATH $<TARGET_FILE:mvsim>)


# Basic sensor readings
add_test(
    NAME Still2DLidarRanges
     COMMAND ${CMAKE_COMMAND}
     -E env
        PYTHONPATH=${CMAKE_BINARY_DIR}:$ENV{PYTHONPATH}
        TESTS_DIR=${TESTS_DIR}
        MVSIM_CLI_EXE_PATH=${MVSIM_CLI_EXE_PATH}
        ${PYTHON_EXECUTABLE} ${TESTS_DIR}/test-still-lidar2d.py
)
# Not needed?
#    LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/modules/comms/lib/:${CMAKE_BINARY_DIR}/modules/simulator/lib/:${CMAKE_BINARY_DIR}/modules/msgs/lib/:$ENV{LD_LIBRARY_PATH}
