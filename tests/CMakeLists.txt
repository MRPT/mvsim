project(tests)

set(TESTS_DIR ${CMAKE_CURRENT_LIST_DIR})
set(MVSIM_CLI_EXE_PATH $<TARGET_FILE:mvsim>)
set(MVSIM_COMMS_LIB_PATH $<TARGET_FILE_DIR:mvsim-comms>)
set(MVSIM_MSGS_LIB_PATH $<TARGET_FILE_DIR:mvsim-msgs>)
set(MVSIM_SIMULATOR_LIB_PATH $<TARGET_FILE_DIR:mvsim-simulator>)


# ----------------------------------------------------------
# Python tests
# ----------------------------------------------------------
# Basic sensor readings
add_test(
    NAME Still2DLidarRanges
     COMMAND ${CMAKE_COMMAND}
     -E env
        PYTHONPATH=${CMAKE_BINARY_DIR}:$ENV{PYTHONPATH}
        LD_LIBRARY_PATH=${MVSIM_COMMS_LIB_PATH}:${MVSIM_MSGS_LIB_PATH}:${MVSIM_SIMULATOR_LIB_PATH}:$ENV{LD_LIBRARY_PATH}
        TESTS_DIR=${TESTS_DIR}
        MVSIM_CLI_EXE_PATH=${MVSIM_CLI_EXE_PATH}
        ${PYTHON_EXECUTABLE} ${TESTS_DIR}/test-still-lidar2d.py
)
# Not needed?
#    LD_LIBRARY_PATH=${CMAKE_BINARY_DIR}/modules/comms/lib/:${CMAKE_BINARY_DIR}/modules/simulator/lib/:${CMAKE_BINARY_DIR}/modules/msgs/lib/:$ENV{LD_LIBRARY_PATH}


# ----------------------------------------------------------
# C++ tests
# ----------------------------------------------------------
mvsim_add_test(
	TARGET test_shape2p5
	SOURCES test_shape2p5.cpp
	LINK_LIBRARIES mvsim::simulator
	)

mvsim_add_test(
	TARGET test_imu_noise_model
	SOURCES test_imu_noise_model.cpp
	LINK_LIBRARIES mvsim::simulator
	)
