cmake_minimum_required(VERSION 3.5)
project(mvsim-subscriber-example)

find_package(mvsim-comms)
find_package(mvsim-msgs)
find_package(mrpt-obs)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
    mvsim::mvsim-comms
    mvsim::mvsim-msgs
    mrpt::obs
)
