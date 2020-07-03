cmake_minimum_required(VERSION 3.5)
project(mvsim-service-caller-example)

find_package(mvsim-comms)
find_package(mvsim-msgs)

add_executable(${PROJECT_NAME} ${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
    mvsim::mvsim-comms
    mvsim::mvsim-msgs
)
