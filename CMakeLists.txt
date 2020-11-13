cmake_minimum_required(VERSION 3.16)
project(robot_v3)

INCLUDE_DIRECTORIES(/opt/etherlab/include)
LINK_DIRECTORIES(/opt/etherlab/lib)

set(CMAKE_CXX_STANDARD 17)

add_executable(robot_v3 main.cpp robot.h robot.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME} ethercat pthread)