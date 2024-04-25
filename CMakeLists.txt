cmake_minimum_required(VERSION 3.15)
project(factor_graph_state_estimation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(GTSAM REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} gtsam::gtsam)
