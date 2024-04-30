cmake_minimum_required(VERSION 3.15)
project(factor_graph_state_estimation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(GTSAM REQUIRED)

add_executable(odometry_example src/odometry_example.cpp)
target_link_libraries(odometry_example gtsam::gtsam)

add_executable(localization_example src/localization_example.cpp)
target_link_libraries(localization_example gtsam::gtsam)

add_executable(sensor_fusion src/sensor_fusion.cpp)
target_link_libraries(sensor_fusion gtsam::gtsam)
