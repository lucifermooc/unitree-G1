cmake_minimum_required(VERSION 3.8)
project(point_filter)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# MID360 CustomMsg -> PointCloud2，按 tag / 抽点 / 盲区 / 高度剔除，算法在 include/point_filter/livox_point_filter.hpp
add_executable(point_filter_node src/point_filter_node.cpp)
target_include_directories(point_filter_node PRIVATE include)
ament_target_dependencies(point_filter_node rclcpp sensor_msgs livox_ros_driver2)

install(TARGETS point_filter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_livox_point_filter test/test_livox_point_filter.cpp)
  target_include_directories(test_livox_point_filter PRIVATE include)
  ament_target_dependencies(test_livox_point_filter sensor_msgs livox_ros_driver2)
endif()

ament_package()
