cmake_minimum_required(VERSION 3.8)
project(g1_nav_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# 只用 libpcl（features/search），不依赖 pcl_ros；lightning-lm 已依赖 PCL，Thor 上现成
find_package(PCL REQUIRED COMPONENTS common features search kdtree)
find_package(unitree_api REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_go REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
  include/g1_nav_bridge/unitree
  include/g1_nav_bridge/unitree/common
  include/nlohmann
)

add_executable(cmdvel_to_sport src/cmdvel_to_sport.cpp)
target_compile_features(cmdvel_to_sport PRIVATE cxx_std_20)
target_include_directories(cmdvel_to_sport PRIVATE
  include
  include/g1_nav_bridge/unitree
  include/g1_nav_bridge/unitree/common
  include/nlohmann
)
ament_target_dependencies(cmdvel_to_sport
  geometry_msgs rclcpp unitree_api)

add_executable(battery_state_bridge src/battery_state_bridge.cpp)
ament_target_dependencies(battery_state_bridge
  rclcpp sensor_msgs unitree_hg)

# /odommodestate -> /odom（2026-10-24 由 Python 版移植，换算在 include/g1_nav_bridge/sport_to_odom.hpp）
add_executable(crab_comp_frame src/crab_comp_frame.cpp)
target_include_directories(crab_comp_frame PRIVATE include)
ament_target_dependencies(crab_comp_frame rclcpp geometry_msgs nav_msgs tf2_ros)

add_executable(sport_to_odom src/sport_to_odom.cpp)
target_include_directories(sport_to_odom PRIVATE include)
ament_target_dependencies(sport_to_odom rclcpp nav_msgs unitree_go)

ament_python_install_package(${PROJECT_NAME})

# D435 标记点云过滤（只给 STVL realsense_mark 用），算法在 include/g1_nav_bridge/d435_mark_filter.hpp
add_executable(d435_mark_filter src/d435_mark_filter.cpp)
target_include_directories(d435_mark_filter PRIVATE include ${PCL_INCLUDE_DIRS})
target_link_libraries(d435_mark_filter ${PCL_LIBRARIES})
ament_target_dependencies(d435_mark_filter
  rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)

# MID360 近距剔除（STVL 没有 obstacle_min_range），算法在 include/g1_nav_bridge/scan_range_filter.hpp
add_executable(scan_range_filter src/scan_range_filter.cpp)
target_include_directories(scan_range_filter PRIVATE include)
ament_target_dependencies(scan_range_filter rclcpp sensor_msgs tf2 tf2_ros)

install(TARGETS cmdvel_to_sport battery_state_bridge d435_mark_filter scan_range_filter sport_to_odom crab_comp_frame
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
# sport_to_odom.py 不再装成可执行（C++ 版同名替代），模块仍随 ament_python_install_package 安装供测试对照
install(PROGRAMS g1_nav_bridge/tf_to_current_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tf_to_current_pose)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md ODOMETRY.md
  DESTINATION share/${PROJECT_NAME})
install(FILES third_party/unitree_ros2/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses
  RENAME unitree_ros2-BSD-3-Clause.txt)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_d435_mark_filter test/test_d435_mark_filter.cpp)
  target_include_directories(test_d435_mark_filter PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_d435_mark_filter ${PCL_LIBRARIES})
  ament_add_gtest(test_scan_range_filter test/test_scan_range_filter.cpp)
  target_include_directories(test_scan_range_filter PRIVATE include)
  ament_target_dependencies(test_scan_range_filter sensor_msgs)
  ament_add_gtest(test_crab_comp test/test_crab_comp.cpp)
  target_include_directories(test_crab_comp PRIVATE include)
  ament_add_gtest(test_cmdvel_rewrite test/test_cmdvel_rewrite.cpp)
  target_include_directories(test_cmdvel_rewrite PRIVATE include)
  ament_add_gtest(test_sport_to_odom_cpp test/test_sport_to_odom.cpp)
  target_include_directories(test_sport_to_odom_cpp PRIVATE include)
  ament_target_dependencies(test_sport_to_odom_cpp nav_msgs unitree_go)
endif()

ament_package()
