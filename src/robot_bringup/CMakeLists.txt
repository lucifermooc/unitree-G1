################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(robot_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(aid_robot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_map_server)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lightning REQUIRED)

include_directories(
  include
)


set(DEPENDENCIES
  "rclcpp"
  "ament_index_cpp"
  "geometry_msgs"
  "nav2_msgs"
  "aid_robot_msgs"
  "std_msgs"
  "sensor_msgs"
  "nav2_util"
  "std_srvs"
  "nav2_map_server"
  "visualization_msgs"
  "lightning"

)

add_library(robot_bringup_lib SHARED
src/map_utils.cpp
)
ament_target_dependencies(robot_bringup_lib ${DEPENDENCIES})

add_executable(robot_status_manager_node src/robot_status_manager.cpp)
ament_target_dependencies(robot_status_manager_node ${DEPENDENCIES})
target_link_libraries(robot_status_manager_node robot_bringup_lib)

add_executable(forbidden_map_create_node src/forbidden_map_create.cpp)
ament_target_dependencies(forbidden_map_create_node ${DEPENDENCIES})
target_link_libraries(forbidden_map_create_node robot_bringup_lib)

add_executable(editor_map_node src/editor_map.cpp)
ament_target_dependencies(editor_map_node ${DEPENDENCIES})
target_link_libraries(editor_map_node robot_bringup_lib)

add_executable(robot_pose_pub_node src/robot_pose_pub.cpp)
ament_target_dependencies(robot_pose_pub_node ${DEPENDENCIES})
################################################################################
# Install
################################################################################

install(TARGETS
  robot_bringup_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS robot_status_manager_node forbidden_map_create_node
editor_map_node robot_pose_pub_node
DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch param script maps
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
