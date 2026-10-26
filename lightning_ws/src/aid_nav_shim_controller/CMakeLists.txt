cmake_minimum_required(VERSION 3.8)
project(aid_nav_shim_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

# humble 的 nav2_core::Controller 没有 cancel() / reset()（jazzy 有），按发行版开关
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  add_compile_definitions(AID_NAV2_HUMBLE)
endif()

set(dep_pkgs
  rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_util nav2_costmap_2d
  geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs angles)

add_library(${PROJECT_NAME} SHARED src/nav_shim_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${dep_pkgs})

pluginlib_export_plugin_description_file(nav2_core shim_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dep_pkgs})
ament_package()
