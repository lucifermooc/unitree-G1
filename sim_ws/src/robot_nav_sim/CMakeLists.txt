cmake_minimum_required(VERSION 3.8)
project(robot_nav_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps meshes rviz urdf worlds
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/floor_manager scripts/go_to
  DESTINATION lib/${PROJECT_NAME})

ament_package()
