cmake_minimum_required(VERSION 3.5)
project(aid_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

# === Environment ===

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(aid_robot_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    nav2_msgs
    pcl_msgs
    pluginlib
    pcl_conversions
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_ros
    tf2_eigen
    sensor_msgs
    aid_robot_msgs)

# === Build ===

add_library(${lib_name} SHARED
            src/observation_buffer.cpp
            src/keepout_layer.cpp
            src/obstacle_layer.cpp
            src/voxel_layer.cpp
            src/cliff_layer.cpp
            src/safty_layer.cpp
            src/range_sensor_layer.cpp
            )
include_directories(include)

target_link_libraries(${lib_name} ${PCL_LIBRARIES})

target_include_directories(${lib_name} PUBLIC
${PCL_INCLUDE_DIRS}
src
)

add_executable(find_best_way_escape 
    src/find_best_way_escape.cpp 
    )
ament_target_dependencies(find_best_way_escape ${dep_pkgs})
target_link_libraries(find_best_way_escape)


# === Installation ===

install(TARGETS ${lib_name} find_best_way_escape
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# === Ament work ===

# pluginlib_export_plugin_description_file() installs costmap_plugins.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()
