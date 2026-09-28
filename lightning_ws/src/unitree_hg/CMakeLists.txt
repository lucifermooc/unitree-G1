cmake_minimum_required(VERSION 3.8)
project(unitree_hg)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# g1_nav_bridge 只需要 G1 电池状态，消息字段与 Unitree 官方定义保持一致。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsState.msg"
)

install(FILES LICENSE DESTINATION share/${PROJECT_NAME}/licenses)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
