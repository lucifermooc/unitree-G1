cmake_minimum_required(VERSION 3.8)
project(unitree_go)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# g1_nav_bridge 只使用 /odommodestate，保留该话题完整且原名的消息依赖链。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimeSpec.msg"
  "msg/IMUState.msg"
  "msg/SportModeState.msg"
)

install(FILES LICENSE DESTINATION share/${PROJECT_NAME}/licenses)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
