cmake_minimum_required(VERSION 3.5)
project(aid_robot_msgs)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
  "msg/StartToEndPoint.msg"
  "msg/FilterCloud.msg"
  "msg/Rectangle.msg"
  "msg/AidPose.msg"
  "msg/AidPoses.msg"
  "msg/AidTaskStatus.msg"
  "msg/BoolSensor.msg"
  "msg/Pose6D.msg"
  "msg/MapMark.msg"
  
)

set(srv_files
  "srv/QueryLaunchStatus.srv"
  "srv/ControlLaunch.srv"
  "srv/GetCurrentMap.srv"
  "srv/SetCurrentMap.srv"
  "srv/MapImage.srv"
  "srv/MapLinkedDataList.srv"
  "srv/MapList.srv"
  "srv/MapOperationAdd.srv"
  "srv/OperationAdd.srv"
  "srv/OperationGet.srv"
  "srv/ForbiddenGet.srv"
  "srv/ForbiddenSet.srv"
  "srv/OperationDelete.srv"
  "srv/OperationUpdate.srv"
  "srv/MapOperation.srv"
  "srv/StatusChange.srv"
  "srv/PatrolControl.srv"
  "srv/DrawPicture.srv"
  "srv/AICmd.srv"
  "srv/GetCurrentForbidden.srv"
  "srv/GetString.srv"
  "srv/SetDockPose.srv"
  "srv/GetDockPose.srv"
  "srv/FindBestVelocity.srv"
  "srv/SetString.srv"
  "srv/PointGet.srv"
  "srv/PathNav.srv"
  "srv/PoseNav.srv"
  "srv/GeneralInterface.srv"
  "srv/SavePose.srv"
  "srv/SaveMap.srv"
  "srv/SetMark.srv"
  "srv/LoadMap.srv"
  
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}

  DEPENDENCIES nav_msgs
  DEPENDENCIES geometry_msgs
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime nav2_msgs nav_msgs geometry_msgs sensor_msgs)

ament_package()
