
#ifndef CUBEBOT3_BRINGUP__MAP_UTILS_HPP_
#define CUBEBOT3_BRINGUP__MAP_UTILS_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rclcpp/rclcpp.hpp"

#include "aid_robot_msgs/srv/map_image.hpp"
#include "aid_robot_msgs/srv/get_current_map.hpp"
#include "glog/logging.h"

namespace robot_bringup
{
    bool GetCurrentMapId(uint32_t &map_id);
    bool GetMapFilePath(int map_id, std::string &map_file_path);
    bool GetCurrentMapPath(std::string &map_file_path);
}  // end namespace robot_bringup

#endif  // CUBEBOT3_BRINGUP__MAP_UTILS_HPP_
