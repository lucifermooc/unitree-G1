#include "robot_bringup/map_utils.hpp"
namespace robot_bringup {
/**
 * @brief 获取当前地图ID
 *
 * 创建一个客户端节点，用于请求当前地图ID的服务，并将获取到的地图ID返回。
 *
 * @param map_id 引用参数，用于存储获取到的地图ID
 *
 * @return 如果成功获取到地图ID，则返回true；否则返回false
 */
bool GetCurrentMapId(uint32_t &map_id) {
  auto node = rclcpp::Node::make_shared("get_current_map_id_client");
  auto get_current_map_id_client =
      node->create_client<aid_robot_msgs::srv::GetCurrentMap>(
          "get_current_map_id");
  auto request =
      std::make_shared<aid_robot_msgs::srv::GetCurrentMap ::Request>();

  if (!get_current_map_id_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_INFO(rclcpp::get_logger("get_current_map_id"),
                "service get_current_map_id not available");
    return false;
  }

  auto result = get_current_map_id_client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result,
                                         std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto get_res = result.get();
    if (get_res->success == true) {
      map_id = get_res->map_id;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("get_current_map_id"),
                   "Call service get_forbidden_client,return false");
      return false;
    }

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("get_current_map_id"),
                 "Failed to call service get_current_map_id");
    return false;
  }
  return true;
}

/**
 * @brief 获取当前地图路径
 *
 * 从服务中获取当前地图的路径，并将其存储在传入的字符串中。
 *
 * @param map_file_path 用于存储获取到的地图文件路径的字符串引用
 *
 * @return 如果成功获取到地图路径，则返回true；否则返回false
 */
bool GetCurrentMapPath(std::string &map_file_path) {
  auto node = rclcpp::Node::make_shared("get_current_map_id_client");
  auto get_current_map_id_client =
      node->create_client<aid_robot_msgs::srv::GetCurrentMap>(
          "get_current_map_id");
  auto request =
      std::make_shared<aid_robot_msgs::srv::GetCurrentMap ::Request>();

  if (!get_current_map_id_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_INFO(rclcpp::get_logger("get_current_map_id"),
                "service get_current_map_id not available");
    return false;
  }

  auto result = get_current_map_id_client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result,
                                         std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto get_res = result.get();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("get_current_map_id"),
                       "message:" << get_res->success <<" map path:"<<get_res->map_file);
    if (get_res->success == true) {
      map_file_path = get_res->map_file;

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("get_current_map_id"),
                   "Call service get_forbidden_client,return false");
      return false;
    }

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("get_current_map_id"),
                 "Failed to call service get_current_map_id");
    return false;
  }
  return true;
}

/**
 * @brief 获取地图文件路径
 *
 * 根据给定的地图 ID，获取对应的地图文件路径。
 *
 * @param map_id 地图 ID
 * @param map_file_path 存储地图文件路径的字符串引用
 *
 * @return 成功返回 true，失败返回 false
 */
bool GetMapFilePath(int map_id, std::string &map_file_path) {
  auto node = rclcpp::Node::make_shared("get_map_file_path_client");
  auto get_2d_map_client =
      node->create_client<aid_robot_msgs::srv::MapImage>("get_map_image");
  auto request = std::make_shared<aid_robot_msgs::srv::MapImage::Request>();
  request->id = map_id;
  if (false == get_2d_map_client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_INFO(
        rclcpp::get_logger("get_2d_map_client"),
        "Wait finish_trajectory failed,service not available, return ...");
    return false;
  }
  auto result = get_2d_map_client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result,
                                         std::chrono::seconds(5)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("get_2d_map_client"),
                       "message:success");
    auto get_data = result.get();

    // current_map_ = get_data->map;
    map_file_path = get_data->map_file;

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("get_2d_map_client"),
                 "Failed to call service get_current_map_idp");
    return false;
  }
  return true;
}

}  // end namespace robot_bringup
