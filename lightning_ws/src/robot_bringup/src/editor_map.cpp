#include <fstream>

#include "robot_bringup/map_utils.hpp"
#include "aid_robot_msgs/srv/draw_picture.hpp"
#include "aid_robot_msgs/srv/map_image.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;
namespace robot_bringup {
class MapEditorNode : public rclcpp::Node {
 public:
  /**
   * @brief MapEditorNode 构造函数
   *
   * 初始化 MapEditorNode 节点，并设置节点名称为 "map_editor"。
   * 目前未实现 map_subscriber_ 的创建和订阅逻辑，该部分已被注释掉。
   * 实现了 map_service_ 的创建和服务回调设置，服务名称为 "/map_editor"，
   * 对应的回调函数为 setMapCallback。
   */
  MapEditorNode() : Node("map_editor") {
    // map_subscriber_ =
    // this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    //     "/map", 10, std::bind(&MapEditorNode::mapCallback, this, _1));

    map_service_ = this->create_service<aid_robot_msgs::srv::DrawPicture>(
        "/map_editor", std::bind(&MapEditorNode::setMapCallback, this, _1, _2));
  }

 private:
  /**
   * @brief 判断文件是否存在
   *
   * 检查指定文件是否存在。
   *
   * @param filename 文件名
   *
   * @return 如果文件存在，则返回 true；否则返回 false
   */
  bool fileExists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
  }

  /**
   * @brief 获取二维地图
   *
   * 根据给定的地图ID获取对应的二维地图。
   *
   * @param map_id 地图ID
   *
   * @return 如果成功获取地图，则返回true；否则返回false
   */
  bool Get2DMap(int map_id) {
    if (!GetMapFilePath(map_id, map_dir_)) {
      return false;
    }
    // get_map_image 返回完整的 ".../map.yaml"，这里统一成不带扩展名的前缀
    const std::string yaml_ext = ".yaml";
    if (map_dir_.size() > yaml_ext.size() &&
        map_dir_.compare(map_dir_.size() - yaml_ext.size(), yaml_ext.size(), yaml_ext) == 0) {
      map_dir_.erase(map_dir_.size() - yaml_ext.size());
    }
    auto map_yaml = map_dir_ + ".yaml";
    auto back_map_yaml = map_dir_ + "_back.yaml";
    try {
      auto map_param = nav2_map_server::loadMapYaml(map_yaml);
      nav2_map_server::loadMapFromFile(map_param, current_map_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "load map %s failed: %s", map_yaml.c_str(), e.what());
      return false;
    }
    if (fileExists(back_map_yaml) == false) {
      saveMap(map_dir_ + "_back");
    }

    return true;
  }

  /**
   * @brief 保存地图
   *
   * 将当前地图保存到指定的文件中。
   *
   * @param map_file 地图文件名
   *
   * @return 始终返回 true（当前实现中不检查保存操作是否成功）
   */
  bool saveMap(std::string map_file) {
    nav2_map_server::SaveParameters save_parameters_loc;
    save_parameters_loc.map_file_name = map_file;
    save_parameters_loc.image_format = "png";
    save_parameters_loc.free_thresh = 0.196;
    save_parameters_loc.occupied_thresh = 0.65;
    save_parameters_loc.mode = nav2_map_server::MapMode::Scale;
    bool r = nav2_map_server::saveMapToFile(current_map_, save_parameters_loc);
    return true;
  }
  /**
   * @brief 地图回调函数
   *
   * 处理接收到的地图消息，并更新当前地图数据。
   *
   * @param msg 地图消息指针
   * 指向 nav_msgs::msg::OccupancyGrid 类型的共享指针，包含地图信息。
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // 处理接收到的地图消息
    current_map_ = *msg;
  }

  /**
   * @brief 设置地图回调函数
   *
   * 根据给定的请求参数，对地图进行编辑，并将结果保存到响应中。
   *
   * @param request 地图绘制请求，包含地图ID和要绘制的矩形数组
   * @param response 地图绘制响应，包含操作结果和消息
   */
  void setMapCallback(
      const std::shared_ptr<aid_robot_msgs::srv::DrawPicture::Request> request,
      const std::shared_ptr<aid_robot_msgs::srv::DrawPicture::Response>
          response) {
    if (Get2DMap(request->map_id) == false) {
      RCLCPP_ERROR(get_logger(), "Get2DMap error.");
      response->success = false;
      response->message = "Get occupancy grid map false";
      return;
    }
    // 检查请求是否有效
    if (request->rectangle_array.empty() && request->type != "point") {
      RCLCPP_ERROR(get_logger(), "Received empty point array for editing");
      response->success = false;
      response->message = "Empty point array received for editing";
      return;
    } else {
      RCLCPP_INFO(get_logger(), "Received point array for editing,map id:%d",
                  request->map_id);
    }

    // 遍历请求中的每个点
    for (const auto& rectangle : request->rectangle_array) {
      auto& point = rectangle.center_point;
      // 将坐标转换为地图栅格索引
      int x_center = (int)((point.x - current_map_.info.origin.position.x) /
                           current_map_.info.resolution);
      int y_center = (int)((point.y - current_map_.info.origin.position.y) /
                           current_map_.info.resolution);

      // 检查索引是否有效
      if (x_center < 0 || x_center >= current_map_.info.width || y_center < 0 ||
          y_center >= current_map_.info.height) {
        RCLCPP_WARN(get_logger(), "Point (%f, %f) is out of map boundaries",
                    point.x, point.y);
        continue;
      }

      // 定义清零范围
      int range =
          (int)(rectangle.side_length / current_map_.info.resolution) / 2;
      // 在范围内将数据清零
      for (int y_offset = -range; y_offset <= range; ++y_offset) {
        for (int x_offset = -range; x_offset <= range; ++x_offset) {
          int x = x_center + x_offset;
          int y = y_center + y_offset;

          // 检查索引是否有效
          if (x < 0 || x >= current_map_.info.width || y < 0 ||
              y >= current_map_.info.height) {
            continue;  // 超出地图范围
          }

          // 计算栅格索引
          int index = y * current_map_.info.width + x;

          current_map_.data[index] = rectangle.grayscale;
        }
      }
    }
    std::string save_map_file = map_dir_;
    saveMap(save_map_file);

    response->success = true;
    response->message = "Map updated successfully";
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber_;
  rclcpp::Service<aid_robot_msgs::srv::DrawPicture>::SharedPtr map_service_;
  nav_msgs::msg::OccupancyGrid current_map_;
  std::string map_dir_;
};
}  // namespace robot_bringup
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_bringup::MapEditorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
