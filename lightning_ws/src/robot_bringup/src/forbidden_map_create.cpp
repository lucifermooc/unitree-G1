#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <vector>

#include "aid_robot_msgs/srv/draw_picture.hpp"
#include "aid_robot_msgs/srv/get_current_forbidden.hpp"
#include "aid_robot_msgs/srv/get_current_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_bringup/map_utils.hpp"
using namespace std::placeholders;

namespace robot_bringup {
typedef struct gridindex_ {
  int x;
  int y;

  void SetIndex(int x_, int y_) {
    x = x_;
    y = y_;
  }
} GridIndex;

/**
 * @brief Increments all the grid cells from (x0, y0) to (x1, y1);
 *
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 * @return std::vector<GridIndex>
 */
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1) {
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;

  // 判断是否陡峭
  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    // 如果是陡峭，则交换x和y的坐标
    std::swap(x0, y0);
    std::swap(x1, y1);
  }

  // 确保x0 <= x1
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  // 计算x和y的差值
  int deltaX = x1 - x0;
  int deltaY = abs(y1 - y0);
  int error = 0;

  // 确定y的步长
  int ystep;
  int y = y0;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++) {
    // 根据是否陡峭来确定pointX和pointY的值
    if (steep) {
      pointX = y;
      pointY = x;
    } else {
      pointX = x;
      pointY = y;
    }

    // 更新误差
    error += deltaY;

    // 判断是否需要更新y的值
    if (2 * error >= deltaX) {
      y += ystep;
      error -= deltaX;
    }

    // 如果当前点就是终点，则跳过
    if (pointX == x1 && pointY == y1) continue;

    // 设置tmpIndex的索引值，并添加到gridIndexVector中
    tmpIndex.SetIndex(pointX, pointY);
    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}

class ForbiddenMapCreate : public rclcpp::Node {
 public:
  /**
   * @brief ForbiddenMapCreate 构造函数
   *
   * ForbiddenMapCreate 类的构造函数，用于初始化 ForbiddenMapCreate 对象。
   *
   * 在构造函数中，将节点命名为 "forbidden_map_create"，并初始化成员变量。
   * 创建一个订阅，用于接收地图信息，并绑定回调函数 MapCallback。
   * 创建一个发布者，用于发布 keepout_filter_map 主题的消息。
   * 创建一个服务，用于接收绘制禁止区域的请求，并绑定回调函数 DrawFobbiddenLineCallback。
   */
  ForbiddenMapCreate() : Node("forbidden_map_create") {
    is_get_map_info_ = false;
    map_id_ = UINT32_MAX;
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&ForbiddenMapCreate::MapCallback, this,
                  std::placeholders::_1));
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/keepout_filter_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    draw_forbidden_server_ =
        this->create_service<aid_robot_msgs::srv::DrawPicture>(
            "aid_draw_forbidden_line",
            std::bind(&ForbiddenMapCreate::DrawFobbiddenLineCallback, this, _1,
                      _2));
  }

 private:
  /**
   * @brief 地图回调函数
   *
   * 当接收到地图信息时，该函数将被调用。该函数会处理地图数据，并根据需要发布新的地图信息。
   *
   * @param msg 地图信息的共享指针
   *
   * 地图信息类型为 nav_msgs::msg::OccupancyGrid。
   */
  void MapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    is_get_map_info_ = true;
    static size_t last_get_subscription_count = 0;
    map_sub_ = (*msg);
    //MapCreate();

    size_t map_data_size = map_sub_.data.size();
    for (size_t i = 0; i < map_data_size; i++) {
        map_sub_.data[i] = 0;
    }

    uint32_t map_id = 0;
    GetCurrentMapId(map_id);
    if ((map_id != map_id_) ||
        (publisher_->get_subscription_count() != last_get_subscription_count)) {
      GetForbiddenAndDraw();
      publisher_->publish(map_pub_);
      map_id_ = map_id;
      last_get_subscription_count = publisher_->get_subscription_count();
    }
  }

  /**
   * @brief 获取禁止区域并绘制
   *
   * 从服务获取禁止区域信息，并将其绘制到地图上。
   *
   * @return 如果成功获取并绘制禁止区域，则返回true；否则返回false。
   */
  bool GetForbiddenAndDraw() {
    map_pub_ = map_sub_;
    auto node = rclcpp::Node::make_shared("get_forbidden_client");
    auto get_forbidden_client =
        node->create_client<aid_robot_msgs::srv::GetCurrentForbidden>(
            "get_current_forbidden");
    auto request =
        std::make_shared<aid_robot_msgs::srv::GetCurrentForbidden::Request>();

    if (!get_forbidden_client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_INFO(rclcpp::get_logger("get_forbidden_client"),
                  "service get_forbidden_client not available");
      return false;
    }

    auto result = get_forbidden_client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result,
                                           std::chrono::seconds(5)) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto get_res = result.get();
      RCLCPP_INFO_STREAM(rclcpp::get_logger("get_forbidden_client"),
                         "message:" << get_res->success);
      if (get_res->success == true) {
        size_t line_num = get_res->message.size();
        for (size_t i = 0; i < line_num; i++) {
          DrawLineToMap(get_res->message[i]);
        }
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("get_forbidden_client"),
                     "Call service get_forbidden_client,return false");
        return false;
      }

    } else {
      RCLCPP_ERROR(rclcpp::get_logger("get_forbidden_client"),
                   "Failed to call service get_forbidden_client");
      return false;
    }
    return true;
  }

  /**
   * @brief 绘制禁止线的回调函数
   *
   * 当收到绘制禁止线的请求时，根据请求内容在地图上绘制禁止线，并返回响应结果。
   *
   * @param request 绘制图片的请求对象指针
   * @param response 绘制图片的响应对象指针
   */
  void DrawFobbiddenLineCallback(
      const std::shared_ptr<aid_robot_msgs::srv::DrawPicture::Request> request,
      std::shared_ptr<aid_robot_msgs::srv::DrawPicture::Response> response) {
    if (is_get_map_info_ == false) {
      response->success = false;
      response->message = "No map get.";
      return;
    }
    map_pub_ = map_sub_;
    // size_t map_data_size = map_pub_.data.size();
    // for (size_t i = 0; i < map_data_size; i++) {
    //     map_pub_.data[i] = 0;
    // }
    // map_pub_.data;
    size_t line_num = request->data.size();
    for (size_t i = 0; i < line_num; i++) {
      DrawLineToMap(request->data[i]);
    }
    response->success = true;
    response->message = "success";
    // Publish new message
    publisher_->publish(map_pub_);
  }

  /**
   * @brief 在地图上绘制线段
   *
   * 根据给定的起点和终点，在地图上绘制一条线段。
   *
   * @param line 线段信息结构体，包含起点和终点的坐标
   */
  void DrawLineToMap(aid_robot_msgs::msg::StartToEndPoint line) {
    auto start_index =
        ConvertWorld2GridIndex(map_pub_, line.start.x, line.start.y);
    auto end_index = ConvertWorld2GridIndex(map_pub_, line.end.x, line.end.y);
    auto line_points =
        TraceLine(start_index.x, start_index.y, end_index.x, end_index.y);
    for (size_t i = 0; i < line_points.size(); i++) {
      GridIndex point = line_points[i];
      if (IsValidGridIndex(map_pub_, point) == false) {
        continue;
      }
      int index = GridIndexToLinearIndex(map_pub_, point);
      map_pub_.data[index] = 100;
    }
  }
  /**
   * @brief Convert linear index to world
   *
   * @param map Input map
   * @param linear_index Input index
   * @param x Out x
   * @param y Out y
   */
  void ConvertLinearIndex2World(const nav_msgs::msg::OccupancyGrid &map,
                                int linear_index, double *x, double *y) {
    GridIndex index;
    index.x = linear_index % map.info.width;
    index.y = linear_index / map.info.width;
    *x = static_cast<double>(index.x) * map.info.resolution -
         0.5 * map.info.resolution + map.info.origin.position.x;
    *y = static_cast<double>(index.y) * map.info.resolution -
         0.5 * map.info.resolution + map.info.origin.position.y;
  }
  /**
   * @brief Check whether index is valid
   *
   * @param map
   * @param index
   * @return true
   * @return false
   */
  bool IsValidGridIndex(const nav_msgs::msg::OccupancyGrid &map,
                        GridIndex index) {
    if (index.x >= 0 && (unsigned int)index.x < map.info.width &&
        index.y >= 0 && (unsigned int)index.y < map.info.height)
      return true;

    return false;
  }

  /**
   * @brief Convert from world to raster coordinates
   *
   * @param map
   * @param x
   * @param y
   * @return GridIndex
   */
  GridIndex ConvertWorld2GridIndex(const nav_msgs::msg::OccupancyGrid &map,
                                   double x, double y) {
    GridIndex index;
    index.x = std::ceil((x - map.info.origin.position.x) / map.info.resolution);
    index.y = std::ceil((y - map.info.origin.position.y) / map.info.resolution);
    return index;
  }
  /**
   * @brief Grid index to linear index
   *
   * @param map
   * @param index
   * @return int
   */
  int GridIndexToLinearIndex(const nav_msgs::msg::OccupancyGrid &map,
                             GridIndex index) {
    int linear_index;
    linear_index = index.x + index.y * map.info.width;
    return linear_index;
  }

  /**
   * @brief 创建地图
   *
   * 将订阅的地图赋值给发布的地图，并在地图上绘制一条从左上角到右下角的线，
   * 然后将该线所在的网格的数值设为100，并发布新的地图消息。
   */
  void MapCreate() {
    map_pub_ = map_sub_;
    // map_pub_.data;
    auto line_points =
        TraceLine(0, 0, map_pub_.info.width, map_pub_.info.height);
    for (size_t i = 0; i < line_points.size(); i++) {
      GridIndex point = line_points[i];
      if (IsValidGridIndex(map_pub_, point) == false) {
        continue;
      }
      int index = GridIndexToLinearIndex(map_pub_, point);
      map_pub_.data[index] = 100;
    }

    // Publish new message
    publisher_->publish(map_pub_);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Service<aid_robot_msgs::srv::DrawPicture>::SharedPtr
      draw_forbidden_server_;
  nav_msgs::msg::OccupancyGrid map_sub_;
  nav_msgs::msg::OccupancyGrid map_pub_;
  bool is_get_map_info_;
  uint32_t map_id_;
};
}
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_bringup::ForbiddenMapCreate>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
