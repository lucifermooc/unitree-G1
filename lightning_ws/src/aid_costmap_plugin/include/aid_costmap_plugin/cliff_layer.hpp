#pragma once

#include <rclcpp/rclcpp.hpp>
#include "aid_robot_msgs/msg/bool_sensor.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <unordered_map>

namespace aid_costmap_plugin
{

class CliffLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  CliffLayer() {
    costmap_ =
        NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }
  virtual ~CliffLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) ;
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  /**
   * @brief Deactivate the layer
   */
  virtual void deactivate();

  /**
   * @brief Activate the layer
   */
  virtual void activate();

  /**
   * @brief Reset this costmap
   */
  virtual void reset();

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() { return true; }
private:
  void sensorCallback(const aid_robot_msgs::msg::BoolSensor::SharedPtr msg);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<rclcpp::Subscription<aid_robot_msgs::msg::BoolSensor>::SharedPtr> subs_;
  std::map<std::string, bool> sensors_;
  std::vector<std::pair<double, double>> active_points_;
  bool was_reset_;
};

}  // namespace aid_costmap_plugin
