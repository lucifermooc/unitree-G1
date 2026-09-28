#include "aid_costmap_plugin/cliff_layer.hpp"
#include <algorithm>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace aid_costmap_plugin
{

  void CliffLayer::onInitialize()
  {
    auto node = node_.lock();
    if (!node)
    {
      throw std::runtime_error{"Failed to lock node"};
    }
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("sensor_topics", rclcpp::PARAMETER_STRING_ARRAY);

    std::vector<std::string> topics;
    node->get_parameter(name_ + "." + "enabled", enabled_);
    node->get_parameter(name_ + "." + "sensor_topics", topics);

    if (enabled_)
    {
      RCLCPP_INFO(logger_, "CliffLayer enabled");

      for (auto &topic : topics)
      {
        RCLCPP_INFO(logger_, "sensor_topics: %s", topic.c_str());
        auto sub = node->create_subscription<aid_robot_msgs::msg::BoolSensor>(
            topic, 10,
            std::bind(&CliffLayer::sensorCallback, this, std::placeholders::_1));
        subs_.push_back(sub);
      }
    }
    else
    {
      RCLCPP_INFO(logger_, "CliffLayer disabled");
    }

  }
  CliffLayer::~CliffLayer() { 

  }
  void CliffLayer::updateBounds(double origin_x, double origin_y, double origin_yaw,
                                double *min_x, double *min_y, double *max_x, double *max_y)
  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

    if (!enabled_) {
      return;
    }
    useExtraBounds(min_x, min_y, max_x, max_y);
    current_ = true;
    for (auto &s : sensors_)
    {
      if (!s.second)
        continue; // 没有检测到悬崖

      try
      {
        geometry_msgs::msg::TransformStamped tf =
            tf_->lookupTransform(layered_costmap_->getGlobalFrameID(),
                                 s.first, tf2::TimePointZero);
        double wx = tf.transform.translation.x;
        double wy = tf.transform.translation.y;

        // 让 costmap 关注这个区域
        touch(wx, wy, min_x, min_y, max_x, max_y);

        // 保存坐标，稍后在 updateCosts 使用
        active_points_.push_back({wx, wy});
      }
      catch (tf2::TransformException &ex)
      {
        static auto clock = rclcpp::Clock(RCL_ROS_TIME);
        RCLCPP_WARN_THROTTLE(logger_, clock, 2000,
                             "TF transform failed for %s: %s", s.first.c_str(), ex.what());
      }
    }
  }

  void CliffLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                               int, int, int, int)
  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    if (!enabled_) {
      return;
    }

    if (!current_ && was_reset_) {
      was_reset_ = false;
      current_ = true;
    }
    unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    for (auto &p : active_points_)
    {
      unsigned int mx, my;
      if (master_grid.worldToMap(p.first, p.second, mx, my))
        master_grid.setCost(mx, my, cost);
    }
    active_points_.clear();
  }

  void CliffLayer::sensorCallback(const aid_robot_msgs::msg::BoolSensor::SharedPtr msg)
  {
    // 保存每个传感器的状态
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    if (msg->header.frame_id.empty())
    {
      RCLCPP_WARN(logger_, "CliffLayer::sensorCallback frame_id is empty");
      return;
    }

    sensors_[msg->header.frame_id] = msg->triggered;
  }
  void CliffLayer::activate() {

    // if we're stopped we need to re-subscribe to topics

    RCLCPP_INFO(logger_, "SaftyLayer plugin activated");
  }

  void CliffLayer::deactivate()
  {

    RCLCPP_INFO(logger_, "CliffLayer::deactivate()");
  }

  void CliffLayer::reset()
  {
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    sensors_.clear();
    active_points_.clear();
    resetMaps();
    current_ = false;
    was_reset_ = true;
    RCLCPP_INFO(logger_, "CliffLayer::reset()");
  }
} // namespace aid_costmap_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aid_costmap_plugin::CliffLayer, nav2_costmap_2d::Layer)