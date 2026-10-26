// 发布 TF base_link -> base_link_nav（走路蟹行补偿，见 crab_comp.hpp）。
// 只给 controller / local costmap 用（Orin 覆盖文件里 local_costmap.robot_base_frame: base_link_nav）；
// 定位、点云投影、global costmap 仍用 base_link，不受影响。crab_deg 0（默认）时 base_link_nav = base_link。
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "g1_nav_bridge/crab_comp.hpp"

namespace g1_nav_bridge
{

class CrabCompFrame final : public rclcpp::Node
{
public:
  CrabCompFrame()
  : Node("g1_crab_comp_frame")
  {
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    parent_ = declare_parameter<std::string>("parent_frame", "base_link");
    child_ = declare_parameter<std::string>("child_frame", "base_link_nav");
    const double crab_deg = declare_parameter<double>("crab_deg", 0.0);
    comp_.v0 = declare_parameter<double>("v0", 0.10);
    comp_.v_full = declare_parameter<double>("v_full", 0.20);
    comp_.tau = declare_parameter<double>("tau", 0.3);
    stale_ = declare_parameter<double>("stale_timeout", 0.5);
    const double rate = declare_parameter<double>("rate", 20.0);
    if (!std::isfinite(crab_deg) || std::fabs(crab_deg) > 30.0 || !std::isfinite(comp_.v0) ||
      comp_.v0 < 0.0 || !(comp_.v_full > comp_.v0) || !(comp_.tau > 0.0) || !(rate > 0.0) ||
      !(stale_ > 0.0))
    {
      throw std::invalid_argument(
              "crab_deg in [-30, 30], 0 <= v0 < v_full, tau / rate / stale_timeout > 0");
    }
    comp_.crab_rad = crab_deg * M_PI / 180.0;

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry::SharedPtr m) {
        speed_ = std::hypot(m->twist.twist.linear.x, m->twist.twist.linear.y);
        speed_time_ = now();
      });
    last_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {OnTimer();});
    RCLCPP_INFO(
      get_logger(), "%s -> %s: crab %.1f deg, v0 %.2f, v_full %.2f, tau %.2f",
      parent_.c_str(), child_.c_str(), crab_deg, comp_.v0, comp_.v_full, comp_.tau);
  }

private:
  void OnTimer()
  {
    const auto t = now();
    // 里程计断了按停下处理（补偿渐出），TF 照发，controller 不会因为缺帧报错
    const bool fresh = speed_time_.nanoseconds() > 0 && (t - speed_time_).seconds() < stale_;
    const double yaw = comp_.Update(fresh ? speed_ : 0.0, (t - last_).seconds());
    last_ = t;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = t;
    tf.header.frame_id = parent_;
    tf.child_frame_id = child_;
    tf.transform.rotation.z = std::sin(yaw / 2.0);
    tf.transform.rotation.w = std::cos(yaw / 2.0);
    broadcaster_->sendTransform(tf);
  }

  CrabCompensation comp_;
  std::string parent_;
  std::string child_;
  double stale_{0.5};
  double speed_{0.0};
  rclcpp::Time speed_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace g1_nav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_nav_bridge::CrabCompFrame>());
  rclcpp::shutdown();
  return 0;
}
