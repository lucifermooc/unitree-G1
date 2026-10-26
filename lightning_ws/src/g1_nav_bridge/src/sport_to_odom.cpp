// 只读的平面里程计适配：/odommodestate -> /odom，不发 TF、不发控制。换算见 include/g1_nav_bridge/sport_to_odom.hpp。
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

#include "g1_nav_bridge/sport_to_odom.hpp"

namespace g1_nav_bridge
{

class SportToOdom final : public rclcpp::Node
{
public:
  SportToOdom()
  : Node("g1_sport_to_odom")
  {
    const auto input = declare_parameter<std::string>("input_topic", "/odommodestate");
    const auto output = declare_parameter<std::string>("output_topic", "/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    timeout_ = declare_parameter<double>("stale_timeout", 0.5);
    const std::vector<double> diag_default{0.1, 0.1, 1e6, 1e6, 1e6, 0.1};
    const auto pose_cov = declare_parameter<std::vector<double>>("pose_covariance_diagonal", diag_default);
    const auto twist_cov = declare_parameter<std::vector<double>>("twist_covariance_diagonal", diag_default);

    if (odom_frame_.empty() || base_frame_.empty() || odom_frame_ == base_frame_) {
      throw std::invalid_argument("Distinct nonempty odom/base frame IDs required");
    }
    for (const auto * diag : {&pose_cov, &twist_cov}) {
      if (diag->size() != 6) {
        throw std::invalid_argument("Covariance diagonal must have six finite nonnegative values");
      }
      for (double v : *diag) {
        if (!std::isfinite(v) || v < 0) {
          throw std::invalid_argument("Covariance diagonal must have six finite nonnegative values");
        }
      }
    }
    if (!std::isfinite(timeout_) || timeout_ <= 0) {
      throw std::invalid_argument("stale_timeout must be positive");
    }
    for (int i = 0; i < 6; ++i) {
      pose_cov_[i] = pose_cov[i];
      twist_cov_[i] = twist_cov[i];
    }

    const auto qos = rclcpp::QoS(10).reliable().durability_volatile();
    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output, qos);
    subscription_ = create_subscription<unitree_go::msg::SportModeState>(
      input, qos, [this](const unitree_go::msg::SportModeState::SharedPtr msg) {OnState(*msg);});
    timer_ = create_wall_timer(
      std::chrono::duration<double>(timeout_), [this]() {CheckStale();});
  }

private:
  void OnState(const unitree_go::msg::SportModeState & state)
  {
    nav_msgs::msg::Odometry out;
    std::string error = ConvertSportState(state, odom_frame_, base_frame_, out);
    const int64_t stamp = int64_t{out.header.stamp.sec} * 1000000000 + out.header.stamp.nanosec;
    if (error.empty() && stamp <= last_stamp_) {
      error = "Nonincreasing source timestamp; restart adapter after robot clock reset";
    }
    if (!error.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.c_str());
      return;
    }
    for (int i = 0; i < 6; ++i) {
      out.pose.covariance[i * 7] = pose_cov_[i];
      out.twist.covariance[i * 7] = twist_cov_[i];
    }
    last_stamp_ = stamp;
    last_receive_ = now();
    received_ = true;
    publisher_->publish(out);
  }

  void CheckStale()
  {
    if (!received_ || (now() - last_receive_).seconds() > timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "No fresh Unitree odometry; no replacement/zero data published");
    }
  }

  std::string odom_frame_;
  std::string base_frame_;
  double timeout_{0.5};
  std::array<double, 6> pose_cov_{};
  std::array<double, 6> twist_cov_{};
  int64_t last_stamp_{-1};
  bool received_{false};
  rclcpp::Time last_receive_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace g1_nav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_nav_bridge::SportToOdom>());
  rclcpp::shutdown();
  return 0;
}
