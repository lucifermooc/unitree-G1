#include <algorithm>
#include <array>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unitree_api/msg/response.hpp>

#include "g1_nav_bridge/cmdvel_rewrite.hpp"
#include "g1_nav_bridge/unitree/g1_loco_client.hpp"

namespace g1_nav_bridge {

class CmdVelToSport final : public rclcpp::Node {
 public:
  CmdVelToSport() : Node("g1_cmdvel_to_sport"), client_(this) {
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
    declare_parameter<double>("duration", 0.5);
    duration_ = get_parameter("duration").as_double();
    if (!std::isfinite(duration_) || duration_ <= 0.0) {
      throw std::invalid_argument("duration must be positive and finite");
    }
    // G1 有最小有效速度：直行 vx < 0.3 m/s 走不动；转弯时 |wz| < 0.6 rad/s 转不动。
    // turn_min_vx：转弯时附加的最小前进速度。2026-10-06 实测 G1 能原地转，配置里改为 0（默认值保留旧行为）。
    // 导航输出的非零低速按方向提升到最小有效值；低于 *_stop 视为停止/不转。
    rewrite_.min_vx = declare_parameter<double>("min_vx", 0.3);
    rewrite_.min_wz = declare_parameter<double>("min_wz", 0.6);
    // 走路时（vx >= vx_stop）的最小 |wz|，默认 = min_wz（旧行为）。原地转的死区走路时没有，见 cmdvel_rewrite.hpp。
    rewrite_.walk_min_wz = declare_parameter<double>("walk_min_wz", rewrite_.min_wz);
    rewrite_.turn_min_vx = declare_parameter<double>("turn_min_vx", 0.2);
    rewrite_.vx_stop = declare_parameter<double>("vx_stop", 0.05);
    // |wz| 低于该值视为直行中的小幅航向修正，原样透传（放大会走成蛇形）。
    rewrite_.wz_stop = declare_parameter<double>("wz_stop", 0.15);
    // 后退（只给脱困用的 BT BackUp）。默认 0 = 不允许，vx<0 一律钳为 0（带 Thor 背包的 G1 倒走会摔，CLAUDE.md 2.1）。
    // 无背包的 G1（Orin 30524）2026-10-24 实测：-0.1 不动、-0.2 / -0.3 稳定后退，在本机覆盖文件里放开。
    max_reverse_vx_ = declare_parameter<double>("max_reverse_vx", 0.0);
    rewrite_.min_reverse_vx = declare_parameter<double>("min_reverse_vx", 0.25);
    if (!std::isfinite(rewrite_.walk_min_wz) || rewrite_.walk_min_wz < 0.0) {
      throw std::invalid_argument("walk_min_wz must be finite and >= 0");
    }
    if (!std::isfinite(max_reverse_vx_) || max_reverse_vx_ < 0.0 ||
        !std::isfinite(rewrite_.min_reverse_vx) || rewrite_.min_reverse_vx < 0.0 ||
        (max_reverse_vx_ > 0.0 && rewrite_.min_reverse_vx > max_reverse_vx_)) {
      throw std::invalid_argument(
          "max_reverse_vx/min_reverse_vx must be finite, >= 0, min <= max");
    }

    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        get_parameter("cmd_vel_topic").as_string(), 10,
        std::bind(&CmdVelToSport::OnCmdVel, this, std::placeholders::_1));
    response_subscription_ = create_subscription<unitree_api::msg::Response>(
        "/api/sport/response", rclcpp::QoS(1),
        [this](const unitree_api::msg::Response::SharedPtr response) {
          if (response->header.identity.api_id !=
              ROBOT_API_ID_LOCO_SET_VELOCITY) {
            return;
          }
          if (response->header.status.code == 0) {
            RCLCPP_INFO_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "SetVelocity response: id=%lld code=0",
                static_cast<long long>(response->header.identity.id));
          } else {
            RCLCPP_ERROR(get_logger(),
                         "SetVelocity response: id=%lld code=%d",
                         static_cast<long long>(response->header.identity.id),
                         response->header.status.code);
          }
        });
    worker_ = std::thread(&CmdVelToSport::Run, this);
  }

  ~CmdVelToSport() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    condition_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr message) {
    std::array<float, 3> velocity{
        static_cast<float>(message->linear.x),
        static_cast<float>(message->linear.y),
        static_cast<float>(message->angular.z)};
    if (!std::isfinite(velocity[0]) || !std::isfinite(velocity[1]) ||
        !std::isfinite(velocity[2])) {
      RCLCPP_ERROR(get_logger(), "Invalid cmd_vel; sending zero velocity");
      velocity.fill(0.0F);
    }
    // 【硬性安全约束】带 Thor 背包的 G1 收到负的前进速度会直接摔倒。max_reverse_vx 为 0（默认）时，无论上游
    // Nav2 / 遥控 / 恢复行为发来什么，这里一律钳到 >= 0；这是最后一道闸，不依赖任何上游配置正确。
    // 放开时也只允许直线后退：带转向的后退没实测过，按原地转处理（vx 置 0）。
    if (velocity[0] < 0.0F) {
      if (max_reverse_vx_ <= 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "拒绝负的前进速度 vx=%.3f（G1 倒走会摔倒），已钳为 0",
                             static_cast<double>(velocity[0]));
        velocity[0] = 0.0F;
      } else if (std::fabs(velocity[2]) >= rewrite_.wz_stop) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "后退时不允许转向 vx=%.3f wz=%.3f，vx 已钳为 0",
                             static_cast<double>(velocity[0]),
                             static_cast<double>(velocity[2]));
        velocity[0] = 0.0F;
      } else {
        velocity[0] = std::max(velocity[0], -static_cast<float>(max_reverse_vx_));
      }
    }

    const std::array<float, 3> raw = velocity;
    velocity = ApplyMinimumVelocity(velocity, rewrite_);
    if (velocity != raw) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "cmd_vel lifted: vx %.3f->%.3f wz %.3f->%.3f",
                           raw[0], velocity[0], raw[2], velocity[2]);
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      velocity_ = velocity;
      pending_ = true;
    }
    condition_.notify_one();
  }

  void Run() {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      condition_.wait(lock, [this] { return stopping_ || pending_; });
      if (stopping_) {
        return;
      }

      const auto velocity = velocity_;
      pending_ = false;
      lock.unlock();
      const auto ret = client_.SetVelocity(
          velocity[0], velocity[1], velocity[2],
          static_cast<float>(duration_));
      if (ret != 0) {
        RCLCPP_ERROR(get_logger(), "SetVelocity failed: %d", ret);
      }
      lock.lock();
    }
  }

  ::unitree::robot::g1::LocoClient client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr
      response_subscription_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::array<float, 3> velocity_{};
  double duration_{0.5};
  VelocityRewrite rewrite_;
  double max_reverse_vx_{0.0};
  bool pending_{false};
  bool stopping_{false};
};

}  // namespace g1_nav_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_nav_bridge::CmdVelToSport>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
