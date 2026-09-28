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
    // G1 是双足人形，不能原地转向，且有最小有效速度：
    //   直行 vx < 0.3 m/s 走不动；转弯时 |wz| < 0.6 rad/s 转不动，且 vx < 0.2 m/s 转不了弯。
    // 导航输出的非零低速按方向提升到最小有效值；低于 *_stop 视为停止/不转。
    min_vx_ = declare_parameter<double>("min_vx", 0.3);
    min_wz_ = declare_parameter<double>("min_wz", 0.6);
    turn_min_vx_ = declare_parameter<double>("turn_min_vx", 0.2);
    vx_stop_ = declare_parameter<double>("vx_stop", 0.05);
    // |wz| 低于该值视为直行中的小幅航向修正，原样透传（放大会走成蛇形）。
    wz_stop_ = declare_parameter<double>("wz_stop", 0.15);

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
    // 【硬性安全约束】G1 收到负的前进速度会直接摔倒。无论上游 Nav2 / 遥控 / 恢复行为
    // 发来什么，这里一律钳到 >= 0；这是最后一道闸，不依赖任何上游配置正确。
    if (velocity[0] < 0.0F) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "拒绝负的前进速度 vx=%.3f（G1 倒走会摔倒），已钳为 0",
                           static_cast<double>(velocity[0]));
      velocity[0] = 0.0F;
    }

    const std::array<float, 3> raw = velocity;
    velocity = ApplyMinimumVelocity(velocity);
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

  static float Lift(float value, double stop, double minimum) {
    const float magnitude = std::fabs(value);
    if (magnitude < stop) {
      return 0.0F;
    }
    return std::copysign(std::max(magnitude, static_cast<float>(minimum)), value);
  }

  std::array<float, 3> ApplyMinimumVelocity(std::array<float, 3> velocity) const {
    const float wz = velocity[2];
    if (std::fabs(wz) >= wz_stop_) {
      // 转弯：角速度提到最小有效值；不能原地转，必须带前进速度（显式后退则保持后退方向）。
      velocity[2] = std::copysign(std::max(std::fabs(wz), static_cast<float>(min_wz_)), wz);
      const float vx = velocity[0];
      velocity[0] = vx <= -static_cast<float>(vx_stop_)
                        ? std::min(vx, -static_cast<float>(turn_min_vx_))
                        : std::max(vx, static_cast<float>(turn_min_vx_));
      return velocity;
    }
    velocity[0] = Lift(velocity[0], vx_stop_, min_vx_);
    if (velocity[0] == 0.0F) {
      velocity[2] = 0.0F;  // 既不走也不转：停止
    }
    return velocity;
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
  double min_vx_{0.3};
  double min_wz_{0.6};
  double turn_min_vx_{0.2};
  double vx_stop_{0.05};
  double wz_stop_{0.15};
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
