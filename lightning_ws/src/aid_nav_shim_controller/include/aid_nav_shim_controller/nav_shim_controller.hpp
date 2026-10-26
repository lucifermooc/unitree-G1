#ifndef AID_NAV_SHIM_CONTROLLER__NAV_SHIM_CONTROLLER_HPP_
#define AID_NAV_SHIM_CONTROLLER__NAV_SHIM_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace aid_nav_shim_controller
{

/**
 * 控制器外壳（参考轮式机器人的 aid_pure_pursuit_shim_controller）：
 *  - 离目标 >= near_goal_distance：转给 primary（MPPI），跟线的同时按 costmap 局部避障；
 *  - 离目标 < near_goal_distance：转给 near（RPP），按 RPP 原地转到点（G1 已实测的到点方案，CLAUDE.md 3.4）；
 *    进入后锁定，离目标超过 near_goal_distance + near_goal_exit_hysteresis 才退回（防来回切）；
 *    near 报错（RPP collision ahead 等）时本周期退回 primary（near_goal_fallback_to_primary）；
 *  - 新目标起步时若机器人朝向与前方路径方向差 > rotate_start_angle，先原地转到 < rotate_stop_angle 再交给 primary
 *    （MPPI 大角度调头会走圆弧，G1 原地转更省地方）。rotate_only_on_new_goal=false 时行驶中也检查。
 *  - primary 阶段沿路径剩余长度 stall_timeout 内没缩短 stall_min_progress 时抛异常（卡住检测），交给行为树恢复。
 * 参数：外壳和 primary 都在 <name>.* 下（primary 用同一个 name 配置，和轮式一致），near 在 <name>.near.* 下。
 */
class NavShimController : public nav2_core::Controller
{
public:
  NavShimController();
  ~NavShimController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
#ifndef AID_NAV2_HUMBLE
  bool cancel() override;
  void reset() override;
#endif

protected:
  enum class Mode { PRIMARY, ROTATE, NEAR };

  // 机器人位姿转到路径坐标系；失败返回 false
  bool robotInPathFrame(const geometry_msgs::msg::PoseStamped & pose, geometry_msgs::msg::PoseStamped & out);
  size_t nearestIndex(const geometry_msgs::msg::PoseStamped & robot) const;
  // 从最近点到路径终点的长度（路径坐标系）
  double remainingLength(const geometry_msgs::msg::PoseStamped & robot) const;
  // 沿路径没有进展超过 stall_timeout 时抛异常
  void checkStall(const geometry_msgs::msg::PoseStamped & robot);
  // 机器人朝向与"最近点往前 forward_sampling_distance 处"路径方向的夹角（路径坐标系）
  double headingErrorToPath(const geometry_msgs::msg::PoseStamped & robot);
  geometry_msgs::msg::TwistStamped rotateCommand(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, double angle);
  void switchMode(Mode m, const char * why);
  static const char * modeName(Mode m);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("NavShimController")};
  rclcpp::Clock::SharedPtr clock_;

  pluginlib::ClassLoader<nav2_core::Controller> loader_;
  nav2_core::Controller::Ptr primary_;
  nav2_core::Controller::Ptr near_;

  nav_msgs::msg::Path path_;
  Mode mode_{Mode::PRIMARY};
  bool have_goal_{false};
  bool align_pending_{false};   // 新目标后还没做过起步对正
  bool stall_armed_{false};
  double stall_ref_{0.0};
  rclcpp::Time stall_t0_;
  bool stalled_{false};          // 已判定卡住，持续报失败直到控制循环断开
  bool have_last_call_{false};
  rclcpp::Time last_call_;
  double goal_x_{0.0}, goal_y_{0.0};

  // 参数
  std::string primary_type_, near_type_;
  double near_goal_distance_, near_goal_exit_hysteresis_;
  bool near_goal_fallback_to_primary_;
  bool rotate_to_heading_, rotate_only_on_new_goal_;
  double rotate_start_angle_, rotate_stop_angle_, rotate_angular_vel_, max_angular_accel_;
  double forward_sampling_distance_, goal_change_tolerance_, transform_tolerance_;
  double control_duration_;
  double stall_timeout_, stall_min_progress_, stall_detour_reset_;
};

}  // namespace aid_nav_shim_controller

#endif  // AID_NAV_SHIM_CONTROLLER__NAV_SHIM_CONTROLLER_HPP_
