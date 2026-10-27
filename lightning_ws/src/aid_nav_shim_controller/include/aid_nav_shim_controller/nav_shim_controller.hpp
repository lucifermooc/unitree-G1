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
 *  - 到点（进 goal_checker 的 xy 容差 - final_rotate_xy_margin）后由外壳原地转到目标朝向（final_rotate）：
 *    锁转向（不在 ±180° 附近来回换向）、接近目标朝向时按 final_rotate_decel 减速到 final_rotate_min_vel；
 *    转向时身体漂出 xy 容差就交回 near 再进点。
 *  - 动作衔接（settle_time > 0）：输出在 走 / 后退 / 左转 / 右转 之间切换，或停下后再动时，先发 0 等 settle_time，
 *    且实测速度（controller_server 给的 odom）降到 settle_max_linear / settle_max_angular 以下（最多等 settle_max_time）。
 *    G1 急停后重心前倾、转向时重心偏一侧，紧接着换动作容易摔（2026-10-26 实机：停步 0.1 s 后就开始原地转，身体还在 0.27 m/s）。
 *    同一动作被短暂打断（MPPI 单次失败 / FollowPath 中止重发）而身体还在按它动（> settle_max_*）时直接接上，不拦。
 *    settle_mid_path=false（默认）时只管到点和起步：这个动作和上一个动作都来自 MPPI 跟线（路途中间：弯道、绕障、
 *    MPPI 失败重发）时不等。实机 10-27 上午全程等（0.6 s）在展厅 S 弯把 MPPI 的走 / 转犹豫和单次失败都变成
 *    "刹停—站—起步"，噔噔噔差点摔；10-26 晚不等时同一 S 弯一路走过去。10-26 要解决的"到点急停 0.1 s 就转、
 *    转完 0.5 s 就起步"只在到点 / 起步。
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
  // 发给 G1 的动作类别（阈值同速度桥：|vx| >= motion_vx_threshold 算走，否则 |wz| >= motion_wz_threshold 算原地转）
  enum class Motion { STOP, WALK, REVERSE, SPIN_LEFT, SPIN_RIGHT };

  geometry_msgs::msg::TwistStamped computeCommand(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker);
  // 到点后原地转到目标朝向；没进 xy 容差时返回 false
  bool finalRotate(
    const geometry_msgs::msg::PoseStamped & robot, double dist, nav2_core::GoalChecker * goal_checker,
    geometry_msgs::msg::TwistStamped & cmd);
  Motion classify(const geometry_msgs::msg::Twist & t) const;
  // 动作衔接：换动作 / 停后再动时先停稳
  void settle(geometry_msgs::msg::TwistStamped & cmd, const geometry_msgs::msg::Twist & velocity);

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
  bool final_rotate_active_{false};   // 已进 xy 容差，只转航向直到换目标
  int final_rotate_dir_{0};
  int rotate_dir_{0};                 // 起步原地转锁定的方向（+1 左 / -1 右，0 = 未定）
  double last_wz_{0.0};               // 上一周期输出的 wz（final_rotate 的角加速度限幅用）
  Motion last_emitted_{Motion::STOP};   // 上一周期实际输出的动作
  Motion last_moving_{Motion::STOP};    // 最近一次输出的非停动作
  Mode last_moving_mode_{Mode::PRIMARY};   // 发出 last_moving_ 时所在阶段
  Mode emit_mode_{Mode::PRIMARY};          // 本周期输出所在阶段
  rclcpp::Time t_last_moving_;
  bool have_moved_{false};
  bool settling_{false};

  // 参数
  std::string primary_type_, near_type_;
  double near_goal_distance_, near_goal_exit_hysteresis_;
  bool near_goal_fallback_to_primary_;
  bool rotate_to_heading_, rotate_only_on_new_goal_;
  double rotate_start_angle_, rotate_stop_angle_, rotate_angular_vel_, max_angular_accel_;
  double forward_sampling_distance_, goal_change_tolerance_, transform_tolerance_;
  double control_duration_;
  double stall_timeout_, stall_min_progress_, stall_detour_reset_;
  bool final_rotate_;
  double final_rotate_min_vel_, final_rotate_decel_, final_rotate_xy_margin_;
  double settle_time_, settle_max_time_, settle_max_linear_, settle_max_angular_;
  bool settle_mid_path_;
  double motion_vx_threshold_, motion_wz_threshold_;
};

}  // namespace aid_nav_shim_controller

#endif  // AID_NAV_SHIM_CONTROLLER__NAV_SHIM_CONTROLLER_HPP_
