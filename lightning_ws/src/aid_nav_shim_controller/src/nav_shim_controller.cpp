#include "aid_nav_shim_controller/nav_shim_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "angles/angles.h"
#ifdef AID_NAV2_HUMBLE
#include "nav2_core/exceptions.hpp"
#else
#include "nav2_core/controller_exceptions.hpp"
#endif
#include "nav2_util/node_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

using nav2_util::declare_parameter_if_not_declared;

namespace aid_nav_shim_controller
{

NavShimController::NavShimController()
: loader_("nav2_core", "nav2_core::Controller")
{
}

void NavShimController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("NavShimController: failed to lock node");
  }
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  auto p = [&](const std::string & k, const rclcpp::ParameterValue & v) {
      declare_parameter_if_not_declared(node, plugin_name_ + "." + k, v);
      return node->get_parameter(plugin_name_ + "." + k);
    };
  primary_type_ = p("primary_controller", rclcpp::ParameterValue(
      std::string("nav2_mppi_controller::MPPIController"))).as_string();
  near_type_ = p("near_goal_controller", rclcpp::ParameterValue(
      std::string("nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"))).as_string();
  near_goal_distance_ = p("near_goal_distance", rclcpp::ParameterValue(0.6)).as_double();
  near_goal_exit_hysteresis_ = p("near_goal_exit_hysteresis", rclcpp::ParameterValue(0.3)).as_double();
  near_goal_fallback_to_primary_ = p("near_goal_fallback_to_primary", rclcpp::ParameterValue(true)).as_bool();
  rotate_to_heading_ = p("rotate_to_heading", rclcpp::ParameterValue(true)).as_bool();
  rotate_only_on_new_goal_ = p("rotate_only_on_new_goal", rclcpp::ParameterValue(true)).as_bool();
  rotate_start_angle_ = p("rotate_start_angle", rclcpp::ParameterValue(0.785)).as_double();
  rotate_stop_angle_ = p("rotate_stop_angle", rclcpp::ParameterValue(0.3)).as_double();
  rotate_angular_vel_ = p("rotate_angular_vel", rclcpp::ParameterValue(0.9)).as_double();
  max_angular_accel_ = p("max_angular_accel", rclcpp::ParameterValue(3.2)).as_double();
  forward_sampling_distance_ = p("forward_sampling_distance", rclcpp::ParameterValue(0.6)).as_double();
  goal_change_tolerance_ = p("goal_change_tolerance", rclcpp::ParameterValue(0.05)).as_double();
  transform_tolerance_ = p("transform_tolerance", rclcpp::ParameterValue(0.1)).as_double();
  stall_timeout_ = p("stall_timeout", rclcpp::ParameterValue(10.0)).as_double();
  stall_min_progress_ = p("stall_min_progress", rclcpp::ParameterValue(0.25)).as_double();
  stall_detour_reset_ = p("stall_detour_reset", rclcpp::ParameterValue(0.3)).as_double();
  final_rotate_ = p("final_rotate", rclcpp::ParameterValue(true)).as_bool();
  final_rotate_min_vel_ = p("final_rotate_min_vel", rclcpp::ParameterValue(0.35)).as_double();
  final_rotate_decel_ = p("final_rotate_decel", rclcpp::ParameterValue(1.5)).as_double();
  final_rotate_xy_margin_ = p("final_rotate_xy_margin", rclcpp::ParameterValue(0.05)).as_double();
  settle_time_ = p("settle_time", rclcpp::ParameterValue(0.6)).as_double();
  settle_max_time_ = p("settle_max_time", rclcpp::ParameterValue(1.2)).as_double();
  settle_max_linear_ = p("settle_max_linear", rclcpp::ParameterValue(0.15)).as_double();
  settle_max_angular_ = p("settle_max_angular", rclcpp::ParameterValue(0.3)).as_double();
  settle_mid_path_ = p("settle_mid_path", rclcpp::ParameterValue(false)).as_bool();
  motion_vx_threshold_ = p("motion_vx_threshold", rclcpp::ParameterValue(0.05)).as_double();
  motion_wz_threshold_ = p("motion_wz_threshold", rclcpp::ParameterValue(0.15)).as_double();

  double freq = 10.0;
  node->get_parameter("controller_frequency", freq);
  control_duration_ = 1.0 / std::max(freq, 1.0);

  try {
    primary_ = loader_.createUniqueInstance(primary_type_);
    near_ = loader_.createUniqueInstance(near_type_);
  } catch (const pluginlib::PluginlibException & ex) {
    throw std::runtime_error(std::string("NavShimController: failed to create controller: ") + ex.what());
  }
  // primary 与外壳同名（参数在 <name>.*），near 在 <name>.near.*
  primary_->configure(parent, plugin_name_, tf_, costmap_ros_);
  near_->configure(parent, plugin_name_ + ".near", tf_, costmap_ros_);

  RCLCPP_INFO(
    logger_, "NavShimController '%s': primary %s, near %s within %.2f m (+%.2f exit), "
    "rotate %s (%.2f -> %.2f rad @ %.2f rad/s)",
    plugin_name_.c_str(), primary_type_.c_str(), near_type_.c_str(), near_goal_distance_,
    near_goal_exit_hysteresis_, rotate_to_heading_ ? "on" : "off", rotate_start_angle_, rotate_stop_angle_,
    rotate_angular_vel_);
  RCLCPP_INFO(
    logger_, "NavShimController '%s': stall check %s (%.2f m in %.1f s)", plugin_name_.c_str(),
    stall_timeout_ > 0.0 ? "on" : "off", stall_min_progress_, stall_timeout_);
  RCLCPP_INFO(
    logger_, "NavShimController '%s': final rotate %s (min %.2f rad/s, decel %.2f), settle %.2f s (max %.2f s, "
    "odom < %.2f m/s / %.2f rad/s, %s)", plugin_name_.c_str(), final_rotate_ ? "on" : "off", final_rotate_min_vel_,
    final_rotate_decel_, settle_time_, settle_max_time_, settle_max_linear_, settle_max_angular_,
    settle_mid_path_ ? "everywhere" : "arrival / start only");
}

void NavShimController::cleanup()
{
  primary_->cleanup();
  near_->cleanup();
  primary_.reset();
  near_.reset();
}

void NavShimController::activate()
{
  primary_->activate();
  near_->activate();
}

void NavShimController::deactivate()
{
  primary_->deactivate();
  near_->deactivate();
}

void NavShimController::setPlan(const nav_msgs::msg::Path & path)
{
  path_ = path;
  if (!path.poses.empty()) {
    const auto & g = path.poses.back().pose.position;
    // 定时重规划（同一目标每 2 s 一条新路径）不复位；换了目标才复位
    if (!have_goal_ || std::hypot(g.x - goal_x_, g.y - goal_y_) > goal_change_tolerance_) {
      if (have_goal_ && mode_ != Mode::PRIMARY) {
        switchMode(Mode::PRIMARY, "new goal");
      }
      goal_x_ = g.x;
      goal_y_ = g.y;
      have_goal_ = true;
      align_pending_ = true;
      stall_armed_ = false;
      final_rotate_active_ = false;
      final_rotate_dir_ = 0;
      rotate_dir_ = 0;
    }
  }
  primary_->setPlan(path);
  near_->setPlan(path);
}

bool NavShimController::robotInPathFrame(
  const geometry_msgs::msg::PoseStamped & pose, geometry_msgs::msg::PoseStamped & out)
{
  if (pose.header.frame_id == path_.header.frame_id) {
    out = pose;
    return true;
  }
  return nav2_util::transformPoseInTargetFrame(pose, out, *tf_, path_.header.frame_id, transform_tolerance_);
}

size_t NavShimController::nearestIndex(const geometry_msgs::msg::PoseStamped & robot) const
{
  const auto & poses = path_.poses;
  const double rx = robot.pose.position.x, ry = robot.pose.position.y;
  size_t k = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < poses.size(); ++i) {
    const double d = std::hypot(poses[i].pose.position.x - rx, poses[i].pose.position.y - ry);
    if (d < best) {
      best = d;
      k = i;
    }
  }
  return k;
}

double NavShimController::remainingLength(const geometry_msgs::msg::PoseStamped & robot) const
{
  const auto & poses = path_.poses;
  const size_t k = nearestIndex(robot);
  double s = std::hypot(
    poses[k].pose.position.x - robot.pose.position.x, poses[k].pose.position.y - robot.pose.position.y);
  for (size_t i = k; i + 1 < poses.size(); ++i) {
    s += std::hypot(
      poses[i + 1].pose.position.x - poses[i].pose.position.x,
      poses[i + 1].pose.position.y - poses[i].pose.position.y);
  }
  return s;
}

void NavShimController::checkStall(const geometry_msgs::msg::PoseStamped & robot)
{
  // MPPI 在过不去的地方会原地小幅挪动，刚好满足 progress_checker（15 s 内 0.1 m），既不恢复也不失败（仿真窄道收窄）。
  // 这里看"沿路径到目标的剩余长度"：stall_timeout 内没缩短 stall_min_progress 就抛异常，交给行为树恢复 / 重规划。
  if (stall_timeout_ <= 0.0) {
    return;
  }
  const double rem = remainingLength(robot);
  const rclcpp::Time now = clock_->now();
  // 新开始、取得进展、或重规划绕远（剩余长度变长）时重新计时
  if (!stall_armed_ || rem < stall_ref_ - stall_min_progress_ || rem > stall_ref_ + stall_detour_reset_) {
    stall_armed_ = true;
    stall_ref_ = rem;
    stall_t0_ = now;
    return;
  }
  stall_ref_ = std::max(stall_ref_, rem);   // 只往"更远"方向跟（小幅后退不算进展，也不让基准越抬越低）
  if ((now - stall_t0_).seconds() > stall_timeout_) {
    stall_armed_ = false;
    stalled_ = true;   // 一直失败到 controller_server 真的中止本次 FollowPath（failure_tolerance 内的单次失败会被吞掉）
    const std::string msg = "NavShim: no progress along path (" + std::to_string(stall_min_progress_) +
      " m in " + std::to_string(stall_timeout_) + " s), remaining " + std::to_string(rem) + " m";
    RCLCPP_WARN(logger_, "%s", msg.c_str());
#ifdef AID_NAV2_HUMBLE
    throw nav2_core::PlannerException(msg);
#else
    throw nav2_core::FailedToMakeProgress(msg);
#endif
  }
}

double NavShimController::headingErrorToPath(const geometry_msgs::msg::PoseStamped & robot)
{
  const auto & poses = path_.poses;
  const double rx = robot.pose.position.x, ry = robot.pose.position.y;
  const size_t k = nearestIndex(robot);
  // 从最近点沿路径往前走 forward_sampling_distance，取那一点相对机器人的方位
  size_t j = k;
  double s = 0.0;
  while (j + 1 < poses.size() && s < forward_sampling_distance_) {
    s += std::hypot(
      poses[j + 1].pose.position.x - poses[j].pose.position.x,
      poses[j + 1].pose.position.y - poses[j].pose.position.y);
    ++j;
  }
  const double tx = poses[j].pose.position.x - rx, ty = poses[j].pose.position.y - ry;
  if (std::hypot(tx, ty) < 1e-3) {
    return 0.0;
  }
  return angles::shortest_angular_distance(tf2::getYaw(robot.pose.orientation), std::atan2(ty, tx));
}

geometry_msgs::msg::TwistStamped NavShimController::rotateCommand(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity, double angle)
{
  // 同 RotationShim / RPP：恒定转速 + 角加速度限幅。G1 足迹是圆（robot_radius），原地转不改变占用，不做碰撞检查。
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  // 锁转向（同 final_rotate）：路径方向在身后 ±30° 内时定位 / 步态抖动会让最短方向来回换
  // （实机 10-27 11:27:40 出发点 D 起步原地转，左转中换成右转）
  int dir = angle > 0.0 ? 1 : -1;
  if (rotate_dir_ != 0 && dir != rotate_dir_ && std::fabs(angle) > 2.6) {
    dir = rotate_dir_;
  }
  rotate_dir_ = dir;
  const double target = dir * rotate_angular_vel_;
  const double dw = max_angular_accel_ * control_duration_;
  cmd.twist.angular.z = std::clamp(target, velocity.angular.z - dw, velocity.angular.z + dw);
  return cmd;
}

const char * NavShimController::modeName(Mode m)
{
  switch (m) {
    case Mode::PRIMARY: return "primary";
    case Mode::ROTATE: return "rotate";
    case Mode::NEAR: return "near";
  }
  return "?";
}

void NavShimController::switchMode(Mode m, const char * why)
{
  if (m == mode_) {
    return;
  }
  RCLCPP_INFO(logger_, "NavShim mode %s -> %s (%s)", modeName(mode_), modeName(m), why);
  mode_ = m;
  stall_armed_ = false;
  final_rotate_active_ = false;
  final_rotate_dir_ = 0;
  rotate_dir_ = 0;
}

NavShimController::Motion NavShimController::classify(const geometry_msgs::msg::Twist & t) const
{
  if (t.linear.x >= motion_vx_threshold_) {
    return Motion::WALK;
  }
  if (t.linear.x <= -motion_vx_threshold_) {
    return Motion::REVERSE;
  }
  if (std::fabs(t.angular.z) >= motion_wz_threshold_) {
    return t.angular.z > 0.0 ? Motion::SPIN_LEFT : Motion::SPIN_RIGHT;
  }
  return Motion::STOP;
}

static const char * motionName(int m)
{
  static const char * names[] = {"stop", "walk", "reverse", "spin left", "spin right"};
  return names[m];
}

void NavShimController::settle(geometry_msgs::msg::TwistStamped & cmd, const geometry_msgs::msg::Twist & velocity)
{
  const rclcpp::Time now = clock_->now();
  const Motion c = classify(cmd.twist);
  // 同一动作被短暂打断（MPPI 单次失败 / FollowPath 中止后马上重发，controller_server 发了几拍 0）而身体还在按这个动作动：
  // 直接接上，不算"停后再动"（实机 10-27 11:22 走着 0.43 m/s 时 MPPI 失败一拍，被这里拦成了急停）
  bool resume = false;
  switch (c) {
    case Motion::WALK: resume = velocity.linear.x >= settle_max_linear_; break;
    case Motion::REVERSE: resume = velocity.linear.x <= -settle_max_linear_; break;
    case Motion::SPIN_LEFT: resume = velocity.angular.z >= settle_max_angular_; break;
    case Motion::SPIN_RIGHT: resume = velocity.angular.z <= -settle_max_angular_; break;
    default: break;
  }
  resume = resume && c == last_moving_;
  // 路途中间（这个动作和上一个动作都来自 MPPI 跟线）不等，只管到点（NEAR）和起步（ROTATE 及其后第一步）
  const bool mid_path = emit_mode_ == Mode::PRIMARY && last_moving_mode_ == Mode::PRIMARY;
  if (c != Motion::STOP && settle_time_ > 0.0 && have_moved_ && c != last_emitted_ && !resume &&
    (settle_mid_path_ || !mid_path))
  {
    // 换动作或停后再动：离上次动作至少 settle_time，且实测速度已降下来（最多等 settle_max_time）
    const double since = (now - t_last_moving_).seconds();
    const bool slow = std::fabs(velocity.linear.x) < settle_max_linear_ &&
      std::fabs(velocity.angular.z) < settle_max_angular_;
    if (since < settle_time_ || (!slow && since < settle_max_time_)) {
      if (!settling_) {
        RCLCPP_INFO(
          logger_, "NavShim settle: %s -> %s, hold 0 (%.2f s after last motion, odom vx %.2f wz %.2f)",
          motionName(static_cast<int>(last_moving_)), motionName(static_cast<int>(c)), since, velocity.linear.x,
          velocity.angular.z);
      }
      settling_ = true;
      cmd.twist = geometry_msgs::msg::Twist();
      last_emitted_ = Motion::STOP;
      return;
    }
  }
  settling_ = false;
  last_emitted_ = c;
  if (c != Motion::STOP) {
    last_moving_ = c;
    last_moving_mode_ = emit_mode_;
    t_last_moving_ = now;
    have_moved_ = true;
  }
}

bool NavShimController::finalRotate(
  const geometry_msgs::msg::PoseStamped & robot, double dist, nav2_core::GoalChecker * goal_checker,
  geometry_msgs::msg::TwistStamped & cmd)
{
  geometry_msgs::msg::Pose pose_tol;
  geometry_msgs::msg::Twist vel_tol;
  if (!goal_checker || !goal_checker->getTolerances(pose_tol, vel_tol)) {
    return false;
  }
  const double xy_tol = pose_tol.position.x;
  if (!final_rotate_active_) {
    // 留 final_rotate_xy_margin：刚好压在容差边上时 goal_checker 那一刻可能还没判进（仿真 0.2999 m 进来，转完漂到 0.32 m，
    // 航向对了却永远不判到达，原地等到 progress_checker 失败）
    if (dist > xy_tol - final_rotate_xy_margin_) {
      return false;
    }
    final_rotate_active_ = true;
    final_rotate_dir_ = 0;
    RCLCPP_INFO(logger_, "NavShim: within xy tolerance (%.2f m < %.2f - %.2f), final rotate", dist, xy_tol,
      final_rotate_xy_margin_);
  } else if (dist > xy_tol) {
    // 原地转时身体漂出了 xy 容差（G1 转一次 13~41 cm）：goal_checker 未必锁住了位置（FollowPath 重发会复位它），交回 RPP 再进点
    final_rotate_active_ = false;
    final_rotate_dir_ = 0;
    RCLCPP_INFO(logger_, "NavShim: drifted out of xy tolerance (%.2f m > %.2f) during final rotate, re-approach", dist,
      xy_tol);
    return false;
  }
  const double tol = std::fabs(tf2::getYaw(pose_tol.orientation));
  const double err = angles::shortest_angular_distance(
    tf2::getYaw(robot.pose.orientation), tf2::getYaw(path_.poses.back().pose.orientation));
  cmd.twist = geometry_msgs::msg::Twist();
  if (std::fabs(err) <= tol) {
    return true;   // goal_checker 本周期判到达
  }
  // 锁转向：目标在身后 ±30° 内时（|err| > 150°）定位 / 步态抖动会让最短方向来回换，沿用已选的方向
  int dir = err > 0.0 ? 1 : -1;
  if (final_rotate_dir_ != 0 && dir != final_rotate_dir_ && std::fabs(err) > 2.6) {
    dir = final_rotate_dir_;
  }
  final_rotate_dir_ = dir;
  // 减速：离容差边界还剩 Δ 时 v = sqrt(v_min^2 + 2·a·Δ)，到边界时降到 final_rotate_min_vel（不再从全速一步到 0）
  const double v = std::min(
    rotate_angular_vel_, std::sqrt(
      final_rotate_min_vel_ * final_rotate_min_vel_ + 2.0 * final_rotate_decel_ * (std::fabs(err) - tol)));
  const double target = dir * v;
  const double dw = max_angular_accel_ * control_duration_;
  if (target * last_wz_ > 0.0 && std::fabs(target) < std::fabs(last_wz_)) {
    cmd.twist.angular.z = target;   // 减速按曲线走，不受加速度限幅
  } else {
    cmd.twist.angular.z = std::clamp(target, last_wz_ - dw, last_wz_ + dw);
  }
  return true;
}

geometry_msgs::msg::TwistStamped NavShimController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // 控制循环断开（本次 FollowPath 被中止 / 新的 FollowPath 开始）时清掉卡住状态；断开期间 controller_server 已发 0
  const rclcpp::Time now = clock_->now();
  if (have_last_call_ && (now - last_call_).seconds() > 2.0 * control_duration_) {
    stalled_ = false;
    stall_armed_ = false;
    last_emitted_ = Motion::STOP;
    last_wz_ = 0.0;
  }
  last_call_ = now;
  have_last_call_ = true;
  geometry_msgs::msg::TwistStamped cmd;
  try {
    cmd = computeCommand(pose, velocity, goal_checker);
  } catch (...) {
    // 失败时 controller_server 发 0（或中止），之后再动要先停稳
    last_emitted_ = Motion::STOP;
    last_wz_ = 0.0;
    throw;
  }
  emit_mode_ = mode_;
  settle(cmd, velocity);
  last_wz_ = cmd.twist.angular.z;
  return cmd;
}

geometry_msgs::msg::TwistStamped NavShimController::computeCommand(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (stalled_) {
#ifdef AID_NAV2_HUMBLE
    throw nav2_core::PlannerException("NavShim: stalled, waiting for controller_server to abort");
#else
    throw nav2_core::FailedToMakeProgress("NavShim: stalled, waiting for controller_server to abort");
#endif
  }

  geometry_msgs::msg::PoseStamped robot;
  if (path_.poses.empty() || !robotInPathFrame(pose, robot)) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "NavShim: no plan / TF to path frame, using primary");
    return primary_->computeVelocityCommands(pose, velocity, goal_checker);
  }

  const auto & g = path_.poses.back().pose.position;
  const double dist = std::hypot(g.x - robot.pose.position.x, g.y - robot.pose.position.y);
  if (mode_ != Mode::NEAR && dist < near_goal_distance_) {
    switchMode(Mode::NEAR, "goal within near_goal_distance");
  } else if (mode_ == Mode::NEAR && dist > near_goal_distance_ + near_goal_exit_hysteresis_) {
    switchMode(Mode::PRIMARY, "left near-goal zone");
  }

  if (mode_ == Mode::NEAR) {
    if (final_rotate_) {
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header = pose.header;
      if (finalRotate(robot, dist, goal_checker, cmd)) {
        return cmd;
      }
    }
    try {
      return near_->computeVelocityCommands(pose, velocity, goal_checker);
    } catch (const std::exception & e) {
      if (!near_goal_fallback_to_primary_) {
        throw;
      }
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "NavShim: near controller failed (%s), primary this cycle", e.what());
      return primary_->computeVelocityCommands(pose, velocity, goal_checker);
    }
  }

  if (rotate_to_heading_ && (align_pending_ || mode_ == Mode::ROTATE)) {
    const double err = headingErrorToPath(robot);
    if (mode_ == Mode::PRIMARY && std::fabs(err) > rotate_start_angle_) {
      switchMode(Mode::ROTATE, "heading off path");
    } else if (mode_ == Mode::ROTATE && std::fabs(err) < rotate_stop_angle_) {
      switchMode(Mode::PRIMARY, "aligned with path");
    }
    // 只在新目标起步时对正一次：MPPI 绕障时会主动偏离路径方向，行驶中再检查会和它来回抢（仿真窄道收窄）
    if (rotate_only_on_new_goal_ && mode_ == Mode::PRIMARY) {
      align_pending_ = false;
    }
    if (mode_ == Mode::ROTATE) {
      return rotateCommand(pose, velocity, err);
    }
  }
  checkStall(robot);
  return primary_->computeVelocityCommands(pose, velocity, goal_checker);
}

void NavShimController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  primary_->setSpeedLimit(speed_limit, percentage);
  near_->setSpeedLimit(speed_limit, percentage);
}

#ifndef AID_NAV2_HUMBLE
bool NavShimController::cancel()
{
  // RPP 的 cancel 会减速到停，MPPI 直接返回 true
  return mode_ == Mode::NEAR ? near_->cancel() : primary_->cancel();
}

void NavShimController::reset()
{
  mode_ = Mode::PRIMARY;
  have_goal_ = false;
  align_pending_ = false;
  stall_armed_ = false;
  stalled_ = false;
  final_rotate_active_ = false;
  final_rotate_dir_ = 0;
  rotate_dir_ = 0;
  last_emitted_ = Motion::STOP;   // 衔接计时（t_last_moving_）保留：下一个目标起步也要等停稳
  last_wz_ = 0.0;
  primary_->reset();
  near_->reset();
}
#endif

}  // namespace aid_nav_shim_controller

PLUGINLIB_EXPORT_CLASS(aid_nav_shim_controller::NavShimController, nav2_core::Controller)
