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
  const double target = std::copysign(rotate_angular_vel_, angle);
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
}

geometry_msgs::msg::TwistStamped NavShimController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // 控制循环断开（本次 FollowPath 被中止 / 新的 FollowPath 开始）时清掉卡住状态
  const rclcpp::Time now = clock_->now();
  if (have_last_call_ && (now - last_call_).seconds() > 2.0 * control_duration_) {
    stalled_ = false;
    stall_armed_ = false;
  }
  last_call_ = now;
  have_last_call_ = true;
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
  primary_->reset();
  near_->reset();
}
#endif

}  // namespace aid_nav_shim_controller

PLUGINLIB_EXPORT_CLASS(aid_nav_shim_controller::NavShimController, nav2_core::Controller)
