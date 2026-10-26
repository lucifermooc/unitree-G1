// /odommodestate（unitree_go/SportModeState）-> /odom（nav_msgs/Odometry）的平面换算，不发 TF。
//
// 2026-10-24 由 g1_nav_bridge/sport_to_odom.py 移植：Python 版在 Orin 上 ~50 Hz 输入就占 15% CPU，
// 逻辑逐行照搬（Python 版保留作对照，test_sport_to_odom.py 仍测它）。
// 位置取本体开机以来的 XY，航向取 IMU 四元数（宇树顺序 w,x,y,z）；position z / body_height 都不用。
#pragma once

#include <cmath>
#include <cstdint>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>

namespace g1_nav_bridge
{

// 成功返回空串；失败返回原因（与 Python 版 ValueError 文案一致），out 不可用。
inline std::string ConvertSportState(
  const unitree_go::msg::SportModeState & s, const std::string & odom_frame,
  const std::string & base_frame, nav_msgs::msg::Odometry & out)
{
  const double values[] = {
    s.position[0], s.position[1], s.position[2], s.velocity[0], s.velocity[1], s.velocity[2],
    s.imu_state.quaternion[0], s.imu_state.quaternion[1], s.imu_state.quaternion[2],
    s.imu_state.quaternion[3], s.yaw_speed};
  bool finite = true;
  for (double v : values) {finite = finite && std::isfinite(v);}
  if (s.error_code != 0 || !finite) {
    return "Unitree error_code or nonfinite odometry";
  }
  const int64_t sec = s.stamp.sec;
  const int64_t nsec = s.stamp.nanosec;
  if (sec < 0 || nsec < 0 || nsec >= 1000000000 || sec * 1000000000 + nsec <= 0) {
    return "Invalid source timestamp";
  }
  double w = s.imu_state.quaternion[0], x = s.imu_state.quaternion[1];
  double y = s.imu_state.quaternion[2], z = s.imu_state.quaternion[3];
  const double norm = std::sqrt(w * w + x * x + y * y + z * z);
  if (norm < 1e-6) {
    return "Invalid quaternion";
  }
  w /= norm; x /= norm; y /= norm; z /= norm;
  const double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));

  out = nav_msgs::msg::Odometry();
  out.header.stamp.sec = static_cast<int32_t>(sec);
  out.header.stamp.nanosec = static_cast<uint32_t>(nsec);
  out.header.frame_id = odom_frame;
  out.child_frame_id = base_frame;
  out.pose.pose.position.x = s.position[0];
  out.pose.pose.position.y = s.position[1];
  out.pose.pose.orientation.z = std::sin(yaw / 2);
  out.pose.pose.orientation.w = std::cos(yaw / 2);
  out.twist.twist.linear.x = s.velocity[0];
  out.twist.twist.linear.y = s.velocity[1];
  out.twist.twist.angular.z = s.yaw_speed;
  return "";
}

}  // namespace g1_nav_bridge
