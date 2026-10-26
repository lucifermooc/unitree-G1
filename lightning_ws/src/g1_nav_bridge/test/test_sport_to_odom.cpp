// 与 test_sport_to_odom.py（Python 版）同样的用例，守住 C++ 移植的换算一致。
#include <cmath>

#include <gtest/gtest.h>

#include "g1_nav_bridge/sport_to_odom.hpp"

namespace
{

unitree_go::msg::SportModeState Sample()
{
  unitree_go::msg::SportModeState m;
  m.stamp.sec = 1789027818;
  m.stamp.nanosec = 107790946;
  m.imu_state.quaternion = {-1.0F, 0.0F, 0.0F, 0.0F};
  m.position = {1.0F, 2.0F, 0.77F};
  m.body_height = 0.77F;
  m.velocity = {0.2F, -0.1F, 0.03F};
  m.yaw_speed = 0.15F;
  return m;
}

}  // namespace

TEST(SportToOdom, FieldsAndNegativeQuaternion)
{
  const auto m = Sample();
  nav_msgs::msg::Odometry out;
  ASSERT_EQ(g1_nav_bridge::ConvertSportState(m, "odom", "base_link", out), "");
  EXPECT_EQ(out.header.stamp.sec, m.stamp.sec);
  EXPECT_EQ(out.header.stamp.nanosec, m.stamp.nanosec);
  EXPECT_EQ(out.header.frame_id, "odom");
  EXPECT_EQ(out.child_frame_id, "base_link");
  EXPECT_EQ(out.pose.pose.position.z, 0.0);
  EXPECT_EQ(out.pose.pose.position.x, 1.0);
  EXPECT_NEAR(out.pose.pose.orientation.w, 1.0, 1e-6);
  EXPECT_NEAR(out.twist.twist.linear.x, 0.2, 1e-6);
  EXPECT_NEAR(out.twist.twist.angular.z, 0.15, 1e-6);
}

TEST(SportToOdom, YawOrder)
{
  auto m = Sample();
  const float h = static_cast<float>(std::sqrt(0.5));
  m.imu_state.quaternion = {h, 0.0F, 0.0F, h};
  nav_msgs::msg::Odometry out;
  ASSERT_EQ(g1_nav_bridge::ConvertSportState(m, "odom", "base_link", out), "");
  EXPECT_NEAR(out.pose.pose.orientation.z, std::sqrt(0.5), 1e-6);
  EXPECT_NEAR(out.pose.pose.orientation.w, std::sqrt(0.5), 1e-6);
}

TEST(SportToOdom, Invalid)
{
  nav_msgs::msg::Odometry out;
  auto m = Sample();
  m.error_code = 1;
  EXPECT_NE(g1_nav_bridge::ConvertSportState(m, "odom", "base_link", out), "");
  m = Sample();
  m.imu_state.quaternion = {0.0F, 0.0F, 0.0F, 0.0F};
  EXPECT_NE(g1_nav_bridge::ConvertSportState(m, "odom", "base_link", out), "");
  m = Sample();
  m.velocity[0] = std::nanf("");
  EXPECT_NE(g1_nav_bridge::ConvertSportState(m, "odom", "base_link", out), "");
  m = Sample();
  m.stamp.sec = 0;
  m.stamp.nanosec = 0;
  EXPECT_NE(g1_nav_bridge::ConvertSportState(m, "odom", "base_link", out), "");
}
