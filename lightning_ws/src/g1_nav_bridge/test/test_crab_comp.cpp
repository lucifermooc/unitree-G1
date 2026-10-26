// 走路蟹行补偿（crab_comp.hpp）：符号、渐入 / 渐出、速度死区、异常输入。
#include <cmath>

#include <gtest/gtest.h>

#include "g1_nav_bridge/crab_comp.hpp"

namespace
{

using g1_nav_bridge::CrabCompensation;

constexpr double kDeg = M_PI / 180.0;

CrabCompensation Orin()
{
  CrabCompensation c;
  c.crab_rad = 10.0 * kDeg;
  return c;
}

double Settle(CrabCompensation & c, double speed, double secs = 5.0)
{
  double yaw = 0.0;
  for (int i = 0; i < static_cast<int>(secs * 20); ++i) {
    yaw = c.Update(speed, 0.05);
  }
  return yaw;
}

TEST(CrabComp, SignPointsNavXAxisAlongTravel)
{
  // G1 往左偏：base_link_nav 的 x 轴在 base_link 里应是 (cos10°, +sin10°)，即绕 z 转 +10°
  auto c = Orin();
  const double yaw = Settle(c, 0.4);
  EXPECT_NEAR(yaw, 10.0 * kDeg, 1e-6);
  const double qz = std::sin(yaw / 2.0), qw = std::cos(yaw / 2.0);
  const double x_axis_x = 1.0 - 2.0 * qz * qz, x_axis_y = 2.0 * qz * qw;   // R(q) 第一列
  EXPECT_NEAR(x_axis_x, std::cos(10.0 * kDeg), 1e-6);
  EXPECT_NEAR(x_axis_y, std::sin(10.0 * kDeg), 1e-6);
}

TEST(CrabComp, DefaultIsIdentity)
{
  CrabCompensation c;   // crab 0（带 Thor 背包的 G1）
  EXPECT_DOUBLE_EQ(Settle(c, 0.4), 0.0);
}

TEST(CrabComp, RampAndDeadband)
{
  auto c = Orin();
  EXPECT_NEAR(Settle(c, 0.08), 0.0, 1e-9);                 // 原地转漂移速度：不补偿
  EXPECT_NEAR(Settle(c, 0.15), 5.0 * kDeg, 1e-6);          // 0.10~0.20 线性渐入
  EXPECT_NEAR(Settle(c, 0.25), 10.0 * kDeg, 1e-6);         // RPP 弯道 / 进点降速仍补满
}

TEST(CrabComp, SmoothedAndDecaysWhenStopped)
{
  auto c = Orin();
  const double first = c.Update(0.4, 0.05);
  EXPECT_GT(first, 0.0);
  EXPECT_LT(first, 2.0 * kDeg);                            // 一阶平滑，不跳变
  Settle(c, 0.4);
  const double after = Settle(c, 0.0, 1.5);                // 停下 1.5 s（5 τ）后基本归零
  EXPECT_LT(after, 0.1 * kDeg);
}

TEST(CrabComp, BadInputTreatedAsStopped)
{
  auto c = Orin();
  Settle(c, 0.4);
  const double before = c.Update(0.4, 0.05);
  EXPECT_DOUBLE_EQ(c.Update(0.4, 0.0), before);            // dt 0 不更新
  EXPECT_DOUBLE_EQ(c.Update(0.4, -1.0), before);
  EXPECT_LT(c.Update(NAN, 0.05), before);                  // 速度非法按停下
}

}  // namespace
