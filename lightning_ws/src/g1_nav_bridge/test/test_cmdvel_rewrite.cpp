// cmdvel_to_sport 的最小有效速度改写（cmdvel_rewrite.hpp）。
// 守住：默认参数下行为与改前一致（带 Thor 背包的 G1），walk_min_wz 只影响走路时的转向，原地转仍抬到 min_wz。
#include <array>

#include <gtest/gtest.h>

#include "g1_nav_bridge/cmdvel_rewrite.hpp"

namespace
{

using g1_nav_bridge::ApplyMinimumVelocity;
using g1_nav_bridge::VelocityRewrite;
using V = std::array<float, 3>;

VelocityRewrite RepoDefault()
{
  VelocityRewrite p;   // nav_bridge.yaml：min_vx 0.3 / min_wz 0.6 / turn_min_vx 0 / vx_stop 0.05 / wz_stop 0.15
  p.turn_min_vx = 0.0;
  p.walk_min_wz = p.min_wz;
  return p;
}

VelocityRewrite Orin()
{
  VelocityRewrite p = RepoDefault();   // nav_bridge.local.yaml
  p.min_wz = 0.32;
  p.walk_min_wz = 0.0;
  return p;
}

void ExpectVel(const V & got, float vx, float vy, float wz)
{
  EXPECT_FLOAT_EQ(got[0], vx);
  EXPECT_FLOAT_EQ(got[1], vy);
  EXPECT_FLOAT_EQ(got[2], wz);
}

TEST(CmdVelRewrite, DefaultKeepsOldBehaviour)
{
  const auto p = RepoDefault();
  ExpectVel(ApplyMinimumVelocity(V{0.40F, 0.0F, 0.16F}, p), 0.40F, 0.0F, 0.6F);    // 走路转弯抬到 min_wz
  ExpectVel(ApplyMinimumVelocity(V{0.40F, 0.0F, -0.10F}, p), 0.40F, 0.0F, -0.10F); // 小修正原样
  ExpectVel(ApplyMinimumVelocity(V{0.0F, 0.0F, 0.30F}, p), 0.0F, 0.0F, 0.6F);      // 原地转
  ExpectVel(ApplyMinimumVelocity(V{0.10F, 0.0F, 0.0F}, p), 0.3F, 0.0F, 0.0F);      // 直行抬到 min_vx
  ExpectVel(ApplyMinimumVelocity(V{0.02F, 0.0F, 0.05F}, p), 0.0F, 0.0F, 0.0F);     // 都低于 stop：停
}

TEST(CmdVelRewrite, OrinWalkingPassesWzThrough)
{
  const auto p = Orin();
  ExpectVel(ApplyMinimumVelocity(V{0.40F, 0.0F, 0.16F}, p), 0.40F, 0.0F, 0.16F);
  ExpectVel(ApplyMinimumVelocity(V{0.40F, 0.0F, -0.25F}, p), 0.40F, 0.0F, -0.25F);
  ExpectVel(ApplyMinimumVelocity(V{0.40F, 0.0F, 0.05F}, p), 0.40F, 0.0F, 0.05F);
  ExpectVel(ApplyMinimumVelocity(V{0.05F, 0.0F, 0.20F}, p), 0.05F, 0.0F, 0.20F);   // vx = vx_stop 算走路
}

TEST(CmdVelRewrite, OrinInPlaceStillLifted)
{
  const auto p = Orin();
  ExpectVel(ApplyMinimumVelocity(V{0.0F, 0.0F, 0.20F}, p), 0.0F, 0.0F, 0.32F);     // 原地死区 0.29
  ExpectVel(ApplyMinimumVelocity(V{0.0F, 0.0F, -0.15F}, p), 0.0F, 0.0F, -0.32F);
  ExpectVel(ApplyMinimumVelocity(V{0.04F, 0.0F, 0.20F}, p), 0.04F, 0.0F, 0.32F);   // vx < vx_stop 按原地
  ExpectVel(ApplyMinimumVelocity(V{0.0F, 0.0F, 0.90F}, p), 0.0F, 0.0F, 0.90F);
  ExpectVel(ApplyMinimumVelocity(V{0.0F, 0.0F, 0.10F}, p), 0.0F, 0.0F, 0.0F);      // 低于 wz_stop：停
}

TEST(CmdVelRewrite, ReverseUnchanged)
{
  auto p = Orin();
  // 后退（调用前已按 max_reverse_vx 放行）：抬到 min_reverse_vx，只走直线；带转向的后退按原地转（min_wz）
  ExpectVel(ApplyMinimumVelocity(V{-0.10F, 0.0F, 0.0F}, p), -0.25F, 0.0F, 0.0F);
  ExpectVel(ApplyMinimumVelocity(V{-0.30F, 0.0F, 0.05F}, p), -0.30F, 0.0F, 0.0F);
  ExpectVel(ApplyMinimumVelocity(V{-0.30F, 0.0F, 0.20F}, p), -0.30F, 0.0F, 0.32F);
}

}  // namespace
