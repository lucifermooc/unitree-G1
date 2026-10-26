// cmd_vel -> LocoClient::SetVelocity 的最小有效速度改写（cmdvel_to_sport 用，test_cmdvel_rewrite 守着）。
//
// G1 原地转（vx=0）有硬死区：Orin 30524 |wz| <= 0.29 完全不转（2026-10-24），所以原地转要抬到 min_wz。
// 走路时（vx >= vx_stop）没有这个死区：2026-10-25 motion_test（vx 0.4，直接调 SetVelocity）wz 0.10 -> 实测 0.054~0.066、
// 0.20 -> 0.13、0.30 -> 0.21~0.23，基本线性（实际/指令 0.55~0.78）。走路时再抬到 min_wz 会把 RPP 的小修正放大 2 倍以上、
// 来回打方向，所以走路另用 walk_min_wz（默认 = min_wz 保持旧行为，Orin 覆盖文件设 0 = 原样透传）。
#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace g1_nav_bridge
{

struct VelocityRewrite
{
  double min_vx{0.3};
  double min_wz{0.6};
  double walk_min_wz{0.6};
  double turn_min_vx{0.2};
  double vx_stop{0.05};
  double wz_stop{0.15};
  double min_reverse_vx{0.25};
};

inline float LiftVelocity(float value, double stop, double minimum)
{
  const float magnitude = std::fabs(value);
  if (magnitude < stop) {
    return 0.0F;
  }
  return std::copysign(std::max(magnitude, static_cast<float>(minimum)), value);
}

// velocity = {vx, vy, wz}；负 vx 的放行 / 钳位在调用前已处理。
inline std::array<float, 3> ApplyMinimumVelocity(std::array<float, 3> velocity, const VelocityRewrite & p)
{
  const float wz = velocity[2];
  if (std::fabs(wz) >= p.wz_stop) {
    // 转弯：角速度提到最小有效值（走路用 walk_min_wz，原地 / 后退用 min_wz）；
    // 前进速度至少 turn_min_vx（为 0 时允许原地转；显式后退则保持后退方向）。
    const float vx = velocity[0];
    const double minimum = vx >= static_cast<float>(p.vx_stop) ? p.walk_min_wz : p.min_wz;
    velocity[2] = std::copysign(std::max(std::fabs(wz), static_cast<float>(minimum)), wz);
    velocity[0] = vx <= -static_cast<float>(p.vx_stop) ?
      std::min(vx, -static_cast<float>(p.turn_min_vx)) :
      std::max(vx, static_cast<float>(p.turn_min_vx));
    return velocity;
  }
  velocity[0] = velocity[0] < 0.0F ? LiftVelocity(velocity[0], p.vx_stop, p.min_reverse_vx) :
    LiftVelocity(velocity[0], p.vx_stop, p.min_vx);
  if (velocity[0] < 0.0F) {
    velocity[2] = 0.0F;  // 后退只走直线
  }
  if (velocity[0] == 0.0F) {
    velocity[2] = 0.0F;  // 既不走也不转：停止
  }
  return velocity;
}

}  // namespace g1_nav_bridge
