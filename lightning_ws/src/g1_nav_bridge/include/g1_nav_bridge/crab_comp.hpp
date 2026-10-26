// 走路蟹行补偿：导航专用坐标系 base_link_nav = base_link 绕 z 转 crab × g(v)（crab_comp_frame 节点发布）。
//
// 无 Thor 背包的 G1（Orin 30524）只发 vx 时，实际行进方向比机体航向偏左约 10°（10-25 motion_test wz=0 直走：
// 定位 +11.1°、/odommodestate 位置差分对骨盆 IMU +10.3°；两个导航包 23 段直走全部偏左）。RPP 让机体航向对准前视点，
// 身体却斜着走，纯追踪的平衡点 = 前视 0.6 × tan(10°) ≈ 0.10 m，就是实测"一直偏左约 9 cm"。
// 给 controller / local costmap 一个航向 = 实际行进方向的坐标系，RPP 就按行进方向追线。
// g(v)：走路时才补偿（停下、原地转为 0，否则到点原地转对准的是 base_link_nav 而不是机体）；
// 速度死区 v0 挡住原地转时 0.03~0.08 m/s 的漂移速度，一阶平滑避免航向跳变。
// 速度取 /odom twist 的水平速度大小（G1 自己的速度估计；机体系 / 世界系都成立）。
#pragma once

#include <algorithm>
#include <cmath>

namespace g1_nav_bridge
{

struct CrabCompensation
{
  double crab_rad{0.0};   // 左正
  double v0{0.10};        // 速度低于它不补偿（m/s）：原地转漂移速度 0.03~0.08
  double v_full{0.20};    // 速度高于它补偿满额（m/s）：蟹行按恒定角度处理，RPP 弯道 / 进点降到 0.25 时仍补满
  double tau{0.3};        // 一阶平滑时间常数（s）
  double gain{0.0};       // 当前补偿比例 0~1

  // speed：水平速度大小（m/s），dt：距上次更新的时间（s）。返回 base_link -> base_link_nav 的航向角（rad）。
  double Update(double speed, double dt)
  {
    double target = 0.0;
    if (std::isfinite(speed) && v_full > v0) {
      target = std::clamp((speed - v0) / (v_full - v0), 0.0, 1.0);
    }
    if (std::isfinite(dt) && dt > 0.0) {
      gain += (target - gain) * (1.0 - std::exp(-dt / tau));
    }
    return crab_rad * gain;
  }
};

}  // namespace g1_nav_bridge
