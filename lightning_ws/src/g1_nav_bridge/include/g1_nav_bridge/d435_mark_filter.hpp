#pragma once
// D435 "标记"点云过滤的纯算法部分（无 ROS 依赖，便于单测）。
//
// 要剔除的是 2026-09-22 取证到的稳定错深度：远处、射线掠射地面（下倾 18~21°）时，
// 双目把深度砍掉近一半，反投影后成为悬空约 0.4~0.6 m、厚 2 cm 的"薄片"，且每帧都有，
// 所以 voxel_decay / decay_acceleration / voxel_min_points 对它都无效。
//
// 判据（全部满足才删）：
//   1. 离相机的三维距离 > min_range。默认 1.5 = 过滤上线前 realsense_mark.obstacle_range，
//      STVL 的 obstacle_range 也是相对传感器原点的三维距离，所以"过滤 + obstacle_range 2.0"
//      在节点正常运行时，保留的点包含"不过滤 + obstacle_range 1.5"保留的点（节点挂掉则标记端断流）。
//   2. 射线相对水平面的下倾角 < graze_max（按实时 TF 的重力水平 base_link 算，不是固定图像行）。
//   3. 高度在 [min_z, max_z) 内：低于 min_z 本来就不标记；max_z 以上（桌面等悬空真障碍）不碰。
//   4. 悬空：在所在 xy 柱及 8 邻域里，从地面（z < ground_z）开始往上、相邻点竖向间隔 <= max_gap
//      地连续爬升，爬不到该点的高度。真实障碍立在地上，正面点从地面连成一串；
//      错深度薄片与下方地面之间隔着一大段空档。
// 已知局限：1.5~2.0 m 带内、底部没有回波（亮面地板空洞、深色物体下沿、被遮挡）的真实障碍
// 也会被判为悬空而删掉，要等进入 min_range 才标记——不比过滤前（obstacle_range 1.5）差，
// 但这段预警带内的召回没有保证，必须靠实机召回测试确认。
// 注意：不能只看"柱内竖向跨度"——薄片所在的柱里通常还有从更陡射线来的真实地面点。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace g1_nav_bridge {

// ---------------------------------------------------------------------------------------------
// 深度图网格级剔除，移植自 Haier S01 的 Aurora 930 驱动
// （haier-robot-ws-dl/src/aid-ros-driver/src/deptrum-ros-driver-aurora930/src/ros2_device.cc:532-614）。
// 只剔除、不平滑：飞点和逐帧闪烁点直接删掉，不会像 spatial/temporal 平滑那样把错深度"抹"到邻居上。
// 网格按行优先存储，单位 m，0 表示无效。
// 注意：这两级都治不了稳定错深度（空间上连续、每帧都在），那一类只能靠后面的悬空剔除或标记距离。

// 量程外的像素置 0，返回被置 0 的个数。
inline size_t DepthRangeGate(std::vector<float>& g, double min_d, double max_d) {
  size_t n = 0;
  for (float& d : g) {
    if (d == 0.0f) continue;
    if (!std::isfinite(d) || d < min_d || d > max_d) { d = 0.0f; ++n; }
  }
  return n;
}

// 4 邻域一致性：|Δd| <= diff 的有效邻居少于 min_count 个就删（孤立飞点、边缘拖尾）。
// 判定基于输入网格，删除结果不会连锁影响同一轮里其他像素的判定。
inline std::vector<float> NeighborFilter(const std::vector<float>& g, int w, int h, double diff,
                                         int min_count, size_t* removed = nullptr) {
  std::vector<float> out = g;
  min_count = std::max(0, std::min(4, min_count));
  size_t n = 0;
  auto ok = [&](int r, int c, float d) {
    if (r < 0 || r >= h || c < 0 || c >= w) return 0;
    const float nd = g[static_cast<size_t>(r) * w + c];
    return (nd != 0.0f && std::fabs(nd - d) <= diff) ? 1 : 0;
  };
  for (int r = 0; r < h; ++r)
    for (int c = 0; c < w; ++c) {
      const float d = g[static_cast<size_t>(r) * w + c];
      if (d == 0.0f) continue;
      if (ok(r - 1, c, d) + ok(r + 1, c, d) + ok(r, c - 1, d) + ok(r, c + 1, d) < min_count) {
        out[static_cast<size_t>(r) * w + c] = 0.0f;
        ++n;
      }
    }
  if (removed) *removed = n;
  return n ? out : g;
}

// 帧间一致性：同一像素与上一帧相比 |Δd| > diff，或上一帧该像素无深度 → 删（逐帧闪烁）。
// prev 必须是上一帧"帧间剔除之前"的网格（与 Haier 一致），否则一次误删会永久传下去。
// 代价：新出现的物体第一帧会被删，第二帧起才保留 —— 晚一帧。
inline size_t TemporalFilter(std::vector<float>& g, const std::vector<float>& prev, double diff) {
  if (prev.size() != g.size()) return 0;
  size_t n = 0;
  for (size_t i = 0; i < g.size(); ++i) {
    if (g[i] == 0.0f) continue;
    if (prev[i] == 0.0f || std::fabs(g[i] - prev[i]) > diff) { g[i] = 0.0f; ++n; }
  }
  return n;
}

// ---------------------------------------------------------------------------------------------
// 悬空点剔除（2026-09-23，G1 自研；Haier 没有对应物）
struct MarkFilterParams {
  double min_range = 1.5;       // m，离相机三维距离
  double graze_max_deg = 30.0;  // 射线下倾角上限
  double min_z = 0.15;          // 与 realsense_mark.min_obstacle_height 一致
  double max_z = 0.65;          // 更高的悬空物（桌面）保留
  double ground_z = 0.10;       // 低于此视为地面，作为爬升起点
  double max_gap = 0.08;        // 爬升时允许的最大竖向空档
  double cell = 0.05;           // xy 柱尺寸，与 STVL voxel_size 一致
};

struct Vec3 {
  double x, y, z;
};

// 机器人自身区域剔除（2026-09-24）：base_link 水平距离 < radius 且高度在 [z_min, z_max] 的点是自己的手。
// 实测（d435near_0924，41 次事件全部发生在走路时）：G1 走路时手向前摆进 D435 画面，深度一致的小块
// （0.55~0.63 m，8~51 像素）过得了 4 邻域，落在 base_link 前方 0.25~0.43 m、左右 0.08~0.30 m、
// 高 0.57~0.73 m，在 global 里贴着 base_link 标出致命格。只删这个高度带：更低的腿、更高的躯干照样标，
// 正前方闯进 0.5 m 内的人仍能让机器人停下（MID360 正前方 ±50° 是盲区，这里只有 D435）。
struct SelfCropParams {
  double radius = 0.0;  // <=0 不剔除
  double z_min = 0.45;
  double z_max = 0.85;
};

inline bool InSelfCrop(const Vec3& b, const SelfCropParams& p) {
  return p.radius > 0.0 && b.z >= p.z_min && b.z <= p.z_max && b.x * b.x + b.y * b.y < p.radius * p.radius;
}

// pts：已变换到重力水平的 base_link 系；cam：相机原点在同一系下的位置。
// 返回每个点是否保留。
inline std::vector<bool> FilterMarkPoints(const std::vector<Vec3>& pts, const Vec3& cam,
                                          const MarkFilterParams& p) {
  const size_t n = pts.size();
  std::vector<bool> keep(n, true);
  const double tan_graze = std::tan(p.graze_max_deg * M_PI / 180.0);
  const double min_r2 = p.min_range * p.min_range;

  // 无符号拼键：C++17 下对负数左移是未定义行为。
  auto key = [](int64_t ix, int64_t iy) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 32) |
           static_cast<uint32_t>(iy);
  };
  auto cell_of = [&](double v) { return static_cast<int64_t>(std::floor(v / p.cell)); };

  // 先找候选点；没有候选就不必建柱。
  std::vector<size_t> cand;
  for (size_t i = 0; i < n; ++i) {
    const Vec3& q = pts[i];
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) continue;
    if (q.z < p.min_z || q.z >= p.max_z) continue;
    const double dx = q.x - cam.x, dy = q.y - cam.y, dz = cam.z - q.z;
    if (dx * dx + dy * dy + dz * dz <= min_r2) continue;
    if (dz >= std::hypot(dx, dy) * tan_graze) continue;  // 射线够陡，不是掠射
    cand.push_back(i);
  }
  if (cand.empty()) return keep;

  std::unordered_map<uint64_t, std::vector<float>> cols;
  cols.reserve(n / 4);
  for (const Vec3& q : pts) {
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) continue;
    if (q.z >= p.max_z + p.max_gap) continue;
    cols[key(cell_of(q.x), cell_of(q.y))].push_back(static_cast<float>(q.z));
  }

  // 每个候选柱算一次"从地面连续爬升能到的高度"。
  std::unordered_map<uint64_t, double> reach;
  std::vector<float> zs;
  for (size_t i : cand) {
    const int64_t cx = cell_of(pts[i].x), cy = cell_of(pts[i].y);
    const uint64_t k = key(cx, cy);
    auto it = reach.find(k);
    if (it == reach.end()) {
      zs.clear();
      for (int64_t ox = -1; ox <= 1; ++ox)
        for (int64_t oy = -1; oy <= 1; ++oy) {
          auto c = cols.find(key(cx + ox, cy + oy));
          if (c != cols.end()) zs.insert(zs.end(), c->second.begin(), c->second.end());
        }
      std::sort(zs.begin(), zs.end());
      double top = -1e9;
      for (float z : zs) {
        if (top < -1e8) {
          if (z < p.ground_z) top = z;  // 找到地面起点
          continue;
        }
        if (z - top > p.max_gap) break;
        top = std::max<double>(top, z);
      }
      it = reach.emplace(k, top).first;
    }
    if (pts[i].z > it->second + p.max_gap) keep[i] = false;
  }
  return keep;
}

}  // namespace g1_nav_bridge
