#include <gtest/gtest.h>

#include <vector>

#include "g1_nav_bridge/d435_mark_filter.hpp"

using g1_nav_bridge::FilterMarkPoints;
using g1_nav_bridge::MarkFilterParams;
using g1_nav_bridge::Vec3;

namespace {

const Vec3 kCam{0.14, 0.0, 1.24};  // 实测 D435 在 base_link 系的位置

// 地面：x∈[x0,x1]，y∈[y0,y1]，2 cm 网格，z=0
void AddGround(std::vector<Vec3>& v, double x0, double x1, double y0, double y1) {
  for (double x = x0; x <= x1; x += 0.02)
    for (double y = y0; y <= y1; y += 0.02) v.push_back({x, y, 0.0});
}
// 竖直面：x 固定，y∈[y0,y1]，z∈[0,h]，2 cm 网格（箱子正面 / 杆子）
size_t AddFace(std::vector<Vec3>& v, double x, double y0, double y1, double h) {
  const size_t b = v.size();
  for (double y = y0; y <= y1 + 1e-9; y += 0.02)
    for (double z = 0.0; z <= h + 1e-9; z += 0.02) v.push_back({x, y, z});
  return b;
}
size_t CountRemoved(const std::vector<bool>& k, size_t b, size_t e) {
  size_t r = 0;
  for (size_t i = b; i < e; ++i) r += !k[i];
  return r;
}

}  // namespace

TEST(D435MarkFilter, RemovesFloatingSheetOverGround) {
  // 复现 d435_ghost_0922：右前 1.9 m、离地 0.37~0.39 m 的 2 cm 薄片，下方是真实地面
  std::vector<Vec3> v;
  AddGround(v, 1.6, 2.2, -1.2, -0.6);
  const size_t b = v.size();
  for (double x = 1.85; x <= 1.99; x += 0.02)
    for (double y = -1.0; y <= -0.8; y += 0.02) v.push_back({x, y, 0.37 + 0.02 * (x > 1.92)});
  const size_t e = v.size();
  const auto k = FilterMarkPoints(v, kCam, MarkFilterParams{});
  EXPECT_EQ(CountRemoved(k, b, e), e - b);
  EXPECT_EQ(CountRemoved(k, 0, b), 0u);  // 地面点一个不动
}

TEST(D435MarkFilter, KeepsStandingObstacles) {
  std::vector<Vec3> v;
  AddGround(v, 1.4, 2.2, -1.0, 1.0);
  const size_t box30 = AddFace(v, 1.9, -0.2, 0.2, 0.30);    // 0.3 m 箱子正面
  const size_t box45 = AddFace(v, 1.8, 0.5, 0.8, 0.45);     // 0.45 m 箱子正面
  const size_t pole = AddFace(v, 2.0, -0.9, -0.88, 0.60);   // 细杆（2 cm 宽）
  const size_t step = AddFace(v, 1.7, -0.6, -0.3, 0.18);    // 台阶/门槛
  const size_t e = v.size();
  const auto k = FilterMarkPoints(v, kCam, MarkFilterParams{});
  EXPECT_EQ(CountRemoved(k, box30, e), 0u) << "立在地上的真实障碍不能被删";
  (void)box45; (void)pole; (void)step;
}

TEST(D435MarkFilter, KeepsObstacleWhenGroundHasHole) {
  // D435 地面有空洞：障碍正面自己就从 z≈0 开始，不依赖周围地面点
  std::vector<Vec3> v;
  const size_t b = AddFace(v, 1.9, -0.2, 0.2, 0.30);
  const auto k = FilterMarkPoints(v, kCam, MarkFilterParams{});
  EXPECT_EQ(CountRemoved(k, b, v.size()), 0u);
}

TEST(D435MarkFilter, NeverTouchesInsideMinRange) {
  // 离相机三维距离 <= 1.5 m 的点（过滤上线前 obstacle_range 内）即使悬空也保留 → 召回不劣于旧配置
  std::vector<Vec3> v;
  AddGround(v, 0.8, 1.2, -0.2, 0.2);
  const size_t b = v.size();
  for (double y = -0.1; y <= 0.1; y += 0.02) v.push_back({1.0, y, 0.40});
  const auto k = FilterMarkPoints(v, kCam, MarkFilterParams{});
  EXPECT_EQ(CountRemoved(k, b, v.size()), 0u);
}

TEST(D435MarkFilter, KeepsHighOverhangAndSteepRays) {
  std::vector<Vec3> v;
  AddGround(v, 1.6, 2.2, -0.2, 0.2);
  const size_t b = v.size();
  for (double y = -0.1; y <= 0.1; y += 0.02) v.push_back({1.9, y, 0.72});  // 桌面边缘
  const auto k = FilterMarkPoints(v, kCam, MarkFilterParams{});
  EXPECT_EQ(CountRemoved(k, b, v.size()), 0u);

  // 下倾角 >= graze_max 的射线（陡射线不会掠射误匹配）
  MarkFilterParams p;
  p.graze_max_deg = 10.0;  // 1.9 m / 0.37 m 的射线约 24°
  std::vector<Vec3> w;
  AddGround(w, 1.6, 2.2, -1.2, -0.6);
  const size_t b2 = w.size();
  for (double x = 1.85; x <= 1.99; x += 0.02) w.push_back({x, -0.9, 0.37});
  const auto k2 = FilterMarkPoints(w, kCam, p);
  EXPECT_EQ(CountRemoved(k2, b2, w.size()), 0u);
}

TEST(D435MarkFilter, MissingBaseInBandIsKnownLimitation) {
  // 障碍底部无回波（z<0.20 全缺）且地面也是空洞：1.5 m 外会被删（已知局限，靠实机召回测试兜底），
  // 同一障碍进入 1.5 m 内必须全部保留（这是"不比过滤前差"的底线）。
  auto make = [](double x, std::vector<Vec3>& v) {
    const size_t b = v.size();
    for (double y = -0.2; y <= 0.2; y += 0.02)
      for (double z = 0.20; z <= 0.40; z += 0.02) v.push_back({x, y, z});
    return b;
  };
  std::vector<Vec3> far, near;
  const size_t bf = make(1.9, far);
  const size_t bn = make(1.0, near);
  // 大部分被删（下沿几行射线下倾已 >30°，不算掠射，会留下来）——局限确实存在，不是保证召回
  EXPECT_GT(CountRemoved(FilterMarkPoints(far, kCam, MarkFilterParams{}), bf, far.size()),
            (far.size() - bf) / 2);
  EXPECT_EQ(CountRemoved(FilterMarkPoints(near, kCam, MarkFilterParams{}), bn, near.size()), 0u);
}

TEST(D435MarkFilter, IgnoresNonFiniteAndNegativeCells) {
  std::vector<Vec3> v;
  AddGround(v, 1.6, 2.2, -2.2, -1.6);           // 负栅格坐标
  const size_t b = AddFace(v, 1.9, -2.0, -1.8, 0.30);
  const size_t e = v.size();
  v.push_back({NAN, 0.0, 0.3});
  v.push_back({1.9, INFINITY, 0.3});
  const auto k = FilterMarkPoints(v, kCam, MarkFilterParams{});
  EXPECT_EQ(CountRemoved(k, b, e), 0u);
}

// ------------------------------------------------------------- 深度网格级（移植自 Haier Aurora 驱动）
#include "g1_nav_bridge/d435_ground_curvature.hpp"

using g1_nav_bridge::DepthRangeGate;
using g1_nav_bridge::GroundByCurvature;
using g1_nav_bridge::GroundCurvatureParams;
using g1_nav_bridge::NeighborFilter;
using g1_nav_bridge::TemporalFilter;

TEST(D435DepthGrid, RangeGate) {
  std::vector<float> g{0.0f, 0.1f, 1.0f, 3.5f, NAN};
  EXPECT_EQ(DepthRangeGate(g, 0.25, 3.0), 3u);
  EXPECT_EQ(g, (std::vector<float>{0.0f, 0.0f, 1.0f, 0.0f, 0.0f}));
}

TEST(D435DepthGrid, NeighborRemovesFlyingPixelKeepsSurface) {
  const int w = 8, h = 6;
  std::vector<float> g(w * h, 2.0f);  // 平整面
  g[2 * w + 3] = 1.2f;                // 孤立飞点
  for (int c = 0; c < w; ++c) g[5 * w + c] = 0.0f;  // 最后一行空洞
  size_t n = 0;
  const auto out = NeighborFilter(g, w, h, 0.08, 3, &n);
  EXPECT_EQ(out[2 * w + 3], 0.0f);
  EXPECT_EQ(out[2 * w + 4], 2.0f);  // 飞点的邻居仍有 3 个一致邻居
  // 角点只有 2 个邻居，min_count=3 时会被删（Haier 同样行为）；内部像素保留
  EXPECT_EQ(out[0], 0.0f);
  EXPECT_EQ(out[1 * w + 1], 2.0f);
  EXPECT_GT(n, 1u);
}

TEST(D435DepthGrid, NeighborKeepsSmallRealObstacle) {
  // 3x3 像素的真实小物体（1.2 m）嵌在 2.0 m 背景中：内部像素 4 邻居一致，保留
  const int w = 9, h = 9;
  std::vector<float> g(w * h, 2.0f);
  for (int r = 3; r < 6; ++r)
    for (int c = 3; c < 6; ++c) g[r * w + c] = 1.2f;
  const auto out = NeighborFilter(g, w, h, 0.08, 3);
  EXPECT_EQ(out[4 * w + 4], 1.2f);      // 中心
  EXPECT_EQ(out[3 * w + 4], 1.2f);      // 边中点：3 个一致邻居
  EXPECT_EQ(out[3 * w + 3], 0.0f);      // 角：只有 2 个一致邻居 → 删（物体会"瘦一圈"）
}

TEST(D435DepthGrid, TemporalDropsFlickerAndDelaysNewObjectOneFrame) {
  std::vector<float> prev{2.0f, 2.0f, 0.0f, 2.0f};
  std::vector<float> cur{2.05f, 1.5f, 2.0f, 2.0f};
  // 0: 抖动 5 cm 保留；1: 跳变 50 cm 删；2: 上一帧无深度删；3: 不变保留
  EXPECT_EQ(TemporalFilter(cur, prev, 0.10), 2u);
  EXPECT_EQ(cur, (std::vector<float>{2.05f, 0.0f, 0.0f, 2.0f}));
  // 新物体持续出现：第二帧起保留（和"剔除前"的上一帧比）
  std::vector<float> prev2{2.0f, 1.5f, 2.0f, 2.0f};
  std::vector<float> cur2{2.05f, 1.5f, 2.0f, 2.0f};
  EXPECT_EQ(TemporalFilter(cur2, prev2, 0.10), 0u);
  // 尺寸不一致不比较
  std::vector<float> small{1.0f};
  EXPECT_EQ(TemporalFilter(small, prev, 0.10), 0u);
}

TEST(D435GroundCurvature, RemovesFlatGroundKeepsLowBoxSides) {
  std::vector<Vec3> v;
  AddGround(v, 0.8, 2.0, -0.6, 0.6);  // z=0 平地
  const size_t ground_end = v.size();
  // 0.08 m 高的矮箱子（全部在地面带内）：正面 + 侧面
  const size_t b = v.size();
  for (double y = -0.1; y <= 0.1 + 1e-9; y += 0.01)
    for (double z = 0.0; z <= 0.08 + 1e-9; z += 0.01) v.push_back({1.4, y, z});
  for (double x = 1.4; x <= 1.6 + 1e-9; x += 0.01)
    for (double z = 0.0; z <= 0.08 + 1e-9; z += 0.01) { v.push_back({x, -0.1, z}); v.push_back({x, 0.1, z}); }
  const size_t e = v.size();
  const auto gr = GroundByCurvature(v, GroundCurvatureParams{});
  size_t g_removed = 0, box_removed = 0;
  for (size_t i = 0; i < ground_end; ++i) g_removed += gr[i];
  for (size_t i = b; i < e; ++i) box_removed += gr[i];
  EXPECT_GT(g_removed, ground_end * 8 / 10) << "平地大部分应判为地面";
  EXPECT_LT(box_removed, (e - b) / 2) << "矮箱子的竖直面不应被当成地面";
}

TEST(D435GroundCurvature, IgnoresPointsOutsideBand) {
  std::vector<Vec3> v;
  AddGround(v, 1.0, 1.6, -0.3, 0.3);
  const size_t b = v.size();
  for (double x = 1.0; x <= 1.6; x += 0.02)
    for (double y = -0.3; y <= 0.3; y += 0.02) v.push_back({x, y, 0.40});  // 桌面：平但在带外
  const auto gr = GroundByCurvature(v, GroundCurvatureParams{});
  for (size_t i = b; i < v.size(); ++i) ASSERT_FALSE(gr[i]);
}


// 自身区域：base_link 水平 < radius 且在高度带内才删（走路时前摆的手，实测 x 0.25~0.43 |y| 0.08~0.30 z 0.57~0.73）
TEST(D435SelfCrop, DropsHandsKeepsLegsTorsoAndFarPoints) {
  g1_nav_bridge::SelfCropParams p;
  p.radius = 0.5; p.z_min = 0.45; p.z_max = 0.85;
  using g1_nav_bridge::InSelfCrop;
  EXPECT_TRUE(InSelfCrop({0.30, 0.20, 0.65}, p));    // 左手
  EXPECT_TRUE(InSelfCrop({0.40, -0.15, 0.70}, p));   // 右手
  EXPECT_FALSE(InSelfCrop({0.30, 0.0, 0.30}, p));    // 正前方 0.3 m 的腿：低于高度带，保留
  EXPECT_FALSE(InSelfCrop({0.30, 0.0, 1.10}, p));    // 躯干：高于高度带，保留
  EXPECT_FALSE(InSelfCrop({0.45, 0.30, 0.65}, p));   // 水平 0.54 m：半径外，保留
  g1_nav_bridge::SelfCropParams off;                 // 默认 radius 0：不删
  EXPECT_FALSE(InSelfCrop({0.30, 0.20, 0.65}, off));
}
