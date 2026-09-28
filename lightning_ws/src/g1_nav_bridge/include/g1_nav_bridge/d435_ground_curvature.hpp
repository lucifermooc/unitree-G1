#pragma once
// 低高度带内按法线曲率去地面，移植自 Haier S01 aid_pointcloud_filter::removeGroundByCurvature
// （haier-robot-ws-dl/src/aid_pointcloud_filter/src/pointcloud_filter_node.cpp:32-120，CPU 分支）。
//
// 与 Haier 的区别：
//   - Haier 用启动时查一次的静态 TF 转到 base_footprint（轮式底盘，相机对地面固定）；
//     G1 头部随步态俯仰，调用方必须按该帧 stamp 的实时 TF 转到重力水平的 base_link 再传进来。
//   - Haier 先对整帧做 2.5 cm 体素再算法线；这里只对带内的点做体素，在体素质心上算法线，
//     再把结果映射回原始点，控制 ARM 上的耗时。
// 只处理 z ∈ [z_min, z_max) 的点：带外的点一律不判为地面（和 Haier 一样）。
// 局限：带内低矮障碍的平整顶面也会被判为地面删掉，靠侧面/边缘的高曲率点保留障碍轮廓；
// 在 realsense_mark.min_obstacle_height 仍为 0.15 时，带内（<0.10）的点本来就不标记，这一级不起作用，
// 它的意义是让 min_obstacle_height 有条件往下调（那是单独一步，单独验证）。
//
// **D435 上默认关闭**（2026-09-23 合成数据扫描，平地 + 高斯噪声 σ，k=30，阈值 0.03）：
//   体素 2.5cm（Haier 值）: σ=1cm 判出地面 33%，σ≥1.5cm ≈0%     —— Aurora 能用，D435 不能
//   体素 5cm             : σ=1cm 100%，σ=2cm 3%；0.08m 箱子误删 0%
//   体素 10cm            : σ=3cm 仍 96~100%，但 0.05~0.08m 箱子被当地面删 30~100%
// 即 σ≈2~3cm 时，5~8cm 的矮障碍与地面起伏在曲率上不可分，这是物理上限，不是调参问题。
// 只有实测（d435_ground_quality.py 的"地面带 z std"）把 D435 地面噪声压到 ~1cm（比如开驱动 spatial 滤波后）
// 才考虑以 5cm 体素打开。

#include <array>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include "g1_nav_bridge/d435_mark_filter.hpp"

namespace g1_nav_bridge {

struct GroundCurvatureParams {
  double z_min = -0.10;
  double z_max = 0.10;
  double voxel = 0.05;      // Haier 用 0.025；D435 噪声大，见上方扫描结果
  int k_search = 30;        // Haier curvature_k_search
  double threshold = 0.03;  // Haier ground_curvature_threshold（上视相机）
};

// 返回每个点是否判为地面。pts 须在重力水平的 base_link 系。
inline std::vector<bool> GroundByCurvature(const std::vector<Vec3>& pts,
                                           const GroundCurvatureParams& p) {
  std::vector<bool> ground(pts.size(), false);
  auto cell = [&](double v) { return static_cast<int64_t>(std::floor(v / p.voxel)); };
  auto key = [](int64_t a, int64_t b, int64_t c) {
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 42) ^
           (static_cast<uint64_t>(static_cast<uint32_t>(b) & 0x1fffff) << 21) ^
           (static_cast<uint64_t>(static_cast<uint32_t>(c)) & 0x1fffff);
  };

  // 体素化：每个体素一个质心
  std::unordered_map<uint64_t, uint32_t> vox_of;
  std::vector<uint32_t> pt_vox(pts.size(), UINT32_MAX);
  std::vector<std::array<double, 4>> acc;  // sx, sy, sz, n
  for (size_t i = 0; i < pts.size(); ++i) {
    const Vec3& q = pts[i];
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z)) continue;
    if (q.z < p.z_min || q.z >= p.z_max) continue;
    const uint64_t k = key(cell(q.x), cell(q.y), cell(q.z));
    auto it = vox_of.find(k);
    if (it == vox_of.end()) {
      it = vox_of.emplace(k, static_cast<uint32_t>(acc.size())).first;
      acc.push_back({0, 0, 0, 0});
    }
    auto& a = acc[it->second];
    a[0] += q.x; a[1] += q.y; a[2] += q.z; a[3] += 1;
    pt_vox[i] = it->second;
  }
  const int k_search = std::max(3, p.k_search);
  if (acc.size() < static_cast<size_t>(k_search)) return ground;  // 点太少，算不出可靠法线

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->reserve(acc.size());
  for (const auto& a : acc)
    cloud->push_back(pcl::PointXYZ(a[0] / a[3], a[1] / a[3], a[2] / a[3]));
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
  ne.setKSearch(k_search);
  pcl::PointCloud<pcl::Normal> normals;
  ne.compute(normals);

  std::vector<bool> vox_ground(acc.size(), false);
  for (size_t v = 0; v < acc.size(); ++v) {
    const float c = normals.points[v].curvature;
    vox_ground[v] = std::isfinite(c) && c < p.threshold;  // 算不出曲率的保留（不判地面）
  }
  for (size_t i = 0; i < pts.size(); ++i)
    if (pt_vox[i] != UINT32_MAX) ground[i] = vox_ground[pt_vox[i]];
  return ground;
}

}  // namespace g1_nav_bridge
