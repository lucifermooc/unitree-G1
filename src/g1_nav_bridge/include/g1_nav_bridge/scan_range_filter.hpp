// MID360 点云近距剔除：/lightning/registered_scan -> STVL。
//
// STVL 没有 obstacle_min_range（标记只跳过距离 < 1 cm 的点）。MID360 装在头上，LIO 的 blind 只有 0.1 m，
// 0.1~0.25 m 的机身回波会原样进 registered_scan；以前靠 obstacle_layer 的 obstacle_min_range 0.25 滤掉，
// 迁到 STVL 后必须在这里剔除，否则会在机器人脚下标出致命格（STVL 自清是关的）。
// 输入是雷达系点云（frame=mid360_link），所以到雷达的距离就是点坐标的模长，不需要 TF。
//
// 另按 base_link 水平半径剔除（BaseCrop，需扫描时刻的 TF）：STVL 的 footprint 自清只把当前足迹在 2D 图上
// 抹成空闲，体素还在；足迹内每帧都有的点（离雷达 0.25~0.26 m 的机身/吊带回波、贴身物体）会被标成体素，
// 机器人一走就在它刚站过的位置露出来，global（衰减长）上沿路径留下一串"自身位置的障碍"（2026-09-24 实测）。
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace g1_nav_bridge
{

// 返回 x/y/z 三个 float32 字段的偏移；缺任一字段返回 false
inline bool XyzOffsets(const sensor_msgs::msg::PointCloud2 & c, uint32_t & ox, uint32_t & oy, uint32_t & oz)
{
  int found = 0;
  for (const auto & f : c.fields) {
    if (f.datatype != sensor_msgs::msg::PointField::FLOAT32) {continue;}
    if (f.name == "x") {ox = f.offset; ++found;}
    if (f.name == "y") {oy = f.offset; ++found;}
    if (f.name == "z") {oz = f.offset; ++found;}
  }
  return found == 3;
}

// 雷达系 -> base_link 的刚体变换（R 行主序），只用来算点在 base_link 下的水平距离。
struct BaseCrop
{
  double R[9];
  double t[3];
  double radius;   // base_link 水平距离 < radius 的点剔除
};

// 保留到原点距离 >= min_range、坐标有限、且（给了 crop 时）base_link 水平距离 >= crop->radius 的点，
// 其余字段原样拷贝。输出为无组织点云（height=1），坐标系不变（仍是雷达系）。
inline sensor_msgs::msg::PointCloud2 FilterByRange(
  const sensor_msgs::msg::PointCloud2 & in, double min_range, const BaseCrop * crop = nullptr)
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = in.header;
  out.fields = in.fields;
  out.is_bigendian = in.is_bigendian;
  out.point_step = in.point_step;
  out.height = 1;
  out.is_dense = true;
  uint32_t ox = 0, oy = 0, oz = 0;
  const size_t n = static_cast<size_t>(in.width) * in.height;
  if (!XyzOffsets(in, ox, oy, oz) || in.point_step == 0 || in.data.size() < n * in.point_step) {
    out.width = 0;
    out.row_step = 0;
    return out;
  }
  const double r2 = min_range * min_range;
  const double c2 = crop ? crop->radius * crop->radius : 0.0;
  out.data.resize(n * in.point_step);
  size_t kept = 0;
  for (size_t i = 0; i < n; ++i) {
    // 有组织点云每行可能有 padding：按 row_step 定位
    const size_t row = i / in.width, col = i % in.width;
    const uint8_t * p = in.data.data() + row * in.row_step + col * in.point_step;
    float x, y, z;
    std::memcpy(&x, p + ox, 4);
    std::memcpy(&y, p + oy, 4);
    std::memcpy(&z, p + oz, 4);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {continue;}
    if (static_cast<double>(x) * x + static_cast<double>(y) * y + static_cast<double>(z) * z < r2) {continue;}
    if (crop) {
      const double * R = crop->R;
      const double bx = R[0] * x + R[1] * y + R[2] * z + crop->t[0];
      const double by = R[3] * x + R[4] * y + R[5] * z + crop->t[1];
      if (bx * bx + by * by < c2) {continue;}
    }
    std::memcpy(out.data.data() + kept * in.point_step, p, in.point_step);
    ++kept;
  }
  out.data.resize(kept * in.point_step);
  out.width = static_cast<uint32_t>(kept);
  out.row_step = out.width * out.point_step;
  return out;
}

}  // namespace g1_nav_bridge
