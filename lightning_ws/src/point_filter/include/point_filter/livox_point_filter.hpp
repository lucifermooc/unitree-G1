// MID360 CustomMsg -> PointCloud2 的逐点剔除（纯算法，无 ROS 节点依赖，便于单测）。
//
// 取代原 lightning-lm/scripts/livox_custom_to_pointcloud2.py（Python 逐点 struct.pack，10 Hz × 2 万点吃 CPU，
// 且不做任何剔除、丢掉了 tag / offset_time）。
//
// 剔除顺序（统计记在第一个命中的条件上）：
//   1. 抽点：每 stride 个点留 1 个（lightning fasterlio.point_filter_num）。
//   2. tag：MID360 每点 8 bit，按 Livox Mid-360 通信协议，每 2 bit 一组、值 = 置信度
//      （0 高置信度/正常，1 中，2 低，3 保留）：
//        bit[1:0] 相邻物体间的粘连点（两物体之间拖出的线状噪点）
//        bit[3:2] 雨、雾、灰尘等微小颗粒
//        bit[5:4] 其他
//        bit[7:6] 保留
//      某组的值 > 该组 max_*_level 就剔除；max_*_level = 3 表示该组不过滤。
//      lightning 里对应的 Fast-LIO tag 过滤（pointcloud_preprocess.cc:53，只留 bit[5:4] ∈ {0,1}）是注释掉的。
//   3. 无效点：坐标非有限值或恰为 (0,0,0)（雷达无回波）。
//   4. 重复点：与上一个原始点坐标完全相同。
//   5. 盲区：离雷达 < blind（lightning fasterlio.blind）。
//   6. 高度：雷达系 z 不在 [z_min, z_max]（lightning roi.height_min/max，雷达系、不是 map 系）。
//
// 与 lightning 的差别（2026-09-24 读源码发现，这里按本意实现）：lightning 的去重/盲区判断写成
// `dx || dy || dz && range > blind`，&& 优先级高于 ||，盲区实际只在 dx、dy 都为 0 时才生效；
// 抽点 >1 时它比较的"上一个点"是没填过的全零点，去重退化成"非零点"判断。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace point_filter
{

struct Params
{
  int stride = 1;
  double blind = 0.1;
  double z_min = -2.0;
  double z_max = 10.0;
  int max_glue_level = 1;
  int max_particle_level = 1;
  int max_other_level = 1;
};

struct Stats
{
  size_t in = 0, stride = 0, glue = 0, particle = 0, other = 0, invalid = 0, dup = 0, blind = 0,
    height = 0, out = 0;
};

// 输出点布局：x y z intensity(=reflectivity) 为 float32，offset_time 为 uint32（ns，相对 header 时刻，
// 去畸变要用），tag / line 为 uint8，补齐到 24 字节。
constexpr uint32_t kPointStep = 24;

inline void SetFields(sensor_msgs::msg::PointCloud2 & c)
{
  using sensor_msgs::msg::PointField;
  auto f = [](const char * name, uint32_t offset, uint8_t type) {
      PointField p;
      p.name = name; p.offset = offset; p.datatype = type; p.count = 1;
      return p;
    };
  c.fields = {f("x", 0, PointField::FLOAT32), f("y", 4, PointField::FLOAT32),
    f("z", 8, PointField::FLOAT32), f("intensity", 12, PointField::FLOAT32),
    f("offset_time", 16, PointField::UINT32), f("tag", 20, PointField::UINT8),
    f("line", 21, PointField::UINT8)};
  c.point_step = kPointStep;
  c.is_bigendian = false;
}

inline sensor_msgs::msg::PointCloud2 Filter(
  const livox_ros_driver2::msg::CustomMsg & in, const Params & p, Stats & st)
{
  sensor_msgs::msg::PointCloud2 out;
  out.header = in.header;
  SetFields(out);
  out.height = 1;
  out.is_dense = true;
  // 以 points.size() 为准，不信 point_num（两者不一致时按实际数组走，不越界）
  const size_t n = in.points.size();
  out.data.resize(n * kPointStep);
  const size_t stride = static_cast<size_t>(std::max(1, p.stride));
  const double b2 = p.blind * p.blind;
  size_t kept = 0;
  for (size_t i = 0; i < n; ++i) {
    const auto & q = in.points[i];
    ++st.in;
    if (i % stride != 0) {++st.stride; continue;}
    const int t = q.tag;
    if ((t & 0x3) > p.max_glue_level) {++st.glue; continue;}
    if (((t >> 2) & 0x3) > p.max_particle_level) {++st.particle; continue;}
    if (((t >> 4) & 0x3) > p.max_other_level) {++st.other; continue;}
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) ||
      (q.x == 0.0f && q.y == 0.0f && q.z == 0.0f))
    {
      ++st.invalid; continue;
    }
    if (i > 0) {
      const auto & r = in.points[i - 1];
      if (q.x == r.x && q.y == r.y && q.z == r.z) {++st.dup; continue;}
    }
    const double x = q.x, y = q.y, z = q.z;
    if (x * x + y * y + z * z < b2) {++st.blind; continue;}
    if (z < p.z_min || z > p.z_max) {++st.height; continue;}
    uint8_t * d = out.data.data() + kept * kPointStep;
    const float intensity = static_cast<float>(q.reflectivity);
    std::memcpy(d + 0, &q.x, 4);
    std::memcpy(d + 4, &q.y, 4);
    std::memcpy(d + 8, &q.z, 4);
    std::memcpy(d + 12, &intensity, 4);
    std::memcpy(d + 16, &q.offset_time, 4);
    d[20] = q.tag;
    d[21] = q.line;
    d[22] = d[23] = 0;
    ++kept;
  }
  out.data.resize(kept * kPointStep);
  out.width = static_cast<uint32_t>(kept);
  out.row_step = out.width * kPointStep;
  st.out += kept;
  return out;
}

}  // namespace point_filter
