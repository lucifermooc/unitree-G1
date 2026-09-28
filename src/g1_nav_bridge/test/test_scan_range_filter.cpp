#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "g1_nav_bridge/scan_range_filter.hpp"

using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace
{
// x y z intensity（与 LIO 输出同为 float32，point_step 16）
PointCloud2 MakeCloud(const std::vector<std::array<float, 4>> & pts)
{
  PointCloud2 c;
  c.header.frame_id = "mid360_link";
  c.header.stamp.sec = 7;
  const char * names[] = {"x", "y", "z", "intensity"};
  for (uint32_t i = 0; i < 4; ++i) {
    PointField f;
    f.name = names[i]; f.offset = 4 * i; f.datatype = PointField::FLOAT32; f.count = 1;
    c.fields.push_back(f);
  }
  c.point_step = 16;
  c.height = 1;
  c.width = pts.size();
  c.row_step = c.width * c.point_step;
  c.data.resize(c.row_step);
  std::memcpy(c.data.data(), pts.data(), c.data.size());
  return c;
}

std::array<float, 4> At(const PointCloud2 & c, size_t i)
{
  std::array<float, 4> p;
  std::memcpy(p.data(), c.data.data() + i * c.point_step, 16);
  return p;
}
}  // namespace

TEST(ScanRangeFilter, DropsNearAndKeepsFarWithAllFields)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto in = MakeCloud({{0.10f, 0.0f, 0.0f, 1.f},      // 机身回波
                       {0.0f, 0.15f, -0.15f, 2.f},    // 0.21 m
                       {0.0f, 0.0f, 0.0f, 3.f},       // 0 距离无效点
                       {0.30f, 0.0f, 0.0f, 4.f},      // 保留
                       {nan, 1.0f, 1.0f, 5.f},        // 非有限
                       {-2.0f, 1.0f, -1.0f, 6.f}});   // 保留
  auto out = g1_nav_bridge::FilterByRange(in, 0.25);
  ASSERT_EQ(out.width, 2u);
  EXPECT_EQ(out.height, 1u);
  EXPECT_EQ(out.row_step, 2u * 16u);
  EXPECT_EQ(out.data.size(), 2u * 16u);
  EXPECT_EQ(out.header.frame_id, "mid360_link");   // 下游按雷达系 + 扫描时刻查 TF
  EXPECT_EQ(out.header.stamp.sec, 7);
  EXPECT_FLOAT_EQ(At(out, 0)[3], 4.f);             // 其它字段原样带走
  EXPECT_FLOAT_EQ(At(out, 1)[0], -2.f);
  EXPECT_FLOAT_EQ(At(out, 1)[3], 6.f);
}

TEST(ScanRangeFilter, ZeroRangeKeepsAllFinite)
{
  auto in = MakeCloud({{0.0f, 0.0f, 0.0f, 1.f}, {1.0f, 0.0f, 0.0f, 2.f}});
  EXPECT_EQ(g1_nav_bridge::FilterByRange(in, 0.0).width, 2u);
}

TEST(ScanRangeFilter, MissingXyzYieldsEmptyCloud)
{
  auto in = MakeCloud({{1.0f, 0.0f, 0.0f, 1.f}});
  in.fields[2].name = "zz";
  auto out = g1_nav_bridge::FilterByRange(in, 0.25);
  EXPECT_EQ(out.width, 0u);
  EXPECT_TRUE(out.data.empty());
}

TEST(ScanRangeFilter, TruncatedDataYieldsEmptyCloud)
{
  auto in = MakeCloud({{1.0f, 0.0f, 0.0f, 1.f}, {2.0f, 0.0f, 0.0f, 1.f}});
  in.data.resize(20);
  EXPECT_EQ(g1_nav_bridge::FilterByRange(in, 0.25).width, 0u);
}

TEST(ScanRangeFilter, OrganizedCloudHonorsRowStepPadding)
{
  // 2x2 有组织点云，每行末尾 8 字节 padding
  auto in = MakeCloud({{1.f, 0.f, 0.f, 1.f}, {0.1f, 0.f, 0.f, 2.f}, {0.f, 3.f, 0.f, 3.f}, {0.f, 0.f, 0.f, 4.f}});
  std::vector<uint8_t> padded;
  for (int r = 0; r < 2; ++r) {
    padded.insert(padded.end(), in.data.begin() + r * 32, in.data.begin() + r * 32 + 32);
    padded.insert(padded.end(), 8, 0xAB);
  }
  in.height = 2; in.width = 2; in.row_step = 40; in.data = padded;
  auto out = g1_nav_bridge::FilterByRange(in, 0.25);
  ASSERT_EQ(out.width, 2u);
  EXPECT_FLOAT_EQ(At(out, 0)[3], 1.f);
  EXPECT_FLOAT_EQ(At(out, 1)[3], 3.f);
}

TEST(ScanRangeFilter, BaseCropDropsPointsNearBaseLink)
{
  // 雷达系 -> base_link：绕 z 转 90°，雷达在 base_link 前 0.1 m、高 1.2 m
  g1_nav_bridge::BaseCrop crop{{0, -1, 0, 1, 0, 0, 0, 0, 1}, {0.1, 0.0, 1.2}, 0.5};
  auto in = MakeCloud({{0.4f, 0.0f, -1.0f, 1.f},    // base_link (0.1, 0.4)：水平 0.41 m，剔除
                       {0.0f, -0.6f, 0.0f, 2.f},    // base_link (0.7, 0)：保留
                       {0.1f, 0.0f, 0.0f, 3.f}});   // 离雷达 0.1 m：近距剔除
  auto out = g1_nav_bridge::FilterByRange(in, 0.25, &crop);
  ASSERT_EQ(out.width, 1u);
  EXPECT_FLOAT_EQ(At(out, 0)[3], 2.f);
  EXPECT_EQ(out.header.frame_id, "mid360_link");   // 坐标系不变
  EXPECT_EQ(g1_nav_bridge::FilterByRange(in, 0.25).width, 2u);   // 不给 crop 时行为不变
}
