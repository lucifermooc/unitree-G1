#include <gtest/gtest.h>

#include <cstring>
#include <vector>

#include "point_filter/livox_point_filter.hpp"

using livox_ros_driver2::msg::CustomMsg;
using livox_ros_driver2::msg::CustomPoint;

namespace
{
CustomPoint P(float x, float y, float z, uint8_t tag = 0, uint8_t refl = 10, uint32_t t = 0, uint8_t line = 0)
{
  CustomPoint p;
  p.x = x; p.y = y; p.z = z; p.tag = tag; p.reflectivity = refl; p.offset_time = t; p.line = line;
  return p;
}

CustomMsg Msg(const std::vector<CustomPoint> & pts)
{
  CustomMsg m;
  m.header.frame_id = "livox_frame";
  m.header.stamp.sec = 7;
  m.points = pts;
  m.point_num = pts.size();
  return m;
}

float F(const sensor_msgs::msg::PointCloud2 & c, size_t i, uint32_t off)
{
  float v;
  std::memcpy(&v, c.data.data() + i * c.point_step + off, 4);
  return v;
}
}  // namespace

TEST(PointFilter, TagLevels)
{
  // bit[1:0] 粘连 / bit[3:2] 微粒 / bit[5:4] 其他；值 2 = 低置信度，默认 max 1 剔除
  auto m = Msg({P(1, 0, 0, 0x00), P(2, 0, 0, 0x01), P(3, 0, 0, 0x02), P(4, 0, 0, 0x08),
      P(5, 0, 0, 0x20), P(6, 0, 0, 0x30), P(7, 0, 0, 0x15)});
  point_filter::Stats st;
  auto out = point_filter::Filter(m, point_filter::Params{}, st);
  ASSERT_EQ(out.width, 3u);   // 0x00、0x01（粘连中置信度）、0x15（三组都是 1）
  EXPECT_FLOAT_EQ(F(out, 0, 0), 1.f);
  EXPECT_FLOAT_EQ(F(out, 1, 0), 2.f);
  EXPECT_FLOAT_EQ(F(out, 2, 0), 7.f);
  EXPECT_EQ(st.glue, 1u);
  EXPECT_EQ(st.particle, 1u);
  EXPECT_EQ(st.other, 2u);    // 0x20（2）和 0x30（3 保留值）
  // 3 = 不过滤
  point_filter::Params all;
  all.max_glue_level = all.max_particle_level = all.max_other_level = 3;
  point_filter::Stats st2;
  EXPECT_EQ(point_filter::Filter(m, all, st2).width, 7u);
}

TEST(PointFilter, InvalidDupBlindHeightStride)
{
  point_filter::Params p;
  p.blind = 0.1; p.z_min = -2.0; p.z_max = 1.0;
  auto m = Msg({P(0, 0, 0),          // 无效：无回波
      P(1, 1, -1.2f),                // 保留（地面点，雷达系 z 为负）
      P(1, 1, -1.2f),                // 与上一个完全相同
      P(0.05f, 0, 0),                // 盲区
      P(1, 0, -2.5f),                // 低于 z_min
      P(1, 0, 1.5f),                 // 高于 z_max
      P(2, 0, 0.5f)});               // 保留
  point_filter::Stats st;
  auto out = point_filter::Filter(m, p, st);
  ASSERT_EQ(out.width, 2u);
  EXPECT_EQ(st.invalid, 1u);
  EXPECT_EQ(st.dup, 1u);
  EXPECT_EQ(st.blind, 1u);
  EXPECT_EQ(st.height, 2u);
  EXPECT_EQ(st.in, 7u);
  EXPECT_EQ(st.out, 2u);

  p.stride = 2;               // 只看下标 0、2、4、6
  point_filter::Stats st2;
  auto out2 = point_filter::Filter(Msg({P(1, 0, 0), P(2, 0, 0), P(3, 0, 0), P(4, 0, 0), P(5, 0, 0)}), p, st2);
  ASSERT_EQ(out2.width, 3u);
  EXPECT_FLOAT_EQ(F(out2, 1, 0), 3.f);
  EXPECT_EQ(st2.stride, 2u);
}

TEST(PointFilter, OutputLayoutKeepsTimeTagLine)
{
  auto m = Msg({P(1.5f, -2.f, 0.25f, 0x01, 200, 12345678u, 3)});
  point_filter::Stats st;
  auto out = point_filter::Filter(m, point_filter::Params{}, st);
  ASSERT_EQ(out.width, 1u);
  EXPECT_EQ(out.height, 1u);
  EXPECT_EQ(out.point_step, point_filter::kPointStep);
  EXPECT_EQ(out.row_step, point_filter::kPointStep);
  EXPECT_EQ(out.data.size(), point_filter::kPointStep);
  EXPECT_EQ(out.header.frame_id, "livox_frame");
  EXPECT_EQ(out.header.stamp.sec, 7);
  EXPECT_FLOAT_EQ(F(out, 0, 0), 1.5f);
  EXPECT_FLOAT_EQ(F(out, 0, 4), -2.f);
  EXPECT_FLOAT_EQ(F(out, 0, 8), 0.25f);
  EXPECT_FLOAT_EQ(F(out, 0, 12), 200.f);
  uint32_t t;
  std::memcpy(&t, out.data.data() + 16, 4);
  EXPECT_EQ(t, 12345678u);
  EXPECT_EQ(out.data[20], 0x01);
  EXPECT_EQ(out.data[21], 3);
  ASSERT_EQ(out.fields.size(), 7u);
  EXPECT_EQ(out.fields[4].name, "offset_time");
}

TEST(PointFilter, PointNumMismatchUsesArraySize)
{
  auto m = Msg({P(1, 0, 0), P(2, 0, 0)});
  m.point_num = 100;          // 驱动字段与数组不一致时不能越界
  point_filter::Stats st;
  EXPECT_EQ(point_filter::Filter(m, point_filter::Params{}, st).width, 2u);
}
