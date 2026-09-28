// D435 标记点云过滤：只给 STVL 的 realsense_mark 用，realsense_clear 继续吃原始点云
// （清除那一路也过滤的话，被删掉的区域就没有视锥清除，会制造新的残影）。
//
// 输入是 decimation 后的深度图（212x120）而不是点云：邻域/帧间一致性需要像素邻接关系。
// 各级按顺序执行，每级一个开关，便于单变量 A/B：
//   1. 量程门限                          d435_mark_filter.hpp  DepthRangeGate
//   2. 4 邻域一致性（Haier Aurora 驱动）   NeighborFilter
//   3. 帧间一致性（Haier Aurora 驱动）     TemporalFilter
//   4. 反投影，按该帧 stamp 查 TF 到 base_link（查不到则本帧跳过 4a、5、6，只输出 1~3 的结果）
//   4a. 机器人自身区域（走路时前摆的手）                InSelfCrop
//   5. 低带曲率去地面（Haier aid_pointcloud_filter） d435_ground_curvature.hpp
//   6. 悬空点剔除（G1 自研）              FilterMarkPoints
// 输出点在相机光学系（与深度图同 frame、同 stamp），STVL 的 obstacle_range 仍是离相机的距离。
#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "g1_nav_bridge/d435_ground_curvature.hpp"
#include "g1_nav_bridge/d435_mark_filter.hpp"

namespace g1_nav_bridge {

class D435MarkFilter final : public rclcpp::Node {
 public:
  D435MarkFilter() : Node("d435_mark_filter") {
    const auto depth = declare_parameter<std::string>("depth_topic", "/camera/camera/depth/image_rect_raw");
    const auto info = declare_parameter<std::string>("info_topic", "/camera/camera/depth/camera_info");
    const auto out = declare_parameter<std::string>("output_topic", "/camera/camera/depth/mark_points");
    // base_link 是重力水平面（躯干倾斜在 base_link->body_link 里），高度/掠射角都按它算。
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    min_depth_ = declare_parameter<double>("min_depth", 0.25);
    max_depth_ = declare_parameter<double>("max_depth", 3.0);
    neighbor_ = declare_parameter<bool>("neighbor_filter", true);
    neighbor_diff_ = declare_parameter<double>("neighbor_depth_diff", 0.08);
    neighbor_min_ = declare_parameter<int>("neighbor_min_count", 3);
    temporal_ = declare_parameter<bool>("temporal_filter", true);
    temporal_diff_ = declare_parameter<double>("temporal_depth_diff", 0.10);
    // 两帧间隔超过它就不做帧间比较（丢帧后拿旧帧比会把整幅图误删）。
    temporal_max_dt_ = declare_parameter<double>("temporal_max_dt", 0.2);

    // 默认关：D435 地面噪声 σ≈2~3cm 时曲率法分不开地面与矮障碍（见 d435_ground_curvature.hpp）。
    ground_ = declare_parameter<bool>("ground_curvature", false);
    gp_.z_min = declare_parameter<double>("ground_z_min", gp_.z_min);
    gp_.z_max = declare_parameter<double>("ground_z_max", gp_.z_max);
    gp_.voxel = declare_parameter<double>("ground_voxel", gp_.voxel);
    gp_.k_search = declare_parameter<int>("ground_k_search", gp_.k_search);
    gp_.threshold = declare_parameter<double>("ground_curvature_threshold", gp_.threshold);

    floating_ = declare_parameter<bool>("floating_filter", false);
    fp_.min_range = declare_parameter<double>("min_range", fp_.min_range);
    fp_.graze_max_deg = declare_parameter<double>("graze_max_deg", fp_.graze_max_deg);
    fp_.min_z = declare_parameter<double>("min_z", fp_.min_z);
    fp_.max_z = declare_parameter<double>("max_z", fp_.max_z);
    fp_.ground_z = declare_parameter<double>("ground_z", fp_.ground_z);
    fp_.max_gap = declare_parameter<double>("max_gap", fp_.max_gap);
    fp_.cell = declare_parameter<double>("cell", fp_.cell);

    // 机器人自身（走路时前摆的手）：base_link 水平 self_crop_radius 内、高度 [z_min, z_max] 的点删掉；<=0 不删
    sp_.radius = declare_parameter<double>("self_crop_radius", 0.0);
    sp_.z_min = declare_parameter<double>("self_crop_z_min", sp_.z_min);
    sp_.z_max = declare_parameter<double>("self_crop_z_max", sp_.z_max);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    // 发布用 reliable：STVL 不论以哪种 QoS 订阅都能匹配。
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, rclcpp::QoS(5));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) { info_ = m; });
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Image::ConstSharedPtr m) { OnDepth(*m); });
    RCLCPP_INFO(get_logger(),
                "%s -> %s  depth[%.2f,%.2f] neighbor=%d(%.2f,%d) temporal=%d(%.2f) "
                "ground=%d(z[%.2f,%.2f) k=%d thr=%.3f) floating=%d self_crop=r%.2f z[%.2f,%.2f]",
                depth.c_str(), out.c_str(), min_depth_, max_depth_, neighbor_, neighbor_diff_,
                neighbor_min_, temporal_, temporal_diff_, ground_, gp_.z_min, gp_.z_max,
                gp_.k_search, gp_.threshold, floating_, sp_.radius, sp_.z_min, sp_.z_max);
    st_.t0 = now();
  }

 private:
  // 16UC1（mm）或 32FC1（m）→ 米；不支持的编码返回 false。
  static bool ToMeters(const sensor_msgs::msg::Image& m, std::vector<float>& g) {
    const size_t w = m.width, h = m.height;
    g.assign(w * h, 0.0f);
    if (m.encoding == "16UC1" || m.encoding == "mono16") {
      for (size_t r = 0; r < h; ++r)
        for (size_t c = 0; c < w; ++c) {
          uint16_t v;
          std::memcpy(&v, m.data.data() + r * m.step + c * 2, 2);
          if (m.is_bigendian) v = static_cast<uint16_t>((v >> 8) | (v << 8));
          g[r * w + c] = v * 0.001f;
        }
      return true;
    }
    if (m.encoding == "32FC1") {
      for (size_t r = 0; r < h; ++r)
        for (size_t c = 0; c < w; ++c) {
          float v;
          std::memcpy(&v, m.data.data() + r * m.step + c * 4, 4);
          g[r * w + c] = std::isfinite(v) ? v : 0.0f;
        }
      return true;
    }
    return false;
  }

  void OnDepth(const sensor_msgs::msg::Image& m) {
    const rclcpp::Time t_in = now();
    if (!info_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "还没收到 camera_info，丢弃深度帧");
      return;
    }
    std::vector<float> g;
    if (!ToMeters(m, g)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "不支持的深度编码 %s", m.encoding.c_str());
      return;
    }
    const int w = static_cast<int>(m.width), h = static_cast<int>(m.height);
    // camera_info 与深度图分辨率不一致时按比例缩放内参（decimation 前后的 info 都可能收到）。
    const double sx = info_->width ? static_cast<double>(w) / info_->width : 1.0;
    const double sy = info_->height ? static_cast<double>(h) / info_->height : 1.0;
    const double fx = info_->k[0] * sx, fy = info_->k[4] * sy;
    const double cx = info_->k[2] * sx, cy = info_->k[5] * sy;
    if (fx <= 0.0 || fy <= 0.0) return;

    size_t valid_in = 0;
    for (float d : g) valid_in += d != 0.0f;
    st_.in += valid_in;
    st_.range += DepthRangeGate(g, min_depth_, max_depth_);

    if (neighbor_) {
      size_t n = 0;
      g = NeighborFilter(g, w, h, neighbor_diff_, neighbor_min_, &n);
      st_.neighbor += n;
    }
    if (temporal_) {
      const rclcpp::Time stamp(m.header.stamp);
      const bool comparable = prev_.size() == g.size() && prev_frame_ == m.header.frame_id &&
                              (stamp - prev_stamp_).seconds() > 0.0 &&
                              (stamp - prev_stamp_).seconds() <= temporal_max_dt_;
      std::vector<float> pre = g;  // 下一帧要和"帧间剔除之前"比
      if (comparable) st_.temporal += TemporalFilter(g, prev_, temporal_diff_);
      else ++st_.temporal_skip;
      prev_ = std::move(pre);
      prev_stamp_ = stamp;
      prev_frame_ = m.header.frame_id;
    }

    // 反投影到光学系
    std::vector<Vec3> opt;
    opt.reserve(g.size() / 2);
    for (int v = 0; v < h; ++v)
      for (int u = 0; u < w; ++u) {
        const float d = g[static_cast<size_t>(v) * w + u];
        if (d == 0.0f) continue;
        opt.push_back({(u - cx) * d / fx, (v - cy) * d / fy, d});
      }

    std::vector<bool> keep(opt.size(), true);
    const bool self_crop = sp_.radius > 0.0;
    if ((ground_ || floating_ || self_crop) && !opt.empty()) {
      try {
        const auto tf_msg = tf_buffer_->lookupTransform(base_frame_, m.header.frame_id,
                                                        m.header.stamp,
                                                        rclcpp::Duration::from_seconds(0.1));
        tf2::Transform tf;
        tf2::fromMsg(tf_msg.transform, tf);
        std::vector<Vec3> base;
        base.reserve(opt.size());
        for (const Vec3& q : opt) {
          const tf2::Vector3 b = tf * tf2::Vector3(q.x, q.y, q.z);
          base.push_back({b.x(), b.y(), b.z()});
        }
        // 自身区域按扫描时刻的 TF 判定（走路时手随步态摆动，用最新 TF 会把区域甩开）
        if (self_crop) {
          for (size_t i = 0; i < keep.size(); ++i)
            if (InSelfCrop(base[i], sp_)) { keep[i] = false; ++st_.self; }
        }
        // 悬空判定要用含地面的完整点集（地面点是"爬升起点"），所以先于去地面计算。
        if (floating_) {
          const tf2::Vector3 o = tf.getOrigin();
          const auto k = FilterMarkPoints(base, {o.x(), o.y(), o.z()}, fp_);
          for (size_t i = 0; i < keep.size(); ++i)
            if (!k[i]) { keep[i] = false; ++st_.floating; }
        }
        if (ground_) {
          const auto gr = GroundByCurvature(base, gp_);
          for (size_t i = 0; i < keep.size(); ++i)
            if (gr[i] && keep[i]) { keep[i] = false; ++st_.ground; }
        }
      } catch (const tf2::TransformException& e) {
        // 宁可保留地面/假点/自身点，也不能让 D435 标记整体消失。
        ++st_.tf_fail;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF 失败，本帧跳过自身区域/去地面/悬空: %s",
                             e.what());
      }
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = m.header;
    sensor_msgs::PointCloud2Modifier mod(out);
    mod.setPointCloud2FieldsByString(1, "xyz");
    size_t kept = 0;
    for (bool k : keep) kept += k;
    mod.resize(kept);
    sensor_msgs::PointCloud2Iterator<float> ox(out, "x"), oy(out, "y"), oz(out, "z");
    for (size_t i = 0; i < opt.size(); ++i) {
      if (!keep[i]) continue;
      *ox = static_cast<float>(opt[i].x); *oy = static_cast<float>(opt[i].y);
      *oz = static_cast<float>(opt[i].z);
      ++ox; ++oy; ++oz;
    }
    out.is_dense = true;
    pub_->publish(out);

    st_.out += kept;
    ++st_.frames;
    const double ms = (now() - t_in).seconds() * 1e3;
    st_.max_ms = std::max(st_.max_ms, ms);
    const double lat = (now() - rclcpp::Time(m.header.stamp)).seconds() * 1e3;
    st_.max_lat_ms = std::max(st_.max_lat_ms, lat);
    const double since = (now() - st_.t0).seconds();
    if (since >= 10.0) {
      auto pct = [&](size_t n) { return st_.in ? 100.0 * n / st_.in : 0.0; };
      RCLCPP_INFO(get_logger(),
                  "%.1f Hz | 输入有效 %zu 点，剔除：量程 %.1f%% 邻域 %.1f%% 帧间 %.1f%% "
                  "地面 %.1f%% 悬空 %.2f%% 自身 %zu 点 | 输出 %.1f%% | 帧间跳过 %zu 帧 TF失败 %zu 帧 | "
                  "最大耗时 %.1f ms 最大延迟 %.0f ms",
                  st_.frames / since, st_.in, pct(st_.range), pct(st_.neighbor), pct(st_.temporal),
                  pct(st_.ground), pct(st_.floating), st_.self, pct(st_.out), st_.temporal_skip, st_.tf_fail,
                  st_.max_ms, st_.max_lat_ms);
      st_ = Stats{};
      st_.t0 = now();
    }
  }

  struct Stats {
    size_t in = 0, range = 0, neighbor = 0, temporal = 0, ground = 0, floating = 0, self = 0, out = 0;
    size_t frames = 0, temporal_skip = 0, tf_fail = 0;
    double max_ms = 0.0, max_lat_ms = 0.0;
    rclcpp::Time t0{0, 0, RCL_ROS_TIME};
  };

  std::string base_frame_;
  double min_depth_, max_depth_, neighbor_diff_, temporal_diff_, temporal_max_dt_;
  int neighbor_min_;
  bool neighbor_, temporal_, ground_, floating_;
  GroundCurvatureParams gp_;
  MarkFilterParams fp_;
  SelfCropParams sp_;

  std::vector<float> prev_;
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
  std::string prev_frame_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  Stats st_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
};

}  // namespace g1_nav_bridge

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_nav_bridge::D435MarkFilter>());
  rclcpp::shutdown();
  return 0;
}
