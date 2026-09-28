// MID360 CustomMsg（/livox/lidar）-> 剔除后的 PointCloud2（/livox/points）。算法见 include/point_filter/livox_point_filter.hpp。
// 注意：lightning-lm 的 LIO 直接订阅 /livox/lidar，不经过本节点；这里只影响 /livox/points 的使用者。
#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "point_filter/livox_point_filter.hpp"

namespace point_filter
{

class PointFilterNode : public rclcpp::Node
{
public:
  PointFilterNode()
  : Node("point_filter")
  {
    const auto in = declare_parameter<std::string>("input_topic", "/livox/lidar");
    const auto out = declare_parameter<std::string>("output_topic", "/livox/points");
    // 只改 header 里的坐标系名，不变换坐标：输入坐标必须已经是这个系下的
    frame_id_ = declare_parameter<std::string>("frame_id", "");
    p_.stride = static_cast<int>(declare_parameter<int64_t>("point_stride", 1));
    p_.blind = declare_parameter<double>("blind", 0.1);
    p_.z_min = declare_parameter<double>("z_min", -2.0);
    p_.z_max = declare_parameter<double>("z_max", 10.0);
    p_.max_glue_level = static_cast<int>(declare_parameter<int64_t>("max_glue_level", 1));
    p_.max_particle_level = static_cast<int>(declare_parameter<int64_t>("max_particle_level", 1));
    p_.max_other_level = static_cast<int>(declare_parameter<int64_t>("max_other_level", 1));
    // 参数写错就起不来，别带着错参数静默运行
    if (p_.stride < 1) {throw std::invalid_argument("point_stride 必须 >= 1");}
    for (int lv : {p_.max_glue_level, p_.max_particle_level, p_.max_other_level}) {
      if (lv < 0 || lv > 3) {throw std::invalid_argument("max_*_level 必须在 0~3（3 = 不过滤）");}
    }
    if (p_.z_min >= p_.z_max) {throw std::invalid_argument("z_min 必须 < z_max");}

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, rclcpp::QoS(5));
    // best-effort 订阅：本节点慢了只丢自己的帧，不反压雷达驱动（驱动发布端是 reliable 也能连上）
    sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      in, rclcpp::SensorDataQoS(),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {OnScan(*msg);});
    timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {Report();});
    RCLCPP_INFO(get_logger(), "%s -> %s  抽点 1/%d  盲区 %.2f m  雷达系 z [%.2f, %.2f]  "
      "tag 最高保留等级 粘连 %d / 微粒 %d / 其他 %d（3 = 不过滤）  frame_id=%s",
      in.c_str(), out.c_str(), p_.stride, p_.blind, p_.z_min, p_.z_max, p_.max_glue_level,
      p_.max_particle_level, p_.max_other_level, frame_id_.empty() ? "(沿用输入)" : frame_id_.c_str());
  }

private:
  void OnScan(const livox_ros_driver2::msg::CustomMsg & msg)
  {
    const auto t0 = std::chrono::steady_clock::now();
    auto cloud = Filter(msg, p_, st_);
    if (!frame_id_.empty()) {cloud.header.frame_id = frame_id_;}
    pub_->publish(std::move(cloud));
    ++frames_;
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    max_ms_ = std::max(max_ms_, ms);
  }

  void Report()
  {
    if (frames_ == 0) {
      RCLCPP_WARN(get_logger(), "10 s 内没收到 %s（驱动没起？xfer_format 不是 1（CustomMsg）？）",
        sub_->get_topic_name());
      return;
    }
    const double n = static_cast<double>(std::max<size_t>(st_.in, 1));
    auto pct = [n](size_t v) {return 100.0 * v / n;};
    RCLCPP_INFO(get_logger(), "%.1f Hz，每帧 %.0f -> %.0f 点（留 %.1f%%）；剔除：抽点 %.1f%%  "
      "tag[粘连 %.2f%% 微粒 %.2f%% 其他 %.2f%%]  无效 %.1f%%  重复 %.2f%%  盲区 %.1f%%  高度 %.1f%%；"
      "最大单帧 %.2f ms",
      frames_ / 10.0, st_.in / static_cast<double>(frames_), st_.out / static_cast<double>(frames_),
      pct(st_.out), pct(st_.stride), pct(st_.glue), pct(st_.particle), pct(st_.other), pct(st_.invalid),
      pct(st_.dup), pct(st_.blind), pct(st_.height), max_ms_);
    st_ = Stats{};
    frames_ = 0;
    max_ms_ = 0.0;
  }

  Params p_;
  Stats st_;
  std::string frame_id_;
  size_t frames_ = 0;
  double max_ms_ = 0.0;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace point_filter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<point_filter::PointFilterNode>());
  rclcpp::shutdown();
  return 0;
}
