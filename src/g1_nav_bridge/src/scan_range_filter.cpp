// /lightning/registered_scan 近距剔除 + base_link 水平半径剔除后转发给 STVL（算法见 include/g1_nav_bridge/scan_range_filter.hpp）。
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "g1_nav_bridge/scan_range_filter.hpp"

namespace g1_nav_bridge
{

class ScanRangeFilter : public rclcpp::Node
{
public:
  ScanRangeFilter()
  : Node("scan_range_filter")
  {
    const auto in = declare_parameter<std::string>("input_topic", "/lightning/registered_scan");
    const auto out = declare_parameter<std::string>("output_topic", "/lightning/registered_scan_nav");
    min_range_ = declare_parameter<double>("min_range", 0.25);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    // <=0 不按 base_link 剔除；导航 launch 里 = robot_radius
    crop_radius_ = declare_parameter<double>("crop_radius", 0.0);
    if (crop_radius_ > 0.0) {
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    // 与原话题一致用 reliable：best-effort / reliable 订阅者都能连上
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(out, rclcpp::QoS(2));
    // best-effort 订阅：本节点慢了只丢自己的帧，绝不反压 LIO 的发布线程
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {OnScan(*msg);});
    timer_ = create_wall_timer(std::chrono::seconds(10), [this]() {
      if (frames_ == 0) {
        RCLCPP_WARN(get_logger(), "10 s 内没有发出 %s（TF 失败丢帧 %zu）", pub_->get_topic_name(), tf_fail_);
        tf_fail_ = 0;
        return;
      }
      RCLCPP_INFO(get_logger(), "%.1f Hz，近距(<%.2f m)剔除 %.1f%%，%s 水平 <%.2f m 剔除 %.1f 点/帧，"
        "TF 失败丢帧 %zu，最大单帧 %.2f ms",
        frames_ / 10.0, min_range_, 100.0 * near_ / std::max<size_t>(total_, 1), base_frame_.c_str(),
        crop_radius_, static_cast<double>(cropped_) / frames_, tf_fail_, max_ms_);
      frames_ = 0; near_ = 0; cropped_ = 0; total_ = 0; tf_fail_ = 0; max_ms_ = 0.0;
    });
    RCLCPP_INFO(get_logger(), "%s -> %s，剔除距雷达 < %.2f m、%s 水平 < %.2f m 的点",
      in.c_str(), out.c_str(), min_range_, base_frame_.c_str(), crop_radius_);
  }

private:
  void OnScan(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const auto t0 = std::chrono::steady_clock::now();
    BaseCrop crop{};
    const BaseCrop * crop_ptr = nullptr;
    if (crop_radius_ > 0.0) {
      geometry_msgs::msg::TransformStamped tf;
      try {
        // 必须用扫描时刻：走路时躯干摆动，用最新 TF 会把足迹边界甩到别处
        tf = tf_buffer_->lookupTransform(base_frame_, msg.header.frame_id, msg.header.stamp,
            rclcpp::Duration::from_seconds(0.1));
      } catch (const tf2::TransformException & e) {
        // 不剔除就发出去会把身边的点标成体素（正是要修的问题），宁可丢这一帧
        ++tf_fail_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "查不到 %s <- %s，丢帧：%s",
          base_frame_.c_str(), msg.header.frame_id.c_str(), e.what());
        return;
      }
      const auto & q = tf.transform.rotation;
      const tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {crop.R[3 * r + c] = m[r][c];}
      }
      crop.t[0] = tf.transform.translation.x;
      crop.t[1] = tf.transform.translation.y;
      crop.t[2] = tf.transform.translation.z;
      crop.radius = crop_radius_;
      crop_ptr = &crop;
    }
    const size_t n_in = static_cast<size_t>(msg.width) * msg.height;
    const size_t n_range = crop_ptr ? FilterByRange(msg, min_range_).width : 0;
    auto filtered = FilterByRange(msg, min_range_, crop_ptr);
    near_ += crop_ptr ? n_in - n_range : n_in - filtered.width;
    cropped_ += crop_ptr ? n_range - filtered.width : 0;
    total_ += n_in;
    ++frames_;
    pub_->publish(std::move(filtered));
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    if (ms > max_ms_) {max_ms_ = ms;}
  }

  double min_range_, crop_radius_;
  std::string base_frame_;
  size_t frames_ = 0, near_ = 0, cropped_ = 0, total_ = 0, tf_fail_ = 0;
  double max_ms_ = 0.0;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace g1_nav_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_nav_bridge::ScanRangeFilter>());
  rclcpp::shutdown();
  return 0;
}
